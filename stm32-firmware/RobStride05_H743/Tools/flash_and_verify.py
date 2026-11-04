"""Flash the FreeRTOS firmware and verify it from real hardware over SWD.

The script never sends a motor command: it only programs the image and then
reads g_debug, exactly like the console does. Motor enable/run stays a manual,
explicit operator action in the console.

Steps
 1. back up the current Flash image (0x08000000, 128 KiB) to Tools/backup/
 2. program the .hex of the AXF, then compare every Flash load segment
 3. reset and let the firmware run
 4. attach again and check boot_stage/fatal/heartbeats/CCR/coherence

Usage
  python Tools/flash_and_verify.py                 # flash + verify
  python Tools/flash_and_verify.py --no-flash      # verify only
  python Tools/flash_and_verify.py --axf OTHER.axf
Exit code 0 means every check passed.

Requirements: pyocd and pyelftools (the console's Python environment has both),
and a CMSIS-DAP/ST-Link probe that is not held by Keil.
"""
import argparse
import hashlib
import json
import os
import struct
import sys
import time
from pathlib import Path

# A CMSIS-DAP v2 probe needs libusb. Prefer the wheel bundled next to Python
# before pyocd enumerates probes, otherwise pyocd falls back to the v1 (HID)
# interface, which some clones do not implement completely.
try:
    import libusb_package

    _dll = Path(libusb_package.get_library_path()).parent
    os.add_dll_directory(str(_dll))
    os.environ['PATH'] = str(_dll) + os.pathsep + os.environ.get('PATH', '')
except Exception:
    pass

from elftools.elf.elffile import ELFFile

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_AXF = PROJECT / 'MDK-ARM' / 'Objects' / 'RobStride05_H743.axf'
BACKUP_DIR = PROJECT / 'Tools' / 'backup'
FLASH_BASE = 0x08000000
BACKUP_SIZE = 0x00020000
BOOT_STAGE_READY = 9


def resolve(die):
    while die.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
        die = die.get_DIE_from_attribute('DW_AT_type')
    return die


def member_offset(value):
    if isinstance(value, int):
        return value
    if value[0] != 0x23:
        raise ValueError('unsupported member offset encoding')
    number = shift = 0
    for byte in value[1:]:
        number |= (byte & 0x7F) << shift
        shift += 7
        if byte < 128:
            break
    return number


def member_layout(die, path):
    """Byte offset of a nested struct member, e.g. ('motor', 'snapshot_seq')."""
    offset = 0
    die = resolve(die)
    for name in path:
        member = next(child for child in die.iter_children()
                      if child.tag == 'DW_TAG_member'
                      and child.attributes['DW_AT_name'].value.decode() == name)
        offset += member_offset(member.attributes['DW_AT_data_member_location'].value)
        die = resolve(member.get_DIE_from_attribute('DW_AT_type'))
    return offset


def decode(die, data, offset=0):
    die = resolve(die)
    if die.tag == 'DW_TAG_structure_type':
        out = {}
        for member in die.iter_children():
            if member.tag != 'DW_TAG_member':
                continue
            name = member.attributes['DW_AT_name'].value.decode()
            at = member_offset(member.attributes['DW_AT_data_member_location'].value)
            out[name] = decode(member.get_DIE_from_attribute('DW_AT_type'), data, offset + at)
        return out
    if die.tag == 'DW_TAG_array_type':
        count = next(child.attributes['DW_AT_upper_bound'].value + 1
                     for child in die.iter_children() if child.tag == 'DW_TAG_subrange_type')
        base = resolve(die.get_DIE_from_attribute('DW_AT_type'))
        width = base.attributes['DW_AT_byte_size'].value
        return [decode(base, data, offset + index * width) for index in range(count)]
    size = die.attributes['DW_AT_byte_size'].value
    raw = data[offset:offset + size]
    if len(raw) != size:
        raise ValueError('truncated g_debug read')
    encoding = die.attributes.get('DW_AT_encoding')
    if encoding is not None and encoding.value == 4:
        return struct.unpack('<f', raw)[0]
    return int.from_bytes(raw, 'little')


def variable_info(variable):
    location = variable.attributes['DW_AT_location'].value
    if location[0] != 3:
        raise ValueError('variable is not at an absolute address')
    address = int.from_bytes(bytes(location[1:5]), 'little')
    die = resolve(variable.get_DIE_from_attribute('DW_AT_type'))
    return {'address': address, 'size': die.attributes['DW_AT_byte_size'].value, 'type': die}


def load_axf(path):
    raw = path.read_bytes()
    with path.open('rb') as handle:
        elf = ELFFile(handle)
        segments = [{'address': seg['p_paddr'], 'data': seg.data()}
                    for seg in elf.iter_segments()
                    if seg['p_type'] == 'PT_LOAD' and FLASH_BASE <= seg['p_paddr'] < FLASH_BASE + BACKUP_SIZE
                    and seg['p_filesz']]
        wanted = {}
        for unit in elf.get_dwarf_info().iter_CUs():
            for die in unit.iter_DIEs():
                if die.tag == 'DW_TAG_variable' and 'DW_AT_location' in die.attributes:
                    name = die.attributes.get('DW_AT_name')
                    key = name.value.decode() if name is not None else ''
                    if key in ('g_debug', 'g_led') and key not in wanted:
                        wanted[key] = die
        if 'g_debug' not in wanted:
            raise ValueError('g_debug not found in ' + str(path))
        debug_info = variable_info(wanted['g_debug'])
        led_info = variable_info(wanted['g_led']) if 'g_led' in wanted else None
    if not segments:
        raise ValueError('no Flash load segment in ' + str(path))
    return {'path': path, 'sha256': hashlib.sha256(raw).hexdigest(), 'segments': segments,
            'address': debug_info['address'], 'size': debug_info['size'], 'type': debug_info['type'],
            'seq_offset': member_layout(debug_info['type'], ('motor', 'snapshot_seq')),
            'led': led_info}


def session_open(probe_id=None, attach=True):
    from pyocd.core.helpers import ConnectHelper
    return ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id, blocking=False, target_override='stm32h743xx',
        connect_mode='attach' if attach else 'halt', frequency=1000000,
        options={'resume_on_disconnect': False, 'auto_unlock': False, 'allow_no_cores': False,
                 'cache.enable_memory': False, 'cache.enable_register': False,
                 'cmsis_dap.limit_packets': True, 'cmsis_dap.deferred_transfers': False})


def pick_probe():
    from pyocd.core.helpers import ConnectHelper
    probes = [{'id': probe.unique_id, 'name': probe.description}
              for probe in ConnectHelper.get_all_connected_probes(blocking=False)]
    if not probes:
        raise SystemExit('no debug probe found; close any Keil debug session and retry')
    return probes[0]


def program(firmware, probe, hex_path, keil):
    """Use this board's verified Keil algorithm; pyOCD remains read-only.

    The pyOCD generic H743 flash algorithm failed with IPSR=3 on this probe.
    Back up first, release SWD, then let Keil erase/program/verify and run.
    """
    import subprocess
    if not keil.is_file():
        raise SystemExit(f'Keil not found: {keil}')
    project=firmware['path'].parents[1]/(firmware['path'].stem+'.uvprojx')
    if not project.is_file() or hex_path.resolve()!=firmware['path'].with_suffix('.hex'):
        raise SystemExit('Use the AXF/HEX from the matching Keil project')
    session=session_open(probe['id'])
    try:
        session.open()
        BACKUP_DIR.mkdir(parents=True,exist_ok=True)
        save=BACKUP_DIR/f"flash_before_{time.strftime('%Y%m%d-%H%M%S')}.bin"
        save.write_bytes(bytes(session.target.read_memory_block8(FLASH_BASE,BACKUP_SIZE)))
        print(f'backup  : {save} ({BACKUP_SIZE} bytes)')
    finally:
        session.close()
    log=PROJECT/'Docs/keil_flash_verify.log'
    startup=subprocess.STARTUPINFO()
    startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
    result=subprocess.run([str(keil),'-f',str(project),'-j0','-o',str(log)],
                          startupinfo=startup,timeout=60)
    text=log.read_text(encoding='utf-8',errors='replace')
    if result.returncode or 'Verify OK' not in text or 'Application running' not in text:
        raise SystemExit('Keil download failed; inspect '+str(log))
    print('program : Keil Erase Done / Programming Done / Verify OK / Application running')


def sample(firmware, probe, count=3, interval=0.6):
    from pyocd.core.target import Target
    readings = []
    session = session_open(probe['id'])
    try:
        session.open()
        target = session.target
        if target.get_state() != Target.State.RUNNING:
            raise RuntimeError('CPU is not RUNNING; verify does not resume the board')
        for segment in firmware['segments']:
            if bytes(target.read_memory_block8(segment['address'],len(segment['data'])))!=segment['data']:
                raise RuntimeError('Full Flash identity mismatch')
        time.sleep(0.4)
        for _ in range(count):
            # The firmware republishes the motor snapshot every 20 ms and a
            # 396-byte SWD read can straddle one publication, so retry exactly
            # like the console instead of reporting a false tear.
            for _attempt in range(12):
                raw = bytes(target.read_memory_block8(firmware['address'], firmware['size']))
                snapshot = decode(firmware['type'], raw)
                sequence = target.read32(firmware['address'] + firmware['seq_offset'])
                snapshot['_coherent'] = not snapshot['motor']['snapshot_seq'] & 1 and \
                    snapshot['motor']['snapshot_seq'] == sequence
                if snapshot['_coherent']:
                    break
            if firmware.get('led'):
                led = firmware['led']
                raw_led = bytes(target.read_memory_block8(led['address'], led['size']))
                snapshot['_led'] = decode(led['type'], raw_led)
            readings.append(snapshot)
            time.sleep(interval)
    finally:
        session.close()
    return readings


def check(readings, motor_expected=True):
    problems = []
    first, last = readings[0], readings[-1]
    for name, reading in (('first', first), ('last', last)):
        system = reading['system']
        if system['fatal']:
            problems.append(f"{name}: system.fatal=0x{system['fatal']:X}")
        if system['boot_stage'] != BOOT_STAGE_READY:
            problems.append(f"{name}: boot_stage={system['boot_stage']} (expected {BOOT_STAGE_READY})")
        if system['vtor'] != FLASH_BASE:
            problems.append(f"{name}: vtor=0x{system['vtor']:08X} (expected 0x08000000)")
        if system['ccr'] & ((1 << 16) | (1 << 17)):
            problems.append(f"{name}: CCR=0x{system['ccr']:X} has I/D cache bits set")
    heartbeats = [reading['system']['loop_heartbeat'] for reading in readings]
    if len(set(heartbeats)) != len(heartbeats):
        problems.append(f'loop_heartbeat did not advance: {heartbeats}')
    ticks = [reading['system']['tick_ms'] for reading in readings]
    if len(set(ticks)) != len(ticks):
        problems.append(f'tick_ms did not advance: {ticks}')
    if not all(reading['_coherent'] for reading in readings):
        problems.append('g_debug.motor snapshot was torn in at least one sample')
    motor = last['motor']
    if motor['health_fault']:
        problems.append(f"health_fault=0x{motor['health_fault']:X}")
    if motor['cpu_dcache_enabled']:
        problems.append('cpu_dcache_enabled is set although this image never enables caches')
    if motor_expected:
        if not motor['online'] or motor['feedback_age_ms'] > 150:
            problems.append(f"motor offline: online={motor['online']} age={motor['feedback_age_ms']} ms")
        if last['motor']['can']['rx'] <= first['motor']['can']['rx']:
            problems.append('CAN RX counter did not advance')
    if '_led' in first and '_led' in last:
        led_first, led_last = first['_led'], last['_led']
        if not led_last['ready']:
            problems.append(f"WS2812 not ready: last_error={led_last['last_error']}")
        elif led_last['frames_started'] <= led_first['frames_started']:
            problems.append('WS2812 task did not send any frame')
    return problems


def summary(readings):
    last = readings[-1]
    system, motor, oled = last['system'], last['motor'], last['oled']
    return {
        'boot_stage': system['boot_stage'], 'fatal': system['fatal'],
        'vtor': hex(system['vtor']), 'ccr': hex(system['ccr']),
        'loop_heartbeat': [reading['system']['loop_heartbeat'] for reading in readings],
        'tick_ms': [reading['system']['tick_ms'] for reading in readings],
        'health_heartbeat': system['health_heartbeat'],
        'max_service_us': system['max_service_us'], 'last_service_us': system['last_service_us'],
        'motor': {key: motor[key] for key in ('state', 'online', 'enabled', 'motor_state',
                                              'feedback_age_ms', 'snapshot_seq', 'result', 'job_command')},
        'can': {key: motor['can'][key] for key in ('rx', 'tx_ok', 'tx_fail', 'rx_drop', 'errors', 'bus_off')},
        'oled': {key: oled[key] for key in ('heartbeat', 'ready', 'refresh_ok', 'refresh_fail', 'last_error')},
        'led': ({key: last['_led'][key] for key in ('ready', 'busy', 'active_mode', 'frames_started',
                                                    'frames_done', 'errors', 'last_error')}
                if '_led' in last else None),
        'coherent': [reading['_coherent'] for reading in readings],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--axf', type=Path, default=DEFAULT_AXF)
    parser.add_argument('--hex', type=Path)
    parser.add_argument('--no-flash', action='store_true')
    parser.add_argument('--keil',type=Path,default=Path('C:/Keil_v5/UV4/UV4.exe'))
    parser.add_argument('--no-motor-check', action='store_true',
                        help='do not fail when the motor is unpowered or unplugged')
    parser.add_argument('--json', type=Path, default=PROJECT / 'Tools' / 'verify_result.json')
    arguments = parser.parse_args()

    axf = arguments.axf.resolve()
    if not axf.exists():
        raise SystemExit(f'AXF not found: {axf}')
    firmware = load_axf(axf)
    hex_path = arguments.hex or axf.with_suffix('.hex')
    probe = pick_probe()
    print(f"probe   : {probe['name']} ({probe['id']})")
    print(f"axf     : {axf} (sha256 {firmware['sha256'][:16]}...)")
    print(f"g_debug : 0x{firmware['address']:08X}, {firmware['size']} bytes")

    if not arguments.no_flash:
        if not hex_path.exists():
            raise SystemExit(f'hex not found: {hex_path}')
        if hex_path.stat().st_mtime < axf.stat().st_mtime:
            raise SystemExit('hex is older than the AXF; rebuild the project first')
        program(firmware, probe, hex_path, arguments.keil)
        time.sleep(0.3)

    readings = sample(firmware, probe)
    problems = check(readings, motor_expected=not arguments.no_motor_check)
    result = {'ok': not problems, 'problems': problems, 'axf': str(axf),
              'axf_sha256': firmware['sha256'], 'probe': probe, 'samples': summary(readings)}
    arguments.json.parent.mkdir(parents=True, exist_ok=True)
    arguments.json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary(readings), ensure_ascii=False, indent=2))
    if problems:
        print('FAILED:')
        for problem in problems:
            print(' -', problem)
        return 1
    print('PASSED: firmware runs with caches off and the console ABI is live.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
