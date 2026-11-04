"""Derive the SWD ABI and Flash identity from the real ARMCC AXF/DWARF."""
import hashlib
from pathlib import Path
from elftools.elf.elffile import ELFFile


def build_profile(path):
    path = Path(path).resolve()
    with path.open('rb') as f:
        elf = ELFFile(f)
        dies = [d for cu in elf.get_dwarf_info().iter_CUs() for d in cu.iter_DIEs()]
        var = next(d for d in dies if d.tag == 'DW_TAG_variable'
                   and d.attributes.get('DW_AT_name')
                   and d.attributes['DW_AT_name'].value == b'g_debug'
                   and 'DW_AT_location' in d.attributes)
        loc = var.attributes['DW_AT_location'].value
        if loc[0] != 3:
            raise ValueError('g_debug is not an absolute RAM variable')

        def resolve(d):
            while d.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
                d = d.get_DIE_from_attribute('DW_AT_type')
            return d

        def offset(v):
            if isinstance(v, int):
                return v
            if v[0] != 0x23:
                raise ValueError('Unsupported member offset')
            n = shift = 0
            for b in v[1:]:
                n |= (b & 127) << shift
                shift += 7
                if b < 128:
                    break
            return n

        def schema(d):
            d = resolve(d)
            if d.tag == 'DW_TAG_structure_type':
                return {'kind': 'struct', 'size': d.attributes['DW_AT_byte_size'].value,
                        'fields': {m.attributes['DW_AT_name'].value.decode(): {
                            'offset': offset(m.attributes['DW_AT_data_member_location'].value),
                            **schema(m.get_DIE_from_attribute('DW_AT_type'))}
                            for m in d.iter_children() if m.tag == 'DW_TAG_member'}}
            if d.tag == 'DW_TAG_array_type':
                base = schema(d.get_DIE_from_attribute('DW_AT_type'))
                count = next(c.attributes['DW_AT_upper_bound'].value + 1
                             for c in d.iter_children() if c.tag == 'DW_TAG_subrange_type')
                if base['size'] != 1:
                    return {'kind':'array','size':count*base['size'],'count':count,'item':base}
                return {'kind': 'bytes', 'size': count}
            return {'kind': 'float' if d.attributes.get('DW_AT_encoding')
                    and d.attributes['DW_AT_encoding'].value == 4 else 'uint',
                    'size': d.attributes['DW_AT_byte_size'].value}

        result = {'address': int.from_bytes(bytes(loc[1:5]), 'little'),
                  'schema': schema(var.get_DIE_from_attribute('DW_AT_type')),
                  'axf_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                  'source': str(path), 'segments': []}
        for symbol,key in (('g_led','led'),('g_led_cmd','led_command')):
            variable=next((d for d in dies if d.tag=='DW_TAG_variable'
                          and d.attributes.get('DW_AT_name')
                          and d.attributes['DW_AT_name'].value.decode()==symbol
                          and 'DW_AT_location' in d.attributes),None)
            if variable is not None:
                location=variable.attributes['DW_AT_location'].value
                if location[0]!=3:raise ValueError('Unsupported LED variable address')
                result[key]={'address':int.from_bytes(bytes(location[1:5]),'little'),
                             'schema':schema(variable.get_DIE_from_attribute('DW_AT_type'))}
        if 'led_command' in result:
            command=result['led_command']['schema']
            names=['seq','enabled','mode','brightness','speed_percent','color_order',
                   'red','green','blue','cycle_count','cycle_interval_ms','cycle_modes']
            if list(command['fields'])!=names or command['size']!=76:
                raise ValueError('Unsupported LED command ABI')
            for i,name in enumerate(names[:-1]):
                if command['fields'][name]!={'offset':i*4,'kind':'uint','size':4}:
                    raise ValueError('Unsupported LED command field')
            if command['fields']['cycle_modes']!={'offset':44,'kind':'array','size':32,
                    'count':8,'item':{'kind':'uint','size':4}}:
                raise ValueError('Unsupported LED playlist ABI')
            if 'led' not in result or not all(n in result['led']['schema']['fields']
                    for n in ('executed_seq','result','cycle_modes')):
                raise ValueError('LED receipt ABI missing')
        for seg in elf.iter_segments():
            addr = seg['p_paddr']
            if seg['p_type'] == 'PT_LOAD' and 0x08000000 <= addr < 0x08200000 and seg['p_filesz']:
                result['segments'].append({'address': addr, 'data': seg.data().hex()})
        expected = ['seq', 'command', 'confirm_key', 'target_rad_s',
                    'speed_limit_rad_s', 'current_limit_a', 'accel_rad_s2']
        cmd = result['schema']['fields']['cmd']
        if list(cmd['fields']) != expected or cmd['size'] != 28 or cmd['offset'] != 0:
            raise ValueError('Unsupported MotorCommand ABI; writes disabled')
        for i, name in enumerate(expected):
            field = cmd['fields'][name]
            if field['offset'] != i * 4 or field['size'] != 4 or field['kind'] != ('uint' if i < 3 else 'float'):
                raise ValueError('Unexpected MotorCommand field type')
        fields=result['schema']['fields']
        if 'drive_cmd' in fields:
            drive=fields['drive_cmd']
            names=['seq','key','command','motor1_rad_s','motor2_rad_s']
            if drive['size']!=20 or list(drive['fields'])!=names:
                raise ValueError('Unsupported drive command ABI')
            for i,name in enumerate(names):
                if drive['fields'][name]!={'offset':i*4,'kind':'uint' if i<3 else 'float','size':4}:
                    raise ValueError('Unsupported drive command field')
            status=fields.get('drive',{})
            if status.get('size')!=12 or status.get('fields')!={
                    name:{'offset':i*4,'kind':'uint','size':4}
                    for i,name in enumerate(('executed_seq','result','active'))}:
                raise ValueError('Unsupported drive receipt ABI')
        if 'cmd2' in fields:
            secondary=fields['cmd2']
            if secondary['size']!=cmd['size'] or secondary['fields']!=cmd['fields'] or 'motor2' not in fields:
                raise ValueError('Unsupported secondary motor ABI')
            setup=fields.get('setup',{})
            setup_names=['seq','key','command','old_id','new_id','isolated_key',
                         'executed_seq','result','busy','dual_ready']
            if list(setup.get('fields',{}))!=setup_names or setup.get('size')!=40:
                raise ValueError('Unsupported commissioning ABI')
            for i,name in enumerate(setup_names):
                field=setup['fields'][name]
                if field['offset']!=4*i or field['size']!=4 or field['kind']!='uint':
                    raise ValueError('Unsupported commissioning field')
            if fields.get('stop_all_seq',{}).get('kind')!='uint' or fields['stop_all_seq']['size']!=4:
                raise ValueError('Unsupported all-stop ABI')
        if not result['segments']:
            raise ValueError('No Flash identity found')
        return result


def decode(schema, data, offset=0):
    import math
    import struct
    if schema['kind'] == 'struct':
        return {k: decode(v, data, offset + v['offset']) for k, v in schema['fields'].items()}
    if schema['kind']=='array':
        return [decode(schema['item'],data,offset+i*schema['item']['size']) for i in range(schema['count'])]
    raw = data[offset:offset + schema['size']]
    if len(raw) != schema['size']:
        raise ValueError('Truncated memory read')
    if schema['kind'] == 'bytes':
        return list(raw)
    if schema['kind'] == 'float':
        value = struct.unpack('<f', raw)[0]
        return value if math.isfinite(value) else None
    return int.from_bytes(raw, 'little')
