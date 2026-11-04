"""Read the SSD1306 frame buffer over SWD and print what is on the OLED.

The firmware keeps the 8x128 page in `pixels` (Devices/oled_ssd1306.c). This
script locates that array through the AXF DWARF, reads it from the running
target and decodes every 5x7 cell back to a character with the glyph table from
Devices/oled_font.h. It is a read-only check: no command is sent to the motor.

Usage
  python Tools/read_oled_page.py                 # decode the current page
  python Tools/read_oled_page.py --raw           # also dump the 128x64 bitmap

Requires pyocd + pyelftools and a free probe.
"""
import argparse
import re
import sys
import time
from pathlib import Path

import os
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
FONT = PROJECT / 'Devices' / 'oled_font.h'
ROWS, COLS, ADVANCE = 8, 128, 6


def load_glyphs():
    text = FONT.read_text(encoding='utf-8')
    glyphs = {}
    for match in re.finditer(r"\{\s*'(.)'\s*,\s*\{([^}]*)\}\s*\}", text):
        character = match.group(1)
        columns = tuple(int(value, 16) for value in re.findall(r'0x([0-9A-Fa-f]{2})U', match.group(2)))
        if len(columns) == 5:
            glyphs[columns] = character
    if not glyphs:
        raise SystemExit('no glyph parsed from ' + str(FONT))
    return glyphs


def load_pixels_symbol(axf):
    with axf.open('rb') as handle:
        elf = ELFFile(handle)
        dwarf = elf.get_dwarf_info()
        for unit in dwarf.iter_CUs():
            name = unit.get_top_DIE().attributes.get('DW_AT_name')
            if name is None or not name.value.decode().endswith('oled_ssd1306.c'):
                continue
            for die in unit.iter_DIEs():
                if die.tag != 'DW_TAG_variable':
                    continue
                attribute = die.attributes.get('DW_AT_name')
                if attribute is None or attribute.value != b'pixels':
                    continue
                location = die.attributes['DW_AT_location'].value
                if location[0] != 3:
                    raise SystemExit('pixels is not at an absolute address')
                address = int.from_bytes(bytes(location[1:5]), 'little')
                size = die.get_DIE_from_attribute('DW_AT_type')
                count = 1
                while size.tag == 'DW_TAG_array_type':
                    sub = next(child for child in size.iter_children()
                               if child.tag == 'DW_TAG_subrange_type')
                    count *= sub.attributes['DW_AT_upper_bound'].value + 1
                    size = size.get_DIE_from_attribute('DW_AT_type')
                return address, die.attributes['DW_AT_type'].value
    raise SystemExit('pixels not found in ' + str(axf))


def decode_page(pixels, glyphs):
    lines, unknown = [], 0
    for row in range(ROWS):
        base = row * COLS
        text = []
        # A 5-pixel glyph plus one blank column: 128 // 6 = 21 usable columns.
        for column in range(COLS // ADVANCE):
            start = base + column * ADVANCE
            cell = tuple(pixels[start:start + 5])
            character = glyphs.get(cell)
            if character is None:
                character = '?'
                if any(cell):
                    unknown += 1
            text.append(character)
        lines.append(''.join(text).rstrip())
    return lines, unknown


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--axf', type=Path, default=DEFAULT_AXF)
    parser.add_argument('--raw', action='store_true', help='also print the pixel bitmap')
    parser.add_argument('--png', type=Path, help='also save the page as an upscaled PNG')
    arguments = parser.parse_args()

    axf = arguments.axf.resolve()
    if not axf.exists():
        raise SystemExit('AXF not found: ' + str(axf))
    address, type_name = load_pixels_symbol(axf)
    glyphs = load_glyphs()

    from pyocd.core.helpers import ConnectHelper
    probes = [probe.unique_id for probe in ConnectHelper.get_all_connected_probes(blocking=False)]
    if not probes:
        raise SystemExit('no debug probe found')
    with ConnectHelper.session_with_chosen_probe(
            unique_id=probes[0], blocking=False, target_override='stm32h743xx',
            connect_mode='attach', frequency=1000000,
            options={'resume_on_disconnect': False, 'cache.enable_memory': False,
                     'cache.enable_register': False, 'cmsis_dap.limit_packets': True,
                     'cmsis_dap.deferred_transfers': False}) as session:
        target = session.target
        pixels = bytes(target.read_memory_block8(address, ROWS * COLS))
    lines, unknown = decode_page(pixels, glyphs)

    print(f"pixels  : 0x{address:08X} ({type_name})")
    print('+---------------------+')
    for index, text in enumerate(lines):
        print(f'|{text:<21}| row {index}')
    print('+---------------------+')
    if unknown:
        print(f'cells without a matching glyph: {unknown}')
    if arguments.raw:
        print()
        # SSD1306 page format: pixels[page][column], bit0 is the top pixel.
        for y in range(64):
            page, bit = divmod(y, 8)
            print(''.join('#' if pixels[page * COLS + x] & (1 << bit) else '.' for x in range(COLS)))
    if arguments.png:
        render_png(pixels, arguments.png)
        print('png     :', arguments.png)
    return 0


def render_png(pixels, path, scale=5):
    """Save the page as an upscaled image, the way the panel shows it."""
    from PIL import Image, ImageOps

    gray = Image.new('L', (COLS, 64), 0)
    dot = gray.load()
    for page in range(ROWS):
        for column in range(COLS):
            byte = pixels[page * COLS + column]
            for bit in range(8):
                if byte & (1 << bit):
                    dot[column, page * 8 + bit] = 255
    big = gray.resize((COLS * scale, 64 * scale), Image.NEAREST)
    ImageOps.colorize(big, black=(0, 0, 0), white=(120, 220, 255)).save(path)


if __name__ == '__main__':
    sys.exit(main())
