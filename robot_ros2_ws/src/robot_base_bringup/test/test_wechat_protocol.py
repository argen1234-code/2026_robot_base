import base64
import struct
import sys
import unittest
import zlib
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from wechat_map import map_payload
from wechat_protocol import parse_command


class WechatProtocolTest(unittest.TestCase):
    def test_direction_speed_is_bounded(self):
        self.assertEqual(parse_command(b'{"command":"FORWARD","speed":250}'), ('FORWARD', 1.0))
        self.assertEqual(parse_command(b'{"command":"LEFT","speed":-5}'), ('LEFT', 0.0))
        self.assertEqual(parse_command(b'{"command":"INDOOR_RECORD_POINT"}'),
                         ('INDOOR_RECORD_POINT', None))

    def test_mission_waypoints_are_validated(self):
        self.assertEqual(parse_command(b'{"command":"INDOOR_MISSION_START","waypoints":[{"x":1,"y":2}]}')[0],
                         'INDOOR_MISSION_START')
        with self.assertRaises(ValueError):
            parse_command(b'{"command":"INDOOR_MISSION_START","waypoints":[{"x":NaN,"y":2}]}')
        with self.assertRaises(ValueError):
            parse_command(b'{"command":"INDOOR_MISSION_START","patrol_mode":"LOOP","waypoints":[{"x":1,"y":2}]}')

    def test_map_payload_encodes_flipped_grayscale_and_origin(self):
        orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        origin = SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-2.0), orientation=orientation)
        info = SimpleNamespace(width=2, height=2, resolution=0.05, origin=origin)
        msg = SimpleNamespace(info=info, data=[0, 100, -1, 50],
                              header=SimpleNamespace(frame_id='map'))
        payload = map_payload(msg, 4)
        self.assertEqual((payload['width'], payload['height'], payload['revision']), (2, 2, 4))
        self.assertEqual((payload['origin_x'], payload['origin_y'], payload['origin_yaw']), (-1.0, -2.0, 0.0))
        png = base64.b64decode(payload['png_base64'])
        self.assertEqual(png[:8], b'\x89PNG\r\n\x1a\n')
        offset = 8
        chunks = {}
        while offset < len(png):
            size = struct.unpack('>I', png[offset:offset + 4])[0]
            kind = png[offset + 4:offset + 8]
            body = png[offset + 8:offset + 8 + size]
            chunks[kind] = body
            offset += size + 12
        self.assertEqual(struct.unpack('>II', chunks[b'IHDR'][:8]), (2, 2))
        pixels = zlib.decompress(chunks[b'IDAT'])
        self.assertEqual(list(pixels), [0, 205, 128, 0, 254, 0])


if __name__ == '__main__':
    unittest.main()
