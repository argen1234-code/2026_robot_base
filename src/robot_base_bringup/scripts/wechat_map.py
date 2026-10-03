"""Encode ROS occupancy grids for the mini-program map topic."""

import base64
import math
import struct
import zlib


def _png_chunk(kind, content):
    body = kind + content
    return struct.pack('>I', len(content)) + body + struct.pack('>I', zlib.crc32(body) & 0xffffffff)


def map_payload(msg, revision, max_dimension=320):
    width, height = msg.info.width, msg.info.height
    if not width or not height or len(msg.data) != width * height or max_dimension < 1:
        raise ValueError('invalid map')
    stride = max(1, math.ceil(max(width, height) / max_dimension))
    out_w, out_h = math.ceil(width / stride), math.ceil(height / stride)
    rows = bytearray()
    for row in range(out_h):
        rows.append(0)
        source_y = height - 1 - row * stride
        for column in range(out_w):
            cell = msg.data[source_y * width + column * stride]
            rows.append(205 if cell < 0 else 0 if cell >= 65 else max(1, 254 - int(cell * 2.53)))
    png = (b'\x89PNG\r\n\x1a\n' +
           _png_chunk(b'IHDR', struct.pack('>IIBBBBB', out_w, out_h, 8, 0, 0, 0, 0)) +
           _png_chunk(b'IDAT', zlib.compress(bytes(rows), 6)) + _png_chunk(b'IEND', b''))
    q = msg.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return {'type': 'indoor_map', 'revision': revision, 'frame_id': msg.header.frame_id or 'map',
            'width': out_w, 'height': out_h, 'resolution': msg.info.resolution * stride,
            'origin_x': msg.info.origin.position.x, 'origin_y': msg.info.origin.position.y,
            'origin_yaw': yaw, 'png_base64': base64.b64encode(png).decode('ascii')}
