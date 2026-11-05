#!/usr/bin/env python3
"""主机 MediaPipe 手部检测（独立 venv，不导入 ROS）。

运行：
    env -u PYTHONPATH -u PYTHONHOME \
      /home/pi/mediapipe_plain/bin/python tools/hand_detector_host.py \
      --device 2 --udp 127.0.0.1:5005

每帧把 JSON 发到 UDP，由 tools/hand_pose_bridge.py 转成 ROS 话题。
"""

import argparse
import json
import math
import socket
import time

import cv2
import mediapipe as mp

HAND_HEIGHT_M = 0.20
FX = 600.0
FY = 600.0
CX = 320.0
CY = 240.0
MIN_D = 0.20
MAX_D = 3.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/hand-cam',
                    help='相机设备（稳定别名 /dev/hand-cam，或 /dev/videoN）')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--udp', default='127.0.0.1:5005')
    ap.add_argument('--image-udp', default='',
                    help='JPEG 帧转发目标 host:port（空=关闭，供 Foxglove 查看）')
    ap.add_argument('--image-fps', type=float, default=10.0)
    ap.add_argument('--jpeg-quality', type=int, default=80)
    ap.add_argument('--max-hands', type=int, default=1)
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()

    host, port = args.udp.rsplit(':', 1)
    port = int(port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    image_sock = None
    image_host = image_port = None
    if args.image_udp:
        image_host, image_port = args.image_udp.rsplit(':', 1)
        image_port = int(image_port)
        image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f'hand JPEG -> udp {image_host}:{image_port} '
              f'@ {args.image_fps:.0f} fps q={args.jpeg_quality}', flush=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {args.device}')
    print(f'hand camera {args.device} opened', flush=True)

    hands = mp.solutions.hands.Hands(
        static_image_mode=False,
        max_num_hands=args.max_hands,
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    mp_draw = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands

    last_t = time.time()
    last_send = 0.0
    last_image = 0.0
    last_valid = None
    oversized_logged = False
    try:
        while True:
            ok, frame = cap.read()
            now = time.time()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            dt = max(1e-6, now - last_t)
            last_t = now
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)
            h, w = frame.shape[:2]
            msg = {'valid': False, 'message': 'no_hand',
                   'timestamp': round(now, 3),
                   'frame_id': 'hand_camera_optical_frame'}
            out = frame
            if result.multi_hand_landmarks:
                lm = result.multi_hand_landmarks[0]
                xs = [p.x * w for p in lm.landmark]
                ys = [p.y * h for p in lm.landmark]
                pad = 20
                x1 = int(max(0, min(xs) - pad))
                y1 = int(max(0, min(ys) - pad))
                x2 = int(min(w - 1, max(xs) + pad))
                y2 = int(min(h - 1, max(ys) + pad))
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                hp = max(1.0, float(y2 - y1))
                z = FY * HAND_HEIGHT_M / hp
                z = max(MIN_D, min(MAX_D, z))
                x = (u - CX) * z / FX
                y = (v - CY) * z / FY
                bearing = math.atan2(x, z)
                dist = math.sqrt(x * x + y * y + z * z)
                msg.update({
                    'valid': True, 'message': 'ok',
                    'bbox_px': [x1, y1, x2 - x1, y2 - y1],
                    'center_px': [round(u, 1), round(v, 1)],
                    'cartesian_m': [round(x, 3), round(y, 3), round(z, 3)],
                    'bearing_rad': round(bearing, 3),
                    'distance_m': round(dist, 3),
                    'fps': round(1.0 / dt, 1),
                })
                if args.show:
                    mp_draw.draw_landmarks(out, lm, mp_hands.HAND_CONNECTIONS)
                    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if msg['valid'] != last_valid:
                last_valid = msg['valid']
                print(f"hand {'detected' if last_valid else 'lost'} "
                      f"{msg.get('cartesian_m', '')} "
                      f"fps={msg.get('fps', '-')}", flush=True)
            if (image_sock is not None and
                    now - last_image >= 1.0 / max(0.1, args.image_fps)):
                ok_jpg, buf = cv2.imencode(
                    '.jpg', out,
                    [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
                if ok_jpg:
                    if len(buf) <= 60000:
                        image_sock.sendto(
                            buf.tobytes(), (image_host, image_port))
                    elif not oversized_logged:
                        oversized_logged = True
                        print(f'hand JPEG too large ({len(buf)} B), '
                              'frame dropped', flush=True)
                last_image = now
            if now - last_send >= 0.05:
                sock.sendto(json.dumps(msg).encode('utf-8'), (host, port))
                last_send = now
            if args.show:
                cv2.imshow('hand_detector_host', out)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            time.sleep(max(0.0, 1.0 / args.fps - (time.time() - now)))
    finally:
        cap.release()
        hands.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
