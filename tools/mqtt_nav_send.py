#!/usr/bin/env python3
"""从命令行发一个 MQTT 导航目标（模拟 AI 侧），用于测试 ingress。

用法：
    python3 tools/mqtt_nav_send.py --x 1.0 --y 0.0 --yaw 0.0
    python3 tools/mqtt_nav_send.py --x 1.0 --y 0.0 --wait 5

读取 .env 的 AI_MQTT_* 凭据；密码不会打印。
"""

import argparse
import json
import os
import sys
import time
import uuid

import paho.mqtt.client as mqtt

ENV_FILE = os.environ.get('ENV_FILE', '/home/pi/2026_robot_base/.env')
HOST_CA_FALLBACK = (
    '/home/pi/2026_robot_base/ai_ws/src/largemodel/config/emqxsl-ca.crt')


def load_env(path):
    env = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                key, value = line.split('=', 1)
                env[key] = value
    except OSError:
        pass
    return env


def get(env, key, default=None):
    return os.environ.get(key, env.get(key, default))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--yaw', type=float, default=0.0, help='rad')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--wait', type=float, default=4.0,
                    help='等待 ack/status 的秒数')
    args = ap.parse_args()

    env = load_env(ENV_FILE)
    broker = get(env, 'AI_MQTT_BROKER', get(env, 'NAV_MQTT_BROKER'))
    port = int(get(env, 'AI_MQTT_PORT', '8883'))
    user = get(env, 'AI_MQTT_USERNAME')
    password = get(env, 'AI_MQTT_PASSWORD')
    ca = get(env, 'AI_MQTT_CA_CERTS')
    if not (ca and os.path.exists(ca)):
        ca = get(env, 'NAV_MQTT_CA_CERTS')
    if not (ca and os.path.exists(ca)):
        ca = HOST_CA_FALLBACK
    goal_topic = get(env, 'AI_MQTT_GOAL_TOPIC', 'robot/ai/nav_goal')
    status_topic = get(env, 'AI_MQTT_STATUS_TOPIC', 'robot/nav/status')
    if not all((broker, user, password)):
        print('AI_MQTT_BROKER/USERNAME/PASSWORD 未配置', file=sys.stderr)
        return 1

    statuses = []

    def on_connect(client, _u, _f, rc):
        if rc == 0:
            client.subscribe(status_topic, qos=1)
            payload = {
                'frame_id': args.frame,
                'x': args.x,
                'y': args.y,
                'yaw': args.yaw,
                'request_id': str(uuid.uuid4()),
            }
            client.publish(goal_topic,
                           json.dumps(payload, separators=(',', ':')),
                           qos=1)
            print(f'published {goal_topic}: {payload}')
        else:
            print(f'connect failed rc={rc}', file=sys.stderr)

    def on_message(_c, _u, msg):
        statuses.append(msg.payload.decode(errors='replace'))

    c = mqtt.Client(client_id=f'nav_send_{os.getpid()}', clean_session=True)
    c.username_pw_set(user, password)
    if ca and os.path.exists(ca):
        c.tls_set(ca_certs=ca)
    else:
        c.tls_set()
    c.tls_insecure_set(False)
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect_async(broker, port, 30)
    c.loop_start()
    time.sleep(max(0.1, args.wait))
    c.loop_stop()
    c.disconnect()
    print('status replies:')
    for s in statuses:
        print(' ', s[:300])
    return 0


if __name__ == '__main__':
    sys.exit(main())
