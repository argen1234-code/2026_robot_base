"""Local-only dashboard; subscribe to tiny numeric metrics, never images."""
import argparse
import asyncio
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'audit_runtime'))
import websockets

LOCK = threading.Lock()
STATE = {'connected': False, 'metrics': {}, 'last_received': None}


async def collect(bridge_url):
    while True:
        try:
            async with websockets.connect(
                bridge_url,
                subprotocols=['foxglove.sdk.v1', 'foxglove.websocket.v1'],
                proxy=None, max_size=30_000_000, max_queue=16, open_timeout=6,
            ) as ws:
                channels = {}
                with LOCK:
                    STATE.update(connected=True, metrics={}, last_received=None)
                async for message in ws:
                    if isinstance(message, str):
                        data = json.loads(message)
                        if data.get('op') == 'advertise':
                            subscriptions = []
                            for channel in data['channels']:
                                if (channel['topic'].startswith('/performance/')
                                        and channel['schemaName'] == 'std_msgs/msg/Float32'
                                        and channel['id'] not in channels):
                                    channels[channel['id']] = channel['topic']
                                    subscriptions.append({'id': channel['id'], 'channelId': channel['id']})
                            if subscriptions:
                                await ws.send(json.dumps({'op':'subscribe', 'subscriptions':subscriptions}))
                        elif data.get('op') == 'unadvertise':
                            for identifier in data.get('channelIds', []):
                                topic = channels.pop(identifier, None)
                                if topic:
                                    with LOCK:
                                        STATE['metrics'].pop(topic, None)
                        continue
                    if len(message) < 21 or message[0] != 1:
                        continue
                    identifier = struct.unpack_from('<I', message, 1)[0]
                    if identifier not in channels:
                        continue
                    payload = message[13:]
                    endian = '<' if payload[1] & 1 else '>'
                    value = struct.unpack_from(endian+'f', payload, 4)[0]
                    with LOCK:
                        STATE['metrics'][channels[identifier]] = value if math.isfinite(value) else None
                        STATE['last_received'] = time.monotonic()
        except Exception as error:
            with LOCK:
                STATE['connected'] = False
            print('Metrics reconnect:', type(error).__name__, flush=True)
            await asyncio.sleep(2)


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.split('?',1)[0] == '/api/frame-rates':
            with LOCK:
                snapshot = {'connected':STATE['connected'], 'metrics':dict(STATE['metrics']),
                            'age_seconds':round(time.monotonic()-STATE['last_received'],2) if STATE['last_received'] is not None else None}
            body = json.dumps(snapshot, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        return super().do_GET()

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8877)
    parser.add_argument('--bridge-url', default='ws://192.168.31.127:8765')
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1',args.port), partial(Handler,directory=str(ROOT/'web')))
    worker = threading.Thread(target=lambda:asyncio.run(collect(args.bridge_url)),daemon=True)
    worker.start()
    print(f'Frame rate monitor: http://127.0.0.1:{args.port}/',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
