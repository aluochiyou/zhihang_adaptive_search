#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import json
import signal
import socket
import struct
import sys
import time
from pathlib import Path
from typing import List

import cv2
import numpy as np


def recv_exact(conn: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        block = conn.recv(size - len(data))
        if not block:
            raise ConnectionError('client disconnected')
        data.extend(block)
    return bytes(data)


def recv_request(conn: socket.socket):
    header_size = struct.unpack('!I', recv_exact(conn, 4))[0]
    if header_size > 2_000_000:
        raise ValueError(f'invalid header size {header_size}')
    header = json.loads(recv_exact(conn, header_size).decode('utf-8'))
    image_size = struct.unpack('!I', recv_exact(conn, 4))[0]
    if image_size > 100_000_000:
        raise ValueError(f'invalid image size {image_size}')
    return header, recv_exact(conn, image_size)


def send_response(conn: socket.socket, payload: dict) -> None:
    raw = json.dumps(payload, separators=(',', ':')).encode('utf-8')
    conn.sendall(struct.pack('!I', len(raw)) + raw)


class FpsWindow:
    def __init__(self, window_seconds: float) -> None:
        self.window_seconds = window_seconds
        self.timestamps = collections.deque(maxlen=2000)

    def add(self) -> None:
        self.timestamps.append(time.monotonic())

    def fps(self) -> float:
        now = time.monotonic()
        while self.timestamps and now - self.timestamps[0] > self.window_seconds:
            self.timestamps.popleft()
        if len(self.timestamps) < 2:
            return 0.0
        return (len(self.timestamps) - 1) / max(
            self.timestamps[-1] - self.timestamps[0], 1e-6
        )

    def count(self) -> int:
        return len(self.timestamps)


class Yolo26SingleWorker:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.vehicle_id = int(args.vehicle_id)
        self.sizes: List[int] = [
            int(value) for value in args.adaptive_sizes.split(',') if value.strip()
        ]
        if not self.sizes:
            raise ValueError('adaptive_sizes must contain at least one integer')
        self.size_index = 0
        self.last_adapt = 0.0
        self.running = True
        self.fps_window = FpsWindow(args.performance_window)

        import torch
        from ultralytics import YOLO

        requested_device = str(args.device).strip()
        if requested_device == 'auto':
            requested_device = '0' if torch.cuda.is_available() else 'cpu'
        if requested_device != 'cpu' and not torch.cuda.is_available():
            raise RuntimeError(
                f'CUDA device {requested_device!r} was requested but CUDA is unavailable'
            )
        torch.set_grad_enabled(False)
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
        self.torch = torch
        self.model = YOLO(str(Path(args.model).expanduser().resolve()))
        self.device = requested_device
        self.quantize = int(args.quantize) if int(args.quantize) > 0 else None
        if self.device == 'cpu' and self.quantize == 16:
            print(
                f'[WARN] v{self.vehicle_id} CPU runtime cannot reliably use '
                '16-bit quantization; quantize -> 32',
                flush=True,
            )
            self.quantize = 32
        self.quantize_supported = True
        self.warmup()

    def predict(self, image, size: int):
        kwargs = {
            'source': image,
            'imgsz': size,
            'conf': self.args.conf,
            'iou': self.args.iou,
            'device': self.device,
            'verbose': False,
        }
        if self.quantize is not None and self.quantize_supported:
            kwargs['quantize'] = self.quantize
        try:
            return self.model.predict(**kwargs)
        except (TypeError, ValueError) as exc:
            if 'quantize' not in kwargs:
                raise
            self.quantize_supported = False
            kwargs.pop('quantize', None)
            print(
                f'[WARN] v{self.vehicle_id} runtime rejected quantize='
                f'{self.quantize}; retry without quantize: {exc}',
                flush=True,
            )
            return self.model.predict(**kwargs)

    def warmup(self) -> None:
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        size = self.sizes[self.size_index]
        for _ in range(max(1, int(self.args.warmup_iterations))):
            self.predict(image, size)
        if self.torch.cuda.is_available():
            self.torch.cuda.synchronize()
        print(
            f'[READY] v{self.vehicle_id} model={self.args.model} '
            f'device={self.device} quantize={self.quantize} '
            f'quantize_supported={self.quantize_supported} imgsz={size}',
            flush=True,
        )

    def maybe_adapt(self) -> None:
        if not self.args.adaptive:
            return
        now = time.monotonic()
        if now - self.last_adapt < self.args.adaptive_check_interval:
            return
        if self.fps_window.count() < max(20, int(self.args.minimum_fps * 2.0)):
            return
        rate = self.fps_window.fps()
        if rate >= self.args.minimum_fps or self.size_index >= len(self.sizes) - 1:
            return
        self.size_index += 1
        self.last_adapt = now
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        size = self.sizes[self.size_index]
        self.predict(image, size)
        if self.torch.cuda.is_available():
            self.torch.cuda.synchronize()
        print(
            f'[ADAPT] v{self.vehicle_id} worker_fps={rate:.2f} < '
            f'{self.args.minimum_fps:.2f}; imgsz -> {size}',
            flush=True,
        )

    def infer(self, header: dict, image_bytes: bytes) -> dict:
        decode_start = time.perf_counter()
        image = cv2.imdecode(
            np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        decode_ms = (time.perf_counter() - decode_start) * 1000.0
        if image is None:
            return {'error': 'JPEG decode failed', 'detections': []}

        size = self.sizes[self.size_index]
        inference_start = time.perf_counter()
        results = self.predict(image, size)
        if self.torch.cuda.is_available():
            self.torch.cuda.synchronize()
        inference_ms = (time.perf_counter() - inference_start) * 1000.0
        self.fps_window.add()
        self.maybe_adapt()
        worker_fps = self.fps_window.fps()

        detections = []
        result = results[0]
        names = result.names
        if result.boxes is not None:
            xyxy = result.boxes.xyxy.detach().cpu().numpy()
            confidences = result.boxes.conf.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy().astype(int)
            for box, score, class_id in zip(xyxy, confidences, classes):
                x1, y1, x2, y2 = map(float, box)
                detections.append(
                    {
                        'class_id': int(class_id),
                        'class_name': str(names.get(int(class_id), class_id)),
                        'confidence': float(score),
                        'xyxy': [x1, y1, x2, y2],
                        'center': [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                    }
                )
        return {
            'vehicle_id': self.vehicle_id,
            'stamp': header.get('stamp'),
            'sequence': header.get('sequence'),
            'decode_ms': decode_ms,
            'inference_ms': inference_ms,
            'worker_fps': worker_fps,
            'imgsz': size,
            'rate_pass': worker_fps >= self.args.minimum_fps,
            'detections': detections,
        }

    def serve(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.args.host, self.args.port))
        server.listen(1)
        server.settimeout(1.0)
        print(
            f'[LISTEN] v{self.vehicle_id} {self.args.host}:{self.args.port}',
            flush=True,
        )
        while self.running:
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            print(f'[CONNECT] v{self.vehicle_id} client={address}', flush=True)
            connection.settimeout(self.args.socket_timeout)
            with connection:
                try:
                    while self.running:
                        header, image_bytes = recv_request(connection)
                        try:
                            response = self.infer(header, image_bytes)
                        except Exception as exc:
                            response = {
                                'error': f'{type(exc).__name__}: {exc}',
                                'detections': [],
                            }
                            print(f'[ERROR] v{self.vehicle_id} inference: {exc}', flush=True)
                        send_response(connection, response)
                except (ConnectionError, BrokenPipeError, ConnectionResetError, socket.timeout) as exc:
                    print(f'[DISCONNECT] v{self.vehicle_id}: {exc}', flush=True)
                except Exception as exc:
                    print(f'[ERROR] v{self.vehicle_id} connection: {exc}', flush=True)
        server.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='One independent portable YOLO worker')
    parser.add_argument('--vehicle-id', type=int, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--device', default='0')
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--iou', type=float, default=0.70)
    parser.add_argument('--quantize', type=int, default=16, choices=[0, 16, 32])
    parser.add_argument('--adaptive', action='store_true', default=True)
    parser.add_argument('--adaptive-sizes', default='640,512,416')
    parser.add_argument('--minimum-fps', type=float, default=10.0)
    parser.add_argument('--performance-window', type=float, default=5.0)
    parser.add_argument('--adaptive-check-interval', type=float, default=5.0)
    parser.add_argument('--warmup-iterations', type=int, default=3)
    parser.add_argument('--socket-timeout', type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    worker = Yolo26SingleWorker(args)

    def stop(_signum, _frame):
        worker.running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    worker.serve()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'[FATAL] {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        raise
