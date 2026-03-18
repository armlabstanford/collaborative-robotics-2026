#!/usr/bin/env python3
"""Test script: subscribes to ZMQ on localhost and displays what the bridge sends."""
import zmq
import msgpack
import numpy as np
import cv2
import lz4.frame
import time

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.setsockopt(zmq.SUBSCRIBE, b'')
sub.setsockopt(zmq.CONFLATE, 1)
sub.connect('tcp://127.0.0.1:5555')

print('Waiting for frames from zmq_bridge_node on tcp://127.0.0.1:5555 ...')

count = 0
t0 = time.time()

while True:
    packed = sub.recv()
    frame = msgpack.unpackb(packed, raw=False)

    # Decode RGB
    rgb_arr = np.frombuffer(frame['rgb_jpeg'], dtype=np.uint8)
    bgr = cv2.imdecode(rgb_arr, cv2.IMREAD_COLOR)

    # Decode depth
    h, w = frame['depth_shape']
    depth_bytes = frame['depth']
    if frame.get('depth_lz4', False):
        depth_bytes = lz4.frame.decompress(depth_bytes)
    depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(h, w)

    cam2world = np.array(frame['camera_to_world'])
    pos = cam2world[:3, 3]

    count += 1
    elapsed = time.time() - t0
    fps = count / elapsed if elapsed > 0 else 0

    print(f'[{count}] {fps:.1f} fps | RGB {bgr.shape} | '
          f'Depth {depth.shape} min={depth.min()} max={depth.max()} mm | '
          f'Cam pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] | '
          f'Intrinsics: {frame["intrinsics"]}')

    # Show RGB + depth side by side
    depth_vis = cv2.applyColorMap(
        (depth.clip(0, 5000) / 5000 * 255).astype(np.uint8),
        cv2.COLORMAP_JET)
    combined = np.hstack([bgr, depth_vis])
    cv2.imshow('ZMQ Bridge Test (RGB | Depth)', combined)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
sub.close()
ctx.term()
