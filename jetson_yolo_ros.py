#!/usr/bin/env python3
"""IMX477 -> YOLO -> ROS vision observations; diagnostic topics by default.

Run after sourcing /opt/ros/humble/setup.bash. No serial port or cmd_vel
publisher is used. Default topics are /diagnostic/vision/*. Class 11 is the
target used by the supplied reference script; the loaded model's label is
printed so the operator can verify it. This is target detection, not a
certified stop-sign safety function or a motion supervisor.

The new lens HFOV is unknown: angle messages are omitted until --hfov-deg
is supplied. That value must describe the actual output image/crop. Angles
then use an approximate pinhole model (positive to image right), followed
by the reference EMA. Lens distortion and physical mounting are not
calibrated here. --live-vision additionally requires that HFOV value and
publishes to /vision instead; keep motion disabled during integration.
Bounding boxes are normalized to the legacy follower's 640x480 convention.
The JSON debug message also preserves native image dimensions/coordinates.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(Path.home() / 'yolov8n.pt'))
    parser.add_argument('--sensor-id', type=int, default=0)
    parser.add_argument('--class-id', type=int, default=11)
    parser.add_argument('--confidence', type=float, default=0.25)
    parser.add_argument('--device', default='0', help='CUDA index, or cpu')
    parser.add_argument('--seconds', type=float, default=30.0,
                        help='Capture/inference duration; 0 runs until Ctrl+C')
    parser.add_argument('--hfov-deg', type=float, default=None)
    parser.add_argument('--alpha', type=float, default=0.15)
    parser.add_argument('--live-vision', action='store_true',
                        help='Explicitly use /vision topics; requires HFOV')
    args = parser.parse_args(argv)
    if args.sensor_id < 0 or args.class_id < 0:
        parser.error('sensor-id and class-id must be non-negative')
    if not math.isfinite(args.confidence) or not 0 < args.confidence <= 1:
        parser.error('confidence must be in (0, 1]')
    if not math.isfinite(args.alpha) or not 0 < args.alpha <= 1:
        parser.error('alpha must be in (0, 1]')
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('seconds must be finite and non-negative')
    if args.hfov_deg is not None and (
            not math.isfinite(args.hfov_deg) or not 0 < args.hfov_deg < 180):
        parser.error('hfov-deg must be in (0, 180)')
    if args.live_vision and args.hfov_deg is None:
        parser.error('--live-vision requires a verified --hfov-deg value')
    if not Path(args.model).expanduser().is_file():
        parser.error('Local model file not found; no weights will be downloaded')
    args.model = str(Path(args.model).expanduser().resolve())
    return args


def camera_pipeline(sensor_id):
    # Sensor 0 and 1920x1080 mode were verified in the supplied Argus log.
    # This BGR conversion path and 30 fps request still require target testing.
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        'video/x-raw(memory:NVMM),width=1920,height=1080,'
        'format=NV12,framerate=30/1 ! nvvidconv ! '
        'video/x-raw,width=640,height=360,format=BGRx ! '
        'videoconvert ! video/x-raw,format=BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    )


def make_observation(box, confidence, width, height, hfov_deg, previous, alpha):
    """Validate a box and retain its normalized area for the legacy follower."""
    coords = [float(x) for x in box]
    if len(coords) != 4 or not all(math.isfinite(x) for x in coords):
        raise ValueError('Invalid box coordinates')
    if width <= 0 or height <= 0 or not math.isfinite(confidence):
        raise ValueError('Invalid frame size or confidence')
    x1, y1, x2, y2 = coords
    x1, x2 = [max(0.0, min(float(width), x)) for x in (x1, x2)]
    y1, y2 = [max(0.0, min(float(height), y)) for y in (y1, y2)]
    if x2 <= x1 or y2 <= y1 or not 0 <= confidence <= 1:
        raise ValueError('Degenerate box or out-of-range confidence')
    offset = ((x1 + x2) / 2 - width / 2) / (width / 2)
    angle = None
    if hfov_deg is not None:
        raw_angle = math.degrees(math.atan(
            offset * math.tan(math.radians(hfov_deg) / 2)))
        angle = raw_angle if previous is None else (
            (1 - alpha) * previous + alpha * raw_angle)
    legacy = [round(x1 / width * 640), round(y1 / height * 480),
              round(x2 / width * 640), round(y2 / height * 480)]
    return dict(seen=True, confidence=confidence, angle_deg=angle,
                horizontal_offset=offset, box_native=[x1, y1, x2, y2],
                box_640x480=legacy, width=width, height=height,
                direction=('right' if offset > 0.05 else
                           'left' if offset < -0.05 else 'center'))


def run(args):
    # Lazy imports allow --help and offline geometry validation without ROS/GPU.
    import cv2
    import torch
    from ultralytics import YOLO
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import Bool, Float32, Int32MultiArray, String

    gst_line = next((line.strip() for line in cv2.getBuildInformation().splitlines()
                     if 'GStreamer:' in line), '')
    if 'YES' not in gst_line:
        raise RuntimeError(f'This OpenCV has no GStreamer capture support: {gst_line}')
    if args.device != 'cpu' and not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; use --device cpu only for CPU testing')

    prefix = '/vision' if args.live_vision else '/diagnostic/vision'
    model = YOLO(args.model)
    names = model.names
    valid_class = (args.class_id in names if isinstance(names, dict)
                   else args.class_id < len(names))
    if not valid_class:
        raise ValueError('Requested class ID is absent from this model')
    print(f'Model: {args.model}; class {args.class_id}: {names[args.class_id]}', flush=True)
    print(f'Device: {args.device}; ROS prefix: {prefix}; {gst_line}', flush=True)
    print('HFOV unset: no target_angle messages.' if args.hfov_deg is None else
          f'HFOV: {args.hfov_deg} degrees; angle/mounting still need validation.', flush=True)

    rclpy.init(args=[])
    node = rclpy.create_node('jetson_yolo_vision')
    cap = None
    frames = 0
    detections = 0
    started = None
    pubs = {}

    def emit(suffix, cls, value):
        message = cls()
        message.data = value
        pubs[suffix].publish(message)

    def publish(observation):
        seen = bool(observation['seen'])
        emit('target_confidence', Float32, float(observation.get('confidence', 0)))
        emit('target_box', Int32MultiArray, observation.get('box_640x480', []))
        if observation.get('angle_deg') is not None:
            emit('target_angle', Float32, float(observation['angle_deg']))
        emit('direction', String, observation.get('direction', 'unknown'))
        # Lost/error/startup states request camera stop. This is not a motor E-stop.
        emit('stop_active', Bool, not seen)
        emit('target_seen', Bool, seen)
        emit('debug', String, json.dumps(observation, allow_nan=False))

    try:
        for suffix, cls in [('target_seen', Bool), ('stop_active', Bool),
                            ('target_angle', Float32), ('target_confidence', Float32),
                            ('target_box', Int32MultiArray), ('direction', String),
                            ('debug', String)]:
            pubs[suffix] = node.create_publisher(cls, prefix + '/' + suffix, 10)
        publish(dict(seen=False, state='starting'))
        cap = cv2.VideoCapture(camera_pipeline(args.sensor_id), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError('OpenCV could not open the Argus/GStreamer pipeline')
        started = time.monotonic()
        previous = None
        last_log = -math.inf
        while rclpy.ok() and (args.seconds == 0 or time.monotonic() - started < args.seconds):
            ok, frame = cap.read()
            captured = time.monotonic()
            if not ok or frame is None or frame.size == 0:
                raise RuntimeError('Camera frame read failed')
            results = model.predict(frame, device=args.device, classes=[args.class_id],
                                    conf=args.confidence, imgsz=640, verbose=False,
                                    save=False, show=False)
            inference_ms = (time.monotonic() - captured) * 1000
            if not rclpy.ok():
                break
            if len(results) != 1:
                raise RuntimeError('Expected one inference result for one frame')
            boxes = results[0].boxes
            observation = dict(seen=False, state='no_target')
            if boxes is not None and len(boxes):
                scores = boxes.conf.detach().cpu().tolist()
                index = max(range(len(scores)), key=scores.__getitem__)
                box = boxes.xyxy[index].detach().cpu().tolist()
                height, width = frame.shape[:2]
                observation = make_observation(box, float(scores[index]), width, height,
                                               args.hfov_deg, previous, args.alpha)
                previous = observation['angle_deg']
                detections += 1
            else:
                previous = None
            frames += 1
            observation.update(class_id=args.class_id, inference_ms=round(inference_ms, 1))
            publish(observation)
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now - last_log >= 1:
                print(json.dumps(dict(frame=frames, **observation), allow_nan=False), flush=True)
                last_log = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok() and len(pubs) == 7:
                publish(dict(seen=False, state='stopped'))
        finally:
            try:
                if cap is not None:
                    cap.release()
            finally:
                try:
                    node.destroy_node()
                finally:
                    if rclpy.ok():
                        rclpy.shutdown()
        elapsed = time.monotonic() - started if started is not None else 0
        print(json.dumps(dict(summary=True, frames=frames,
                              frames_with_target=detections,
                              elapsed_s=round(elapsed, 2),
                              processed_fps=round(frames / elapsed, 2) if elapsed else 0)),
              flush=True)


def main():
    args = parse_args()
    try:
        run(args)
    except Exception as exc:
        print(f'ERROR: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
