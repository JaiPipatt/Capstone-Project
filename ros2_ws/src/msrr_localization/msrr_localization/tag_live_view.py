"""
Live view of AprilTag tracking from the USB webcam.

  ros2 run msrr_localization tag_live_view --device C270

Thin red arrow = raw detection, thick cyan arrow = Kalman filtered pose
(latency-compensated; grey + PRED when the tag is missed), yellow = where the tag
will be in 0.5 s. Keys: Esc/q quit, b binarized view, r raw overlay.
"""
import argparse
import time

import cv2
import numpy as np

from msrr_localization.tag_tracker import TagTracker
from msrr_localization.usbcam import UsbCam


def _arrow(img, x, y, th, length, color, thick):
    tip = (int(x + length * np.cos(th)), int(y - length * np.sin(th)))
    cv2.arrowedLine(img, (int(x), int(y)), tip, color, thick, tipLength=0.3)


def draw(frame, tracker, poses, show_raw=True):
    if show_raw:
        for d in tracker.detections.values():
            color = (0, 255, 0) if d.source == 'gray' else (255, 0, 255)
            cv2.polylines(frame, [d.corners.astype(int)], True, color, 2)
            _arrow(frame, d.x, d.y, d.theta, 1.1 * d.side, (0, 0, 255), 2)
    for p in poses.values():
        color = (255, 255, 0) if p.detected else (160, 160, 160)
        _arrow(frame, p.x, p.y, p.theta, 70, color, 5)
        cv2.line(frame, (int(p.x), int(p.y)),
                 (int(p.x + 0.5 * p.vx), int(p.y + 0.5 * p.vy)), (0, 255, 255), 2)
        label = f'id {p.tag_id} {np.degrees(p.theta):+.0f}deg {np.hypot(p.vx, p.vy):.0f}px/s'
        cv2.putText(frame, label + ('' if p.detected else ' PRED'),
                    (int(p.x) - 80, int(p.y) + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    return frame


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--device', default='C270',
                    help='webcam name (part of), /dev/videoN or index')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--exposure', type=int, default=-1,
                    help='exposure in 100 us units (lower = less blur, darker); -1 = auto')
    args, _ = ap.parse_known_args(argv)              # ignore ROS args from ros2 run

    cam = UsbCam(args.device, args.width, args.height)
    cam.configure(exposure=None if args.exposure < 0 else args.exposure)
    tracker = TagTracker()

    show_bin, show_raw = False, True
    t_last, fps, last_print = time.time(), 0.0, 0.0
    for frame, t_cam in cam.frames():
        poses = tracker.process(frame, t_cam)
        draw(frame, tracker, poses, show_raw)

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_last, 1e-3)
        t_last = now
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(frame, f'{frame.shape[1]}x{frame.shape[0]} {fps:4.1f}fps  '
                           f'extra latency {tracker.latency(t_cam) * 1000:3.0f}ms  '
                           f'detected {sorted(tracker.detections)}  tracked {sorted(poses)}',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if now - last_print > 0.5 and poses:
            print(' | '.join(
                f'id {p.tag_id}: x={p.x:.0f} y={p.y:.0f} th={np.degrees(p.theta):+.1f}deg '
                f'v=({p.vx:+.0f},{p.vy:+.0f})px/s w={np.degrees(p.omega):+.0f}deg/s'
                + ('' if p.detected else ' [pred]')
                for _, p in sorted(poses.items())), flush=True)
            last_print = now

        view = frame
        if show_bin and tracker.binarized is not None:
            view = np.hstack([frame, cv2.cvtColor(tracker.binarized, cv2.COLOR_GRAY2BGR)])
        scale = min(1.0, 1400 / view.shape[1])
        view = cv2.resize(view, None, fx=scale, fy=scale)
        cv2.imshow('AprilTag live + Kalman (Esc to quit)', view)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):
            break
        if k == ord('b'):
            show_bin = not show_bin
        if k == ord('r'):
            show_raw = not show_raw
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
