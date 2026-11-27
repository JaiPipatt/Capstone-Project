"""
Floor calibration for tag_pose_node: fit the image -> floor homography.

Put 4 or more AprilTags at known positions, at the SAME HEIGHT as the tags on the
modules (e.g. on spare modules, or the modules themselves), spread over the area
the modules drive in, not all on one line. Then:

  ros2 run msrr_localization tag_calibrate --device C270 \\
      --points 1:0:0 2:0.6:0 3:0.6:0.4 4:0:0.4

Each point is <tag id>:<x m>:<y m> in the world frame you want (x, y on the floor,
z up). Use the same --width/--height as tag_pose_node: H only fits that resolution.
Writes ~/.ros/msrr_floor_homography.yaml (change with --out) and prints the fit error.
"""
import argparse
import sys

import numpy as np

from msrr_localization.floor import fit_homography
from msrr_localization.tag_detection import TagDetector
from msrr_localization.usbcam import UsbCam


def parse_point(text):
    tag_id, x, y = text.split(':')
    return int(tag_id), (float(x), float(y))


def main(argv=None):
    ap = argparse.ArgumentParser(description='Fit the image -> floor homography.')
    ap.add_argument('--points', nargs='+', required=True, help='<tag id>:<x m>:<y m>, 4 or more')
    ap.add_argument('--frames', type=int, default=30, help='frames to average')
    ap.add_argument('--out', default='~/.ros/msrr_floor_homography.yaml')
    ap.add_argument('--device', default='C270',
                    help='webcam name (part of), /dev/videoN or index')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--exposure', type=int, default=-1,
                    help='exposure in 100 us units (lower = less blur, darker); -1 = auto')
    args, _ = ap.parse_known_args(argv)

    world = dict(parse_point(p) for p in args.points)
    if len(world) < 4:
        sys.exit('need at least 4 points')

    cam = UsbCam(args.device, args.width, args.height)
    cam.configure(exposure=None if args.exposure < 0 else args.exposure)
    detector = TagDetector()

    seen = {tag_id: [] for tag_id in world}
    size = None
    print(f'averaging {args.frames} frames, tags {sorted(world)}')
    for n, (frame, _) in enumerate(cam.frames(), 1):
        size = (frame.shape[1], frame.shape[0])
        found = detector.detect(frame)
        for tag_id in world:
            if tag_id in found:
                seen[tag_id].append((found[tag_id].x, found[tag_id].y))
        print(f'\rframe {n}/{args.frames}  seen: '
              + ' '.join(f'{i}:{len(v)}' for i, v in sorted(seen.items())), end='', flush=True)
        if n >= args.frames:
            break
    print()

    missing = [i for i, v in seen.items() if not v]
    if missing:
        sys.exit(f'tags never detected: {missing}')
    ids = sorted(seen)
    image_pts = [np.mean(seen[i], axis=0) for i in ids]
    world_pts = [world[i] for i in ids]
    floor = fit_homography(image_pts, world_pts, size)

    print('tag   image px          world m           fit error')
    errors = []
    for i, (u, v), (x, y) in zip(ids, image_pts, world_pts):
        fx, fy = floor.point(u, v)
        err = np.hypot(fx - x, fy - y)
        errors.append(err)
        print(f'{i:3d}   ({u:6.1f}, {v:6.1f})   ({x:+.3f}, {y:+.3f})   {err * 1000:5.1f} mm')
    print(f'mean error {np.mean(errors) * 1000:.1f} mm '
          '(always 0 with exactly 4 points: add more to check)')

    floor.save(args.out, points={int(i): list(world[i]) for i in ids})
    print(f'saved {args.out} for {size[0]}x{size[1]} images')


if __name__ == '__main__':
    main()
