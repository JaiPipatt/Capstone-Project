"""
Multi-tag tracker: detection + one Kalman filter per tag id.

  tracker = TagTracker()
  for frame, t_cam in UsbCam('C270').frames():
      poses = tracker.process(frame, t_cam)          # {tag_id: TagPose}

Each TagPose is latency-compensated: extrapolated from the frame's capture time to
"now". All values are in image pixels / radians (no camera calibration yet).
"""
from dataclasses import dataclass
import time

from msrr_localization.tag_detection import TagDetector
from msrr_localization.tag_filter import TagKalmanFilter


@dataclass
class TagPose:
    tag_id: int
    x: float            # px
    y: float            # px (image y, down)
    theta: float        # rad, front direction, 0 = image right, CCW positive
    vx: float           # px/s
    vy: float           # px/s
    omega: float        # rad/s
    detected: bool      # False = predicted only (tag missed in this frame)
    age: float          # s since the last accepted detection


class TagTracker:

    def __init__(self, detector=None, lost_after=0.5, **filter_kwargs):
        """lost_after: seconds without detection before a track is dropped.
        filter_kwargs go to TagKalmanFilter (accel_std, alpha_std, gate_chi2, gate_after)."""
        self.detector = detector or TagDetector()
        self.lost_after = lost_after
        self.filter_kwargs = filter_kwargs
        self.tracks = {}
        self.detections = {}            # raw detections of the last frame (for drawing)
        self.binarized = None           # binarized image of the last frame (for drawing)
        self._clock_offset = None       # min(pc_time - cam_time)

    def process(self, frame, t_cam=None, now=None):
        """Detect, update filters and return {tag_id: TagPose} at time `now`.

        t_cam: capture time of the frame (camera clock). If None, the PC time is used.
        now  : PC time to extrapolate to (default time.time()).
        """
        t_pc = time.time()
        if t_cam is None:
            t_cam = t_pc
        off = t_pc - t_cam
        self._clock_offset = off if self._clock_offset is None else min(self._clock_offset, off)

        self.detections, self.binarized = self.detector.detect(frame, return_binarized=True)

        for kf in self.tracks.values():
            kf.predict(t_cam)
        for tag_id, d in self.detections.items():
            kf = self.tracks.get(tag_id)
            if kf is None:
                self.tracks[tag_id] = TagKalmanFilter(d.x, d.y, d.theta, t_cam,
                                                      **self.filter_kwargs)
            else:
                kf.update(d.x, d.y, d.theta, d.side)
        for tag_id in [i for i, kf in self.tracks.items() if t_cam - kf.t_seen > self.lost_after]:
            del self.tracks[tag_id]

        return self.poses(now)

    def poses(self, now=None):
        """Current poses of all tracks, extrapolated to PC time `now` (default: now)."""
        if self._clock_offset is None:
            return {}
        t = (now if now is not None else time.time()) - self._clock_offset
        out = {}
        for tag_id, kf in self.tracks.items():
            x, y, th, vx, vy, w = kf.state_at(t)
            out[tag_id] = TagPose(tag_id, x, y, th, vx, vy, w,
                                  detected=kf.t_seen == kf.t, age=max(0.0, t - kf.t_seen))
        return out

    def latency(self, t_cam, now=None):
        """Delay (s) of a frame beyond the fastest frame seen so far."""
        if self._clock_offset is None or t_cam is None:
            return 0.0
        return (now if now is not None else time.time()) - self._clock_offset - t_cam
