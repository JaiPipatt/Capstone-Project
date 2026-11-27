"""
USB webcam client (V4L2), same interface as EspCam.

  cam = UsbCam('C270', 1280, 720)        # device name, /dev/videoN or index
  cam.configure(exposure=100)            # 100 us units; None = auto exposure
  for frame, t_cam in cam.frames():      # t_cam is None: the tracker uses PC time
      ...

The device can be given by (part of) its name, e.g. 'C270', so it is found again
when the /dev/video number changes after replugging.
"""
import glob
import os
import time

import cv2


def find_device(device):
    """'C270' / '/dev/video2' / '2' / 2 -> '/dev/video2' (or the index)."""
    if isinstance(device, int) or str(device).isdigit():
        return int(device)
    if str(device).startswith('/dev/'):
        return device
    for sys_dir in sorted(glob.glob('/sys/class/video4linux/video*')):
        try:
            with open(os.path.join(sys_dir, 'name')) as f:
                name = f.read().strip()
            with open(os.path.join(sys_dir, 'index')) as f:
                index = int(f.read().strip())
        except (OSError, ValueError):
            continue
        if index == 0 and str(device).lower() in name.lower():   # index 0 = capture node
            return '/dev/' + os.path.basename(sys_dir)
    raise RuntimeError(f'no video device named "{device}"')


class UsbCam:

    def __init__(self, device='C270', width=1280, height=720, fps=30):
        self.device = device
        self.size = (int(width), int(height))
        self.fps = fps
        self.exposure = None                      # None = auto

    def configure(self, framesize=None, exposure=None, settle=0.0):
        """exposure = V4L2 exposure_time_absolute in 100 us units; None = auto.
        framesize is ignored (ESP32-CAM only); use width/height instead."""
        self.exposure = exposure

    def _open(self):
        cap = cv2.VideoCapture(find_device(self.device), cv2.CAP_V4L2)
        if not cap.isOpened():
            return cap
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # full fps at 720p
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)                              # newest frame only
        if self.exposure is None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)                       # V4L2: 3 = auto
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)                       # 1 = manual
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        return cap

    def capture(self):
        """Single frame (BGR)."""
        cap = self._open()
        try:
            ok, frame = cap.read()
            return frame if ok else None
        finally:
            cap.release()

    def frames(self, retry_delay=2.0, log=print):
        """Yield (frame_bgr, None) forever; reopens the device on errors."""
        while True:
            try:
                cap = self._open()
            except RuntimeError as e:
                log(f'usbcam: {e}; retrying')
                time.sleep(retry_delay)
                continue
            if not cap.isOpened():
                log(f'usbcam: cannot open {self.device} (in use?); retrying')
                time.sleep(retry_delay)
                continue
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            log(f'usbcam: {self.device} opened at {w}x{h} '
                f'{cap.get(cv2.CAP_PROP_FPS):.0f}fps')
            try:
                while True:
                    ok, frame = cap.read()
                    if not ok:
                        break
                    yield frame, None
            finally:
                cap.release()
            log(f'usbcam: lost {self.device}; retrying')
            time.sleep(retry_delay)
