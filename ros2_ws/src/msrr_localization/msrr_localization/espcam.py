"""
ESP32-CAM client (embedding/espcam firmware, CameraWebServer endpoints).

  cam = EspCam('192.168.72.114')
  cam.configure(framesize=10, exposure=100)
  for frame, t_cam in cam.frames():      # t_cam = capture time on the camera clock (s)
      ...

The camera serves the MJPEG stream to ONE client at a time: close any browser tab
showing it, or frames() will keep retrying.
"""
import time
import urllib.request

import cv2
import numpy as np

FRAMESIZE_VGA = 8
FRAMESIZE_SVGA = 9
FRAMESIZE_XGA = 10


class EspCam:

    def __init__(self, ip, timeout=5.0):
        self.ip = ip
        self.timeout = timeout

    def control(self, var, val):
        """Set one sensor setting, e.g. control('framesize', 10)."""
        url = f'http://{self.ip}/control?var={var}&val={val}'
        urllib.request.urlopen(url, timeout=self.timeout).read()

    def configure(self, framesize=FRAMESIZE_XGA, exposure=100, settle=1.0):
        """Frame size and exposure. exposure = aec_value in sensor lines (~0.053 ms each);
        short exposure = less motion blur but darker. None keeps auto exposure."""
        self.control('framesize', framesize)
        if exposure is None:
            self.control('aec', 1)
        else:
            self.control('aec', 0)
            self.control('aec_value', exposure)
            self.control('agc', 1)
        time.sleep(settle)

    def capture(self):
        """Single JPEG frame (BGR) via /capture. Works while someone else holds the stream."""
        data = urllib.request.urlopen(f'http://{self.ip}/capture', timeout=self.timeout).read()
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def frames(self, retry_delay=2.0, log=print):
        """Yield (frame_bgr, t_cam) forever; reconnects on errors.

        t_cam is the camera's capture timestamp (X-Timestamp header, seconds on the
        ESP32 clock); None if the firmware does not send it.
        """
        while True:
            try:
                stream = urllib.request.urlopen(f'http://{self.ip}:81/stream',
                                                timeout=self.timeout)
                log(f'espcam: stream connected ({self.ip})')
                buf = b''
                while True:
                    chunk = stream.read(8192)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(b'\xff\xd8')
                    z = buf.find(b'\xff\xd9', a + 2)
                    if a == -1 or z == -1:
                        continue
                    head, jpg, buf = buf[:a], buf[a:z + 2], buf[z + 2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        yield frame, _timestamp(head)
            except Exception as e:  # noqa: BLE001  (network errors of many kinds)
                log(f'espcam: stream unavailable ({e}); is a browser tab holding it? retrying')
                time.sleep(retry_delay)


def _timestamp(header):
    k = header.rfind(b'X-Timestamp:')
    if k == -1:
        return None
    try:
        return float(header[k + 12:].split(b'\r\n')[0])
    except ValueError:
        return None
