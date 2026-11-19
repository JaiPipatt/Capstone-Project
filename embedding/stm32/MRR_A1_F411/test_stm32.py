#!/usr/bin/env python3
"""
Bring-up / calibration tool for the STM32 firmware (no ESP32 needed).

Plug the STM32 board's USB-C cable into the PC. It shows up as a virtual COM port
(Windows: Device Manager -> Ports -> "STMicroelectronics Virtual COM Port", e.g. COM7).
The baud rate is ignored on USB. (Alternative: a 3.3 V USB-UART adapter on PA2/PA3.)

  python test_stm32.py COM7 monitor              # watch ALL 4 encoders, send nothing
  python test_stm32.py COM7 duty 0 0.15 5        # open loop, motor 0, 15 % power, 5 s
  python test_stm32.py COM7 duty 1,2 0.15,-0.15 10 # motor 1 at 15%, motor 2 at -15%
  python test_stm32.py COM7 vel  0,1 0.5 10      # speed loop, motors 0 and 1, 0.5 rad/s
  python test_stm32.py COM7 vel  all 0.3 10      # all four motors
  python test_stm32.py COM7 pos  2 0.3 15        # position, motor 2, 0.3 rad
  python test_stm32.py COM7 zero all             # set position of the motors to zero

Motor index:  0 = M1_L (left wheel)   1 = M2_IR (inner right)
              2 = M3_IL (inner left)  3 = M4_R (right wheel)
The last number is the run time in seconds (default 3). Press Ctrl+C any time:
the script always sends STOP before it exits and then prints a short verdict.
Requires: pip install pyserial
"""
import struct
import sys
import time

try:
    import serial
except ImportError:          # lets the self-test import this file without pyserial
    serial = None

BAUD = 115200            # ignored on USB; used only with a USB-UART adapter

MSG_CMD, MSG_ZERO, MSG_STATE = 0x01, 0x04, 0x81
MODE_STOP, MODE_VELOCITY, MODE_POSITION, MODE_DUTY = 0, 1, 2, 3

# อัปเดตลำดับใหม่ให้ตรงกับ: [0]=M1_L [3]=M4_R [2]=M3_IL [1]=M2_IR
NAMES = ["M1_L ", "M2_IR ", "M3_IL", "M4_R "]
CFG = ["M1", "M2", "M3", "M4"]          # suffix used in config.h

# encoder counts per radian of the 48T spur gear = 4 * PPR * gearbox * (48/9) / (2*pi)
# (must match config.h: ENC_PPR=7, MOTOR_GEARBOX_RATIO=298)
COUNTS_PER_RAD = 4 * 7 * 298 * (48 / 9) / (2 * 3.141592653589793)


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return b"\xAA\x55" + body + struct.pack("<H", crc16(body))


class FrameReader:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                self.buf = self.buf[-1:]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            ln = self.buf[3]
            if ln > 48:
                del self.buf[:2]
                continue
            total = 4 + ln + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:4 + ln])
            crc_rx = struct.unpack("<H", self.buf[4 + ln:6 + ln])[0]
            if crc16(body) == crc_rx:
                frames.append((body[0], body[2:]))
                del self.buf[:total]
            else:
                del self.buf[:2]
        return frames


def cmd_frame(modes, targets) -> bytes:
    return build(MSG_CMD, struct.pack("<4B4f", *modes, *targets))


def parse_state(pl: bytes):
    stamp, *rest = struct.unpack("<I4f4fB", pl)
    return stamp, rest[0:4], rest[4:8], rest[8]


def parse_motors(text: str):
    """'0' -> [0];  '0,1' -> [0, 1];  'all' -> [0, 1, 2, 3]"""
    if text.lower() == "all":
        return [0, 1, 2, 3]
    idx = sorted({int(t) for t in text.split(",") if t != ""})
    if not idx or any(i < 0 or i > 3 for i in idx):
        raise ValueError("motor index must be 0..3, e.g. 0   or  0,1   or  all")
    return idx


WD_HELP = (
    "\n!! The STM32 reports wd=1 (watchdog tripped) although this script is sending commands.\n"
    "   -> The board is NOT receiving commands over USB, so no motor can move.\n"
    "   Most likely the 2-line edit in USB_DEVICE/App/usbd_cdc_if.c is missing\n"
    "   (App_UsbRx(Buf, *Len); inside CDC_Receive_FS), or the board runs older firmware.\n"
    "   Reading encoders still works: use  'monitor'  mode.\n"
)


def verdict(what, value_str, motors, log):
    """Print a plain-language result for every motor (what to change in config.h)."""
    print("\n================ RESULT ================")
    if not log:
        print("No data received.")
        return
        
    values_str_list = value_str.split(',')
    if len(values_str_list) == 1:
        target_vals = [float(values_str_list[0])] * 4
    elif len(values_str_list) == len(motors):
        target_vals = [0.0] * 4
        for idx_in_list, m_idx in enumerate(motors):
             target_vals[m_idx] = float(values_str_list[idx_in_list])
    else:
         target_vals = [0.0] * 4 # Default to 0 for print if error
        
    for i in motors:
        pos0 = log[0][1][i]
        pos1 = log[-1][1][i]
        d_pos = pos1 - pos0
        tail = [e for e in log if e[0] >= log[0][0] + 0.6 * (log[-1][0] - log[0][0])]
        v_avg = sum(e[2][i] for e in tail) / max(1, len(tail))
        head = f"{NAMES[i]}: position changed {d_pos:+.4f} rad, steady speed {v_avg:+.3f} rad/s"
        
        current_target = target_vals[i]

        if what in ("duty", "vel") and current_target > 0:
            if abs(d_pos) < 0.003:
                print(f"{head}\n   -> NO MOVEMENT counted. Power off? wheel blocked? encoder wires? STBY pin?")
            elif d_pos < 0:
                print(f"{head}\n   -> WRONG SIGN: a positive command makes the position go DOWN.\n"
                      f"      Fix in config.h:  ENC_SIGN_{CFG[i]}  -> flip it (+1 <-> -1).  (see step 1 of the guide)")
            elif what == "vel":
                ratio = v_avg / current_target
                if ratio > 0.9:
                    print(f"{head}\n   -> OK: reaches the target ({ratio * 100:.0f} %).")
                elif ratio > 0.6:
                    print(f"{head}\n   -> close ({ratio * 100:.0f} % of the target). Tune VEL_KP / VEL_KI, or the power limit is nearly reached.")
                else:
                    print(f"{head}\n   -> TOO SLOW ({ratio * 100:.0f} % of the target). The target is above what the motor can do\n"
                          f"      with DUTY_LIMIT in config.h. Raise DUTY_LIMIT, or ask for a lower speed.")
            else:
                print(f"{head}\n   -> OK: a positive duty makes the position go UP (feedback sign is correct).")
        else:
            print(head)
    print("========================================")


def monitor(ser, reader):
    print("Monitor mode: turn each motor's encoder (rear magnet) by hand and see which index moves.")
    print("counts = encoder counts since power-up (1 motor-shaft turn = 28 counts). Ctrl+C to quit.\n")
    last = None
    t_print = 0.0
    state = None
    try:
        while True:
            for typ, pl in reader.feed(ser.read(256)):
                if typ == MSG_STATE and len(pl) == 37:
                    state = parse_state(pl)
            if state and time.time() >= t_print:
                t_print = time.time() + 0.5
                _, pos, _vel, flags = state
                counts = [round(p * COUNTS_PER_RAD) for p in pos]
                cells = []
                for i in range(4):
                    moved = last is not None and counts[i] != last[i]
                    cells.append(f"{NAMES[i]}={counts[i]:+7d}{' <-- MOVING' if moved else ''}")
                last = counts
                print("  ".join(cells) + f"  | wd={flags & 1}")
            elif state is None and time.time() >= t_print:
                t_print = time.time() + 2.0
                print("...no data from the STM32 yet (wrong COM port? board not running?)")
    except KeyboardInterrupt:
        pass


def main():
    argv = sys.argv
    if len(argv) < 3 or (argv[2] not in ("monitor",) and len(argv) < 4) \
            or (argv[2] in ("duty", "vel", "pos") and len(argv) < 5):
        print(__doc__)
        sys.exit(1)
    if serial is None:
        print("pyserial is missing:  pip install pyserial")
        sys.exit(1)

    port, what = argv[1], argv[2]
    ser = serial.Serial(port, BAUD, timeout=0.01)
    reader = FrameReader()

    if what == "monitor":
        try:
            monitor(ser, reader)
        finally:
            ser.close()
        return

    motors = parse_motors(argv[3])
    if what == "zero":
        mask = sum(1 << i for i in motors)
        ser.write(build(MSG_ZERO, struct.pack("<B", mask)))
        print("zero sent")
        ser.close()
        return

    # --- ส่วนที่ปรับแก้เพื่อรองรับการใส่หลายค่าคั่นด้วยลูกน้ำ ---
    values_str = argv[4].split(',')
    
    if len(values_str) == 1:
        values = [float(values_str[0])] * len(motors)
    elif len(values_str) == len(motors):
        values = [float(v) for v in values_str]
    else:
        print("Error: Number of values must be 1 or match the number of motors.")
        sys.exit(1)

    mode = {"duty": MODE_DUTY, "vel": MODE_VELOCITY, "pos": MODE_POSITION}[what]
    modes = [MODE_STOP] * 4
    targets = [0.0] * 4
    
    for idx, i in enumerate(motors):
        modes[i] = mode
        targets[i] = values[idx]
    # -------------------------------------------------------------

    duration = float(argv[5]) if len(argv) > 5 else 3.0
    t_start = time.time()
    t_end = t_start + duration
    t_next = 0.0
    warned = False
    log = []                                   # (seconds, pos[4], vel[4])
    try:
        while time.time() < t_end:
            if time.time() >= t_next:          # resend at 20 Hz (watchdog is 200 ms)
                ser.write(cmd_frame(modes, targets))
                t_next = time.time() + 0.05
            for typ, pl in reader.feed(ser.read(256)):
                if typ == MSG_STATE and len(pl) == 37:
                    stamp, pos, vel, flags = parse_state(pl)
                    log.append((time.time() - t_start, list(pos), list(vel)))
                    cells = "  ".join(f"{NAMES[i]} p={pos[i]:+9.4f} v={vel[i]:+7.3f}" for i in motors)
                    print(f"t={stamp:8d}ms {cells}  wd={flags & 1}")
                    if (flags & 1) and not warned and time.time() - t_start > 1.5:
                        warned = True
                        print(WD_HELP)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(cmd_frame([MODE_STOP] * 4, [0.0] * 4))   # always leave in STOP
        ser.close()
    verdict(what, argv[4], motors, log)


if __name__ == "__main__":
    main()