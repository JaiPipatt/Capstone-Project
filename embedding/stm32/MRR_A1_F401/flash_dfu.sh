#!/bin/bash
# Build + flash the STM32 over USB DFU WITHOUT pressing BOOT0 / NRST.
# Works for MRR_A1_F411 and MRR_A1_F401 (copy this file into each project folder).
#
# Usage (from the project folder):
#   ./flash_dfu.sh                      auto-detect the STM32 USB serial port
#   ./flash_dfu.sh /dev/cu.usbmodemXXX  use this port
#
# Needs (once):  brew install dfu-util
#                python3 -m venv ~/venvs/stm32 && ~/venvs/stm32/bin/pip install pyserial
# The board must already run firmware that can enter DFU by itself:
#   - this robot firmware (bootloader.c, "1200 baud touch"), or
#   - the functional-test firmware A_!_Func_Test (console command "dfu").
# Otherwise flash it ONCE with BOOT0 + NRST.
set -e
cd "$(dirname "$0")"
NAME="$(basename "$(pwd)")"                 # MRR_A1_F411 or MRR_A1_F401
ELF="build/Debug/${NAME}.elf"
BIN="build/Debug/${NAME}.bin"
PY="${HOME}/venvs/stm32/bin/python3"

in_dfu() { dfu-util -l 2>/dev/null | grep -qi "0483:df11"; }

echo "== 1) Build ${NAME}"
if [ ! -d build/Debug ]; then cmake --preset Debug; fi
cmake --build --preset Debug
arm-none-eabi-objcopy -O binary "$ELF" "$BIN"

echo "== 2) Ask the running firmware to enter DFU (no buttons)"
if in_dfu; then
    echo "   Board is already in DFU mode."
else
    PORT="$1"
    if [ -z "$PORT" ]; then
        # STMicroelectronics USB virtual COM port: VID 0x0483, PID 0x5740
        PORT=$("$PY" -c "
from serial.tools import list_ports
p=[x.device for x in list_ports.comports() if x.vid==0x0483 and x.pid==0x5740 and x.device.startswith('/dev/cu.')]
print(p[0] if len(p)==1 else ('MANY' if p else ''))")
    fi
    if [ "$PORT" = "MANY" ]; then
        echo "   More than one STM32 is plugged in. Run: ./flash_dfu.sh /dev/cu.usbmodemXXXX"
        exit 1
    fi
    if [ -z "$PORT" ]; then
        echo "   No STM32 USB port found. Check the cable (VMware -> Connect to Mac),"
        echo "   or press BOOT0 + NRST once and run this script again."
        exit 1
    fi
    echo "   Port: $PORT"
    # a) robot firmware (bootloader.c): open the port at 1200 baud
    "$PY" -c "import serial,sys; s=serial.Serial(sys.argv[1],1200); s.close()" "$PORT" || true
    for i in $(seq 1 6); do in_dfu && break; sleep 0.5; done
    # b) functional-test firmware (A_!_Func_Test): type the console command "dfu"
    if ! in_dfu && [ -e "$PORT" ]; then
        echo "   No DFU yet -> sending the console command 'dfu' (functional-test firmware)"
        "$PY" -c "import serial,sys,time; s=serial.Serial(sys.argv[1],115200,timeout=1); s.write(b'\r\ndfu\r\n'); s.flush(); time.sleep(0.5); s.close()" "$PORT" || true
    fi
fi

echo "== 3) Wait for the DFU device (STM32 BOOTLOADER, 0483:df11)"
for i in $(seq 1 20); do
    if in_dfu; then break; fi
    sleep 0.5
done
if ! in_dfu; then
    echo "   DFU device not found. Fallback: press BOOT0 + NRST, then run this script again."
    exit 1
fi

echo "== 4) Flash and start the new firmware"
set +e
OUT=$(dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D "$BIN" 2>&1)
echo "$OUT"
set -e
if echo "$OUT" | grep -q "File downloaded successfully"; then
    echo "== Done: new firmware is running."
    echo "   (An 'Error during download get_status' line after this is normal: the board already restarted.)"
else
    echo "== FLASH FAILED (see the dfu-util output above)."
    exit 1
fi
