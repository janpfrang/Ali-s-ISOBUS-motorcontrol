"""
Canable Firmware Detector

Plug in your Canable and run this script to determine which firmware it has.
  pip install pyserial
  python detect_canable.py

- If a new COM port appears → slcan firmware (stock)
- If no COM port but USB device appears → candleLight firmware
"""

import serial.tools.list_ports
import sys

print("=" * 50)
print("Canable Firmware Detector")
print("=" * 50)
print()
print("Scanning for serial ports...\n")

ports = serial.tools.list_ports.comports()

if not ports:
    print("No serial ports found.")
    print()
    print("If your Canable is plugged in, it likely has")
    print("candleLight firmware (shows as USB device, not COM port).")
    print()
    print("To use with python-can on Windows with candleLight:")
    print("  1. Install Zadig (https://zadig.akeo.ie/)")
    print("  2. Select your device and install WinUSB driver")
    print("  3. pip install python-can[gs_usb] gs_usb pyusb")
    print("  4. Use bustype='gs_usb' instead of 'slcan'")
    sys.exit(0)

print(f"Found {len(ports)} serial port(s):\n")

canable_found = False
for port in ports:
    print(f"  Port: {port.device}")
    print(f"  Desc: {port.description}")
    print(f"  HWID: {port.hwid}")
    print(f"  VID:PID: {port.vid}:{port.pid}" if port.vid else "  VID:PID: n/a")
    print()

    # Canable typically shows as STMicroelectronics Virtual COM Port
    # or similar USB CDC device
    desc_lower = (port.description or "").lower()
    if any(kw in desc_lower for kw in ["canable", "cantact", "stm", "usb serial", "usb-serial"]):
        canable_found = True
        print(f"  >>> This looks like a Canable! (slcan firmware)")
        print(f"  >>> Use COM port: {port.device}")
        print()

if not canable_found:
    print("No obvious Canable found among serial ports.")
    print("Try unplugging and re-plugging the Canable,")
    print("then run this script again to see which port appears/disappears.")
