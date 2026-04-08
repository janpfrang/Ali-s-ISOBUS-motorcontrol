"""
ISOBUS Vehicle Speed Simulator - Phase 2.2
Simulates PGN 65256 (Vehicle Direction/Speed) on the CAN bus.

Sends ground-based vehicle speed (SPN 1859) at configurable intervals,
mimicking what a CCI 100 or tractor ECU would broadcast.

PGN 65256 (0xFEE8) - Vehicle Direction/Speed:
  29-bit CAN ID = 0x18FEE8xx (priority 6, source address xx)
  Byte 0-1: SPN 1859 - Ground-Based Vehicle Speed
            Little-endian uint16, resolution 1/256 km/h per bit
  Byte 2-3: SPN 1860 - Ground-Based Vehicle Direction (not used, set to N/A)
  Byte 4-7: Other SPNs (set to N/A = 0xFF)

Setup:
  pip install python-can gs-usb pyusb libusb-package

Usage:
  python isobus_speed_sim.py              # Interactive: type speed and press Enter
  python isobus_speed_sim.py --ramp       # Auto ramp 0-20-0 km/h
  python isobus_speed_sim.py --fixed 12.5 # Fixed speed
"""

import can
import time
import sys
import argparse

# --- Configuration -----------------------------------------------------------
BITRATE = 250000       # 250 kbit/s (ISOBUS standard)
SOURCE_ADDRESS = 0x27  # Simulated tractor ECU address
SEND_INTERVAL = 0.1    # 100ms between messages (10 Hz, typical for speed)

# PGN 65256 = 0xFEE8
# 29-bit CAN ID: Priority(3)=6, R=0, DP=0, PF=0xFE, PS=0xE8, SA=source
# Binary: 110 0 0 11111110 11101000 SSSSSSSS
# Hex: 0x18FEE800 | source_address
CAN_ID_PGN_65256 = 0x18FEE800 | SOURCE_ADDRESS

def speed_to_raw(kmh):
    """Convert km/h to SPN 1859 raw value (1/256 km/h per bit)."""
    if kmh < 0:
        return 0
    raw = int(kmh * 256.0 + 0.5)
    if raw > 0xFAFF:  # max valid value
        raw = 0xFAFF
    return raw

def build_pgn_65256(speed_kmh):
    """Build 8-byte payload for PGN 65256."""
    raw_speed = speed_to_raw(speed_kmh)
    data = [
        raw_speed & 0xFF,         # Byte 0: Speed LSB
        (raw_speed >> 8) & 0xFF,  # Byte 1: Speed MSB
        0xFF, 0xFF,               # Byte 2-3: Direction = N/A
        0xFF, 0xFF,               # Byte 4-5: N/A
        0xFF, 0xFF,               # Byte 6-7: N/A
    ]
    return data

def send_speed(bus, speed_kmh):
    """Send one PGN 65256 frame with the given speed."""
    data = build_pgn_65256(speed_kmh)
    msg = can.Message(
        arbitration_id=CAN_ID_PGN_65256,
        data=data,
        is_extended_id=True
    )
    bus.send(msg)
    return msg

# --- Main --------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="ISOBUS Vehicle Speed Simulator")
    parser.add_argument("--ramp", action="store_true",
                        help="Auto ramp speed 0 -> 20 -> 0 km/h")
    parser.add_argument("--fixed", type=float, default=None,
                        help="Send fixed speed (km/h)")
    args = parser.parse_args()

    print("=" * 55)
    print("ISOBUS Vehicle Speed Simulator - Phase 2.2")
    print("=" * 55)
    print(f"PGN 65256 (0xFEE8), SA=0x{SOURCE_ADDRESS:02X}")
    print(f"CAN ID: 0x{CAN_ID_PGN_65256:08X}")
    print(f"Send rate: {1/SEND_INTERVAL:.0f} Hz")
    print()

    # Connect
    try:
        import libusb_package
    except ImportError:
        pass

    try:
        bus = can.interface.Bus(
            interface="gs_usb",
            channel=0,
            bitrate=BITRATE
        )
        print("Connected to Canable!\n")
    except Exception as e:
        print(f"FAILED to connect: {e}")
        sys.exit(1)

    try:
        if args.ramp:
            # --- Ramp mode: 0 -> 20 -> 0 km/h, repeat ---
            print("RAMP MODE: 0 -> 20 -> 0 km/h (2 km/h per second)")
            print("Press Ctrl+C to stop")
            print("-" * 55)

            speed = 0.0
            direction = 1  # 1 = up, -1 = down
            step = 2.0 * SEND_INTERVAL  # 2 km/h per second

            count = 0
            while True:
                send_speed(bus, speed)
                count += 1

                if count % 10 == 0:  # Print every 1 second
                    raw = speed_to_raw(speed)
                    print(f"  Speed: {speed:5.1f} km/h | raw: 0x{raw:04X} "
                          f"({raw}) | msgs: {count}")

                speed += direction * step
                if speed >= 20.0:
                    speed = 20.0
                    direction = -1
                elif speed <= 0.0:
                    speed = 0.0
                    direction = 1

                time.sleep(SEND_INTERVAL)

        elif args.fixed is not None:
            # --- Fixed speed mode ---
            speed = args.fixed
            raw = speed_to_raw(speed)
            print(f"FIXED MODE: {speed:.1f} km/h (raw: 0x{raw:04X})")
            print("Press Ctrl+C to stop")
            print("-" * 55)

            count = 0
            while True:
                send_speed(bus, speed)
                count += 1
                if count % 10 == 0:
                    print(f"  Sent {count} messages at {speed:.1f} km/h")
                time.sleep(SEND_INTERVAL)

        else:
            # --- Interactive mode ---
            print("INTERACTIVE MODE")
            print("Type a speed in km/h and press Enter to start sending.")
            print("Type a new speed to change. Type 'q' to quit.")
            print("-" * 55)

            current_speed = 0.0
            running = False

            import threading
            import queue

            input_queue = queue.Queue()

            def input_thread():
                while True:
                    try:
                        line = input()
                        input_queue.put(line)
                    except EOFError:
                        break

            t = threading.Thread(target=input_thread, daemon=True)
            t.start()

            count = 0
            last_print = 0

            print(f"\nCurrent speed: {current_speed:.1f} km/h (enter new value): ")

            while True:
                # Check for user input
                try:
                    line = input_queue.get_nowait()
                    line = line.strip()
                    if line.lower() == 'q':
                        break
                    try:
                        new_speed = float(line)
                        if new_speed < 0: new_speed = 0
                        if new_speed > 60: new_speed = 60
                        current_speed = new_speed
                        running = True
                        raw = speed_to_raw(current_speed)
                        print(f"  -> Speed set to {current_speed:.1f} km/h "
                              f"(raw: 0x{raw:04X})")
                    except ValueError:
                        print("  Enter a number (km/h) or 'q' to quit")
                except queue.Empty:
                    pass

                # Send frames
                if running:
                    send_speed(bus, current_speed)
                    count += 1
                    now = time.time()
                    if (now - last_print) >= 2.0:
                        last_print = now
                        print(f"  Sending {current_speed:.1f} km/h "
                              f"({count} msgs sent)")

                time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        bus.shutdown()
        print("Bus shut down.")

if __name__ == "__main__":
    main()
