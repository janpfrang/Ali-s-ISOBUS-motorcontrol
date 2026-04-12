"""
ISOBUS Address Claim Verifier - Phase 2.3
Simulates a CCI 100 / tractor ECU that:
  1. Listens for address claim messages (PGN 60928)
  2. Sends Request for Address Claim (PGN 59904) to verify ESP32 responds
  3. Optionally sends a conflicting address claim to test conflict resolution
  4. Also sends vehicle speed (PGN 65256) to keep Phase 2.2 working

Usage:
  python isobus_address_claim_verify.py              # Normal: watch + send speed
  python isobus_address_claim_verify.py --conflict    # Also send conflicting claim
  python isobus_address_claim_verify.py --request     # Send Request for Address Claim
"""

import can
import time
import sys
import argparse
import struct

# --- Configuration ---
BITRATE = 250000
OUR_SA = 0x27           # Simulated tractor/CCI source address
SPEED_INTERVAL = 0.5    # Send speed every 500ms

# --- PGN definitions ---
PGN_ADDRESS_CLAIMED = 60928   # 0xEE00
PGN_REQUEST = 59904           # 0xEA00
PGN_VEHICLE_SPEED = 65256     # 0xFEE8

def make_can_id(priority, pgn, dest_or_ge, sa):
    """Build 29-bit ISOBUS CAN ID."""
    pf = (pgn >> 8) & 0xFF
    if pf < 240:
        ps = dest_or_ge  # PDU1: destination address
    else:
        ps = pgn & 0xFF  # PDU2: group extension
    dp = (pgn >> 16) & 0x01
    return (priority << 26) | (dp << 24) | (pf << 16) | (ps << 8) | sa

def extract_pgn(can_id):
    """Extract PGN from 29-bit CAN ID."""
    pf = (can_id >> 16) & 0xFF
    ps = (can_id >> 8) & 0xFF
    dp = (can_id >> 24) & 0x01
    if pf >= 240:
        return (dp << 16) | (pf << 8) | ps
    else:
        return (dp << 16) | (pf << 8)

def name_to_str(data):
    """Format 8-byte NAME as hex string."""
    return " ".join(f"{b:02X}" for b in data)

def decode_name(data):
    """Decode 64-bit ISOBUS NAME fields."""
    val = int.from_bytes(data, byteorder='little')
    identity = val & 0x1FFFFF
    mfr_code = (val >> 21) & 0x7FF
    ecu_inst = (val >> 32) & 0x07
    func_inst = (val >> 35) & 0x1F
    function = (val >> 40) & 0xFF
    reserved = (val >> 48) & 0x01
    dev_class = (val >> 49) & 0x7F
    dev_class_inst = (val >> 56) & 0x0F
    industry = (val >> 60) & 0x07
    self_config = (val >> 63) & 0x01
    return {
        "self_config": self_config,
        "industry_group": industry,
        "dev_class_instance": dev_class_inst,
        "dev_class": dev_class,
        "function": function,
        "func_instance": func_inst,
        "ecu_instance": ecu_inst,
        "mfr_code": mfr_code,
        "identity": identity,
    }

def build_speed_msg(speed_kmh, sa):
    """Build PGN 65256 vehicle speed message."""
    raw = int(speed_kmh * 256.0 + 0.5)
    can_id = make_can_id(6, PGN_VEHICLE_SPEED, 0, sa)
    data = [raw & 0xFF, (raw >> 8) & 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
    return can.Message(arbitration_id=can_id, data=data, is_extended_id=True)

def build_request_for_address_claim(sa):
    """Build Request for Address Claimed (PGN 59904 requesting PGN 60928)."""
    # PGN 59904: PDU1, PF=0xEA, dest=0xFF (global)
    can_id = make_can_id(6, PGN_REQUEST, 0xFF, sa)
    # Data = requested PGN in 3 bytes little-endian
    pgn_bytes = PGN_ADDRESS_CLAIMED.to_bytes(3, byteorder='little')
    return can.Message(arbitration_id=can_id, data=list(pgn_bytes), is_extended_id=True)

def build_conflicting_claim(sa, target_sa):
    """Build a fake address claim from our SA claiming target_sa's address."""
    # NAME with higher identity number (= lower priority, ESP32 should win)
    fake_name = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x82, 0x32, 0xA0]
    can_id = make_can_id(6, PGN_ADDRESS_CLAIMED, 0xFF, target_sa)
    return can.Message(arbitration_id=can_id, data=fake_name, is_extended_id=True)

# --- Main ---
def main():
    parser = argparse.ArgumentParser(description="ISOBUS Address Claim Verifier")
    parser.add_argument("--conflict", action="store_true",
                        help="Send conflicting address claim (ESP32 should win)")
    parser.add_argument("--request", action="store_true",
                        help="Send Request for Address Claimed")
    parser.add_argument("--speed", type=float, default=8.0,
                        help="Simulated speed in km/h (default 8.0)")
    args = parser.parse_args()

    print("=" * 55)
    print("ISOBUS Address Claim Verifier - Phase 2.3")
    print("=" * 55)

    try:
        import libusb_package
    except ImportError:
        pass

    try:
        bus = can.interface.Bus(interface="gs_usb", channel=0, bitrate=BITRATE)
        print("Connected to Canable!\n")
    except Exception as e:
        print(f"FAILED: {e}")
        sys.exit(1)

    print("Listening for address claims...")
    print(f"Sending speed: {args.speed:.1f} km/h")
    if args.conflict:
        print("Will send CONFLICTING claim after 5 seconds")
    if args.request:
        print("Will send REQUEST for address claim after 3 seconds")
    print("-" * 55)

    last_speed_time = 0
    request_sent = False
    conflict_sent = False
    esp32_sa = None
    start_time = time.time()

    try:
        while True:
            now = time.time()
            elapsed = now - start_time

            # Send speed periodically
            if (now - last_speed_time) >= SPEED_INTERVAL:
                last_speed_time = now
                msg = build_speed_msg(args.speed, OUR_SA)
                bus.send(msg)

            # Send Request for Address Claim after 3 seconds
            if args.request and not request_sent and elapsed > 3.0:
                print("\n>>> Sending Request for Address Claimed (global)")
                msg = build_request_for_address_claim(OUR_SA)
                bus.send(msg)
                request_sent = True
                print(f"    CAN ID: 0x{msg.arbitration_id:08X}")
                print(f"    Data: {' '.join(f'{b:02X}' for b in msg.data)}\n")

            # Send conflicting claim after 5 seconds
            if args.conflict and not conflict_sent and elapsed > 5.0 and esp32_sa is not None:
                print(f"\n>>> Sending CONFLICTING claim for SA=0x{esp32_sa:02X}")
                msg = build_conflicting_claim(OUR_SA, esp32_sa)
                bus.send(msg)
                conflict_sent = True
                print(f"    Fake NAME: {' '.join(f'{b:02X}' for b in msg.data)}")
                print("    (Higher identity = lower priority, ESP32 should win)\n")

            # Receive
            rx = bus.recv(timeout=0.05)
            if rx is not None and rx.is_extended_id:
                pgn = extract_pgn(rx.arbitration_id)
                sa = rx.arbitration_id & 0xFF

                if pgn == PGN_ADDRESS_CLAIMED:
                    esp32_sa = sa
                    name_hex = name_to_str(rx.data)
                    fields = decode_name(rx.data)
                    print(f"RX Address Claim from SA=0x{sa:02X}")
                    print(f"   NAME: {name_hex}")
                    print(f"   Decoded: industry={fields['industry_group']}"
                          f" class={fields['dev_class']}"
                          f" func={fields['function']}"
                          f" mfr={fields['mfr_code']}"
                          f" id={fields['identity']}"
                          f" self_config={fields['self_config']}")
                    print()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()
        print("Bus shut down.")

if __name__ == "__main__":
    main()
