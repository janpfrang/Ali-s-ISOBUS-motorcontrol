# ISOBUS Motor Control — Requirements Specification

**Project:** Proportional DC Motor Control via ISOBUS (ISO 11783)
**Version:** 0.5 — Cleaned
**Date:** 2026-02-18
**Status:** Draft — For Review

---

## 1. Project Overview

A 12V DC motor shall be controlled proportionally to the vehicle ground speed, as reported over the ISOBUS (ISO 11783 / CAN 2.0B at 250 kbit/s). The control unit is an ESP32 NodeMCU microcontroller. A Virtual Terminal (VT) user interface on the CCI 100 terminal provides operator interaction.

---

## 2. Functional Requirements

### FR-01: Motor Speed Control — Proportional to Vehicle Speed

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-01.1 | The motor speed shall be proportional to the vehicle ground speed received via ISOBUS. | Linear interpolation between defined points (see FR-01.3). |
| FR-01.2 | The motor is driven via PWM signal to a Solid State Relay (SSR). | |
| FR-01.3 | The PWM duty cycle mapping shall be: | |
| | — 0 km/h → 0% PWM (motor off) | |
| | — 9 km/h → 45% PWM | |
| | — 18 km/h → 90% PWM | |
| | — 20 km/h → 100% PWM (maximum) | |
| FR-01.4 | Above 20 km/h the PWM duty cycle shall be clamped at 100%. | |
| FR-01.5 | If no vehicle speed data is received for more than **5 seconds**, the motor shall stop (0% PWM). The system shall enter "Stopped" mode. | |
| FR-01.6 | The PWM duty cycle shall be interpolated linearly between the defined mapping points. | |

### FR-02: Start/Stop Control

| ID | Requirement |
|----|-------------|
| FR-02.1 | The operator can **start** proportional motor control via a "Start" button on the Virtual Terminal. |
| FR-02.2 | The operator can **stop** motor control via a "Stop" button on the VT. The motor shall turn off immediately (0% PWM). |
| FR-02.3 | On power-up / boot, the motor shall default to **Stopped** mode. |

### FR-03: Virtual Terminal (VT) User Interface

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-03.1 | Display current vehicle speed (km/h). | Source: ISOBUS ground-based vehicle speed PGN. |
| FR-03.2 | Display current motor speed as **PWM duty cycle (%)**. If a speed sensor is added later, display actual RPM instead. | |
| FR-03.3 | "Start" soft key to enable proportional mode ("Auto"). | |
| FR-03.4 | "Stop" soft key to disable motor and enter "Stopped" mode. | |
| FR-03.5 | Status indicator showing current mode: **"Auto"** (green) or **"Stopped"** (red). | |
| FR-03.6 | Display a **warning/alarm** on the VT if vehicle speed data is lost (no data for >5 s, see FR-01.5). | |

### FR-04: ISOBUS Communication

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-04.1 | The ESP32 shall act as an ISOBUS Working Set (implement ECU) on the CAN bus. | Requires address claiming per ISO 11783-5. |
| FR-04.2 | The ESP32 shall receive vehicle ground speed from the **CCI 100** terminal. | PGN 65256 (0x00FEE8) — Ground-Based Vehicle Speed, SPN 1859. |
| FR-04.3 | The ESP32 shall implement the VT Client protocol (ISO 11783-6) to display the UI on the CCI 100. | Requires object pool upload, Working Set activation, etc. |
| FR-04.4 | The ESP32 shall perform ISO 11783-5 address claim on the bus. | Requires a valid NAME (64-bit identifier). |
| FR-04.5 | CAN bus speed: **250 kbit/s** (ISOBUS standard). | |

---

## 3. Non-Functional Requirements

### NFR-01: Hardware Platform

| ID | Requirement | Notes |
|----|-------------|-------|
| NFR-01.1 | Microcontroller: **ESP32 NodeMCU** dev board. | |
| NFR-01.2 | CAN Transceiver: **SN65HVD230** breakout board. | Suitable for 3.3V logic; supports 250 kbit/s. |
| NFR-01.3 | SSR connected to **GPIO 4** for PWM motor control. | |
| NFR-01.4 | CAN RX → **GPIO 21**, CAN TX → **GPIO 22**. | Standard choice, fine. |
| NFR-01.5 | Motor: **12V DC, 10A rated**. Inrush current not yet known. The SSR shall be rated for ≥12V DC and ≥10A continuous. | **[OPEN]** Inrush current TBD — SSR should be derated or oversized (recommend ≥20A SSR) until inrush is measured. |
| NFR-01.6 | PWM frequency for SSR: **10 Hz**. | At 10 Hz with 8-bit resolution, each step ≈ 0.39 ms — sufficient for motor control. |

### NFR-02: Software / Development Environment

| ID | Requirement | Notes |
|----|-------------|-------|
| NFR-02.1 | Firmware shall be developed using the **Arduino IDE** (Arduino-ESP32 core) or **PlatformIO with ESP-IDF**. | PlatformIO + ESP-IDF is the more flexible option and better suited for AgIsoStack++. |
| NFR-02.2 | An ISOBUS protocol stack library shall be used, such as **AgIsoStack++** or a similar ISO 11783 library. | |
| NFR-02.3 | The VT object pool (UI layout) shall be designed using a tool compatible with ISO 11783-6 (e.g., **ISODesigner** or similar). | Object pools are binary resources uploaded to the VT. |

### NFR-03: Testing Strategy (Phased)

| Phase | Description | Notes |
|-------|-------------|-------|
| **Phase 1** | Desktop simulation: Python script on PC simulates CCI 100 + VT over USB-CAN adapter (TJA1051T/3-based). | The simulator must: (a) respond to address claims, (b) provide vehicle speed PGN, (c) emulate a VT server (complex!). Consider using **python-can** + **python-isobus** or a VT simulator like the open-source **AgIsoVirtualTerminal**. |
| **Phase 2** | Bench test with real CCI 100 terminal (standalone, not in vehicle). | |
| **Phase 3** | In-vehicle test with built-in CCI 100 and real vehicle speed signal. | |

---

## 4. Open Questions & Recommendations

| # | Topic | Question / Recommendation |
|---|-------|--------------------------|
| Q9 | **Python VT simulator complexity** | A full VT simulator in Python is a significant project in itself. Consider using existing tools (e.g., AgIsoVirtualTerminal) or simplify Phase 1 to just CAN speed message simulation without full VT emulation. |
| Q10 | **PWM resolution** | ESP32 LEDC supports 1–16 bit resolution. Recommend 8-bit (256 steps) or 10-bit (1024 steps). Define acceptable granularity. |
| Q11 | **Power supply** | How is the ESP32 powered? USB from CCI 100? Separate 12V→5V/3.3V regulator? |
| Q12 | **Reverse polarity / transient protection** | Vehicle environment requires protection against voltage spikes, reverse polarity, load dump, etc. Is this in scope? |

---

## 5. System Architecture (Summary)

```
┌──────────────────────┐       ISOBUS (CAN 2.0B, 250 kbit/s)
│  CCI 100 Terminal    │◄──────────────────────────────────────┐
│  (VT Server + Speed) │                                       │
└──────────────────────┘                                       │
                                                               │
                                                    CAN H ─────┤
                                                    CAN L ─────┤
                                                               │
                                                  ┌────────────┴───────────┐
                                                  │   SN65HVD230           │
                                                  │   CAN Transceiver      │
                                                  └────┬──────────┬────────┘
                                                  RX(21)│          │TX(22)
                                                  ┌─────┴──────────┴───────┐
                                                  │   ESP32 NodeMCU        │
                                                  │   (ISOBUS WS / ECU)   │
                                                  │                        │
                                                  │   GPIO 4 → PWM ──┐    │
                                                  └───────────────────┼────┘
                                                                      │
                                                               ┌──────┴──────┐
                                                               │   SSR       │
                                                               └──────┬──────┘
                                                                      │
                                                               ┌──────┴──────┐
                                                               │  12V DC     │
                                                               │  Motor      │
                                                               └─────────────┘
```

---

## 6. Evaluation Summary

| Aspect | Assessment |
|--------|------------|
| **Clarity** | Good. Key parameters (speed mapping, pins, hardware, timeouts) are well defined. |
| **Completeness** | Most requirements defined. 4 open questions remain (Q9–Q12). |
| **Feasibility** | Hardware is feasible. ISOBUS stack library (AgIsoStack++) mitigates protocol complexity risk. |
| **Testability** | Phased approach is good. Phase 1 (Python VT sim) scope to be clarified (Q9). |
| **Safety** | Signal loss timeout (5 s) and boot-up default (Stopped) are defined. |

---

*Please review the remaining open questions (Q9–Q12) so this document can be finalized as version 1.0.*
