# ISOBUS Motor Control — Requirements Specification

**Project:** Proportional DC Motor Control via ISOBUS (ISO 11783)
**Version:** 1.0
**Date:** 2026-02-22
**Status:** Draft — For Review

---

## 1. Project Overview

A 12V DC motor shall be controlled proportionally to the vehicle ground speed. The system is developed in two stages:

**Stage 1** — Basic standalone motor control with local UI (rotary encoder + LCD display), local speed measurement via reed sensor, and two operating modes (Auto / Manual). No ISOBUS communication. This stage validates hardware, wiring, and core motor control logic.

**Stage 2** — Full ISOBUS integration (ISO 11783 / CAN 2.0B at 250 kbit/s). Vehicle speed is received over ISOBUS from the CCI 100 terminal. A Virtual Terminal (VT) user interface replaces the local LCD for operator interaction. Additional sensors (lift position) are integrated. The local LCD and encoder remain available as a fallback interface.

The control unit for both stages is an ESP32 NodeMCU microcontroller.

---

## 2. Functional Requirements

### FR-01: Motor Speed Control — Proportional to Speed

*Stage 1: Speed from reed sensor. Stage 2: Speed from ISOBUS PGN.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-01.1 | The motor speed shall be proportional to the vehicle ground speed. | Linear interpolation between defined points (see FR-01.3). |
| FR-01.2 | The motor is driven via PWM signal (10 kHz, 8-bit resolution) to a MOSFET driver module. | DollaTek 15A 400W dual-MOSFET module. |
| FR-01.3 | The PWM duty cycle mapping shall be: | Piecewise linear interpolation. |
| | — 0 km/h → 0% PWM (motor off) | |
| | — 9 km/h → 45% PWM | |
| | — 18 km/h → 90% PWM | |
| | — 20 km/h → 100% PWM (maximum) | |
| FR-01.4 | Above 20 km/h the PWM duty cycle shall be clamped at 100%. | |
| FR-01.5 | Stage 1: If no reed sensor pulse is received for >2 seconds, speed shall read 0 km/h. | Software timeout. |
| FR-01.6 | Stage 2: If no ISOBUS vehicle speed data is received for >5 seconds, the motor shall stop (0% PWM) and the system shall enter Stopped mode. | Alarm shown on VT (see FR-03.6). |
| FR-01.7 | Speed values above 40 km/h shall be discarded as noise. | Applies to reed sensor in Stage 1. |

### FR-02: Operating Modes

*Stage 1: Both modes available. Stage 2: Auto mode uses ISOBUS speed source.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-02.1 | The system shall support two operating modes: AUTO and MANUAL. | |
| FR-02.2 | AUTO mode: PWM duty cycle is proportional to measured speed (per FR-01.3 mapping). Stage 1 uses reed sensor speed; Stage 2 uses ISOBUS ground speed. | Stage 1: simple linear 0–20 km/h. Stage 2: piecewise linear per FR-01.3. |
| FR-02.3 | MANUAL mode: PWM duty cycle is set directly by operator via rotary encoder (0–100%). | Encoder value persists when switching modes. |
| FR-02.4 | Mode can be switched at any time, including while the motor is running. | |
| FR-02.5 | On power-up / boot, the system shall default to MANUAL mode, Stopped state. | |

### FR-03: Start/Stop Control

*Stage 1: Via encoder button. Stage 2: Also via VT soft keys.*

| ID | Requirement |
|----|-------------|
| FR-03.1 | The operator can start motor control via a short press on the rotary encoder button (Stage 1) or a Start button on the VT (Stage 2). |
| FR-03.2 | The operator can stop motor control via a short press on the rotary encoder button (Stage 1) or a Stop button on the VT (Stage 2). The motor shall turn off immediately (0% PWM). |
| FR-03.3 | On power-up / boot, the motor shall default to Stopped state (0% PWM output). |

### FR-04: Local User Interface (Stage 1)

*Stage 1 primary interface. Stage 2: retained as fallback.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-04.1 | A 20x4 character I2C LCD shall display system status. | I2C address 0x27 (configurable). |
| FR-04.2 | LCD Row 0: Title and current mode indicator ([AUTO] or [MANUAL]). | |
| FR-04.3 | LCD Row 1: Current PWM duty cycle (%). In AUTO mode, annotated with "(auto)". | |
| FR-04.4 | LCD Row 2: Current speed in km/h (from reed sensor). | |
| FR-04.5 | LCD Row 3: Run state (RUN / STOP) and uptime (MM:SS). | |
| FR-04.6 | Display refresh interval: 250 ms. | |
| FR-04.7 | KY-040 rotary encoder with push button for all local input. | |
| FR-04.8 | Short press (<1s): Toggle START / STOP. | 50 ms debounce. |
| FR-04.9 | Long press (≥1s): Switch between AUTO and MANUAL mode. | |
| FR-04.10 | Rotation: Adjust PWM % in MANUAL mode (0–100, 1% steps). | No effect in AUTO mode. |

### FR-05: Speed Measurement (Stage 1)

*Stage 1 only. Stage 2 replaces with ISOBUS vehicle speed.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-05.1 | Ground speed shall be measured using a reed sensor mounted on a wheel. | One magnet per revolution. |
| FR-05.2 | Wheel diameter: 410 mm (configurable constant). | Circumference ≈ 1.288 m. |
| FR-05.3 | Speed is calculated from the period between consecutive reed sensor pulses. | Interrupt-driven on falling edge. |
| FR-05.4 | Reed sensor debounce: ignore pulses faster than 5 ms. | Prevents false triggers. |
| FR-05.5 | If no pulse is received for >2 seconds, speed shall read 0 km/h. | Wheel stopped or stalled. |
| FR-05.6 | Reed sensor cable length up to 6 m shall be supported. | Requires external 4.7kΩ pull-up + 100nF filter cap. Shielded/twisted pair cable recommended. |

### FR-06: Virtual Terminal (VT) User Interface (Stage 2)

*Stage 2 only.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-06.1 | Display current vehicle speed (km/h) on the CCI 100 VT. | Source: ISOBUS ground-based vehicle speed PGN. |
| FR-06.2 | Display current motor PWM duty cycle (%). If a motor speed sensor is added later, display actual RPM instead. | |
| FR-06.3 | "Start" soft key to enable the active mode (Auto or Manual). | |
| FR-06.4 | "Stop" soft key to disable motor and enter Stopped state. | |
| FR-06.5 | Mode selection control (Auto / Manual) on VT. | |
| FR-06.6 | Status indicator showing current mode: "Auto" (green) or "Manual" (blue) or "Stopped" (red). | |
| FR-06.7 | Display a warning/alarm on the VT if vehicle speed data is lost (>5s, see FR-01.6). | |
| FR-06.8 | Display lift position status: "Lowered" or "Raised" (see FR-07). | |

### FR-07: Lift Position Sensor (Stage 2)

*Stage 2 only.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-07.1 | A sensor shall detect whether the implement is in the lowered (working) or raised (transport) position. | Binary signal: 2 positions only. |
| FR-07.2 | Sensor type and GPIO pin: TBD. | **[OPEN]** Sensor selection pending. |
| FR-07.3 | When the implement is raised, the motor shall stop automatically (0% PWM), regardless of mode. | Safety interlock. |
| FR-07.4 | When the implement is lowered again, the motor shall resume operation in the previously active mode. | No operator action required to restart. |
| FR-07.5 | Lift position shall be displayed on the VT (see FR-06.8) and optionally on the local LCD. | |

### FR-08: ISOBUS Communication (Stage 2)

*Stage 2 only.*

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-08.1 | The ESP32 shall act as an ISOBUS Working Set (implement ECU) on the CAN bus. | Requires address claiming per ISO 11783-5. |
| FR-08.2 | The ESP32 shall receive vehicle ground speed from the CCI 100 terminal. | PGN 65256 (0x00FEE8), SPN 1859. |
| FR-08.3 | The ESP32 shall implement the VT Client protocol (ISO 11783-6) to display the UI on the CCI 100. | Requires object pool upload, Working Set activation. |
| FR-08.4 | The ESP32 shall perform ISO 11783-5 address claim on the bus. | Requires a valid NAME (64-bit identifier). |
| FR-08.5 | CAN bus speed: 250 kbit/s (ISOBUS standard). | |
| FR-08.6 | CAN transceiver: SN65HVD230. 120Ω termination resistor required if device is at bus end. | **[OPEN]** Confirm bus topology. |
| FR-08.7 | 100nF decoupling capacitor on SN65HVD230 VCC. | |

---

## 3. Non-Functional Requirements

### NFR-01: Hardware Platform

**GPIO Pin Assignment:**

| Function | GPIO | Stage | Notes |
|----------|------|-------|-------|
| I2C SDA (LCD) | GPIO5 | 1+2 | |
| I2C SCL (LCD) | GPIO4 | 1+2 | |
| Encoder CLK | GPIO14 | 1+2 | Interrupt on CHANGE |
| Encoder DT | GPIO27 | 1+2 | |
| Encoder SW | GPIO32 | 1+2 | Internal pull-up |
| PWM Output | GPIO18 | 1+2 | 10 kHz, 8-bit, to MOSFET module |
| Reed Sensor | GPIO25 | 1 | FALLING edge interrupt. Ext 4.7kΩ pull-up + 100nF cap for long cables. |
| CAN TX | GPIO22 | 2 | To SN65HVD230 |
| CAN RX | GPIO21 | 2 | From SN65HVD230 |
| Lift Sensor | TBD | 2 | **[OPEN]** |

| ID | Requirement | Notes |
|----|-------------|-------|
| NFR-01.1 | Microcontroller: ESP32 NodeMCU dev board. | |
| NFR-01.2 | CAN Transceiver: SN65HVD230 breakout board (Stage 2). | 3.3V logic, 250 kbit/s. |
| NFR-01.3 | MOSFET driver: DollaTek 15A 400W dual-MOSFET trigger switch module. | Accepts 3.3V logic, 0–20 kHz PWM. |
| NFR-01.4 | Motor: 12V DC, 10A rated. | **[OPEN]** Inrush current TBD. MOSFET module rated 15A continuous, 30A with heatsink. |
| NFR-01.5 | Flyback diode: MBR2045 Schottky on motor output. | Cathode to +12V, anode to motor−/MOSFET output. |
| NFR-01.6 | Power supply: 12V from ISOBUS / trailer connector, regulated to 3.3V via DC/DC converter. | |
| NFR-01.7 | Transient protection: P6KE20A TVS diode on 12V input, close to DC/DC. | Cathode (stripe) to +12V. |
| NFR-01.8 | Input decoupling: 100µF electrolytic + 100nF ceramic on 12V input. | |
| NFR-01.9 | Bulk capacitor: 470µF–1000µF (25V) recommended near MOSFET module power input. | **[OPEN]** To be added for Stage 2 / field use. |

### NFR-02: Software / Development Environment

| ID | Requirement | Notes |
|----|-------------|-------|
| NFR-02.1 | Stage 1 firmware: Arduino IDE with Arduino-ESP32 Core 3.x. | Uses LiquidCrystal_I2C library. |
| NFR-02.2 | Stage 2 firmware: PlatformIO with ESP-IDF recommended. | Better suited for AgIsoStack++. |
| NFR-02.3 | ISOBUS protocol stack: AgIsoStack++ or similar ISO 11783 library (Stage 2). | |
| NFR-02.4 | VT object pool: designed using ISODesigner or compatible tool (Stage 2). | Binary resource uploaded to VT. |

### NFR-03: Wiring and EMC

| ID | Requirement | Notes |
|----|-------------|-------|
| NFR-03.1 | Reed sensor cable: shielded or twisted pair, up to 6 m. | Shield grounded on ESP32 side only. |
| NFR-03.2 | PWM signal to MOSFET module: twisted wires. | |
| NFR-03.3 | Motor output from MOSFET module: twisted wires. | |
| NFR-03.4 | Reed sensor signal conditioning: 4.7kΩ external pull-up to 3.3V + 100nF ceramic cap to GND at GPIO pin. | Low-pass filter cutoff ≈340 kHz. |
| NFR-03.5 | Motor power wires and signal wires shall maintain ≥10 cm separation where possible. | |

---

## 4. Testing Strategy

| Phase | Stage | Description | Notes |
|-------|-------|-------------|-------|
| Phase 1a | 1 | Bench test: LCD, encoder, PWM output verification with oscilloscope/multimeter. | Completed. |
| Phase 1b | 1 | Bench test: Reed sensor speed measurement with manual magnet passes. | |
| Phase 1c | 1 | Bench test: Motor running with MOSFET module, both AUTO and MANUAL modes. | |
| Phase 2a | 2 | Desktop simulation: Python script simulates CCI 100 + vehicle speed over USB-CAN adapter. | Use python-can. VT simulation optional (see Q3). |
| Phase 2b | 2 | Bench test with real CCI 100 terminal (standalone, not in vehicle). | |
| Phase 2c | 2 | Bench test: Lift position sensor integration. | |
| Phase 3 | 2 | In-vehicle test with CCI 100 and real vehicle speed signal. | |

---

## 5. Open Questions & Recommendations

| # | Stage | Question / Recommendation |
|---|-------|--------------------------|
| Q1 | 2 | **Lift position sensor:** Type and GPIO pin to be determined. Options: limit switch, proximity sensor, or Hall effect sensor. |
| Q2 | 2 | **CAN bus termination:** Confirm if ESP32 node is at bus end and needs 120Ω resistor. |
| Q3 | 2 | **Python VT simulator complexity:** A full VT simulator in Python is a significant project. Consider using AgIsoVirtualTerminal or simplify Phase 2a to CAN speed message simulation only. |
| Q4 | 1+2 | **Motor inrush current:** TBD. MOSFET module should be derated until measured. |
| Q5 | 2 | **Power supply details:** Confirm DC/DC converter specifications and whether ESP32 USB power is used during development only. |
| Q6 | 2 | **Bulk decoupling capacitor** (470µF–1000µF) near MOSFET module: recommended before field deployment. |
| Q7 | 2 | **SN65HVD230 CAN TX pull-up:** 10kΩ to 3.3V recommended to prevent bus garbage during ESP32 boot/reset. |
| Q8 | 1 | **Stage 1 AUTO mode** uses simple linear mapping (0–20 km/h). Stage 2 uses the piecewise linear mapping from FR-01.3. Confirm if Stage 1 should also use piecewise mapping. |

---

## 6. System Architecture Summary

### Stage 1 Architecture

```
Reed Sensor → GPIO25 (interrupt) → ESP32 → GPIO18 (PWM 10kHz) → MOSFET Module → 12V DC Motor

KY-040 Encoder → GPIO14/27/32 → ESP32 → I2C (GPIO5/4) → 20x4 LCD Display

12V (ISOBUS/Trailer) → TVS + Caps → DC/DC 3.3V → ESP32
```

### Stage 2 Architecture (additions)

```
CCI 100 (VT + Speed) ↔ CAN H/L ↔ SN65HVD230 ↔ GPIO21/22 ↔ ESP32 (ISOBUS Working Set)

Lift Position Sensor → GPIO TBD → ESP32
```

Local LCD + Encoder retained as fallback interface.

---

## 7. Evaluation Summary

| Aspect | Assessment |
|--------|------------|
| **Stage 1 Status** | Core hardware validated. LCD, encoder, MOSFET PWM, and reed sensor operational. AUTO and MANUAL modes implemented. |
| **Clarity** | Good. Key parameters (speed mapping, pins, hardware, timeouts) are well defined per stage. |
| **Completeness** | Stage 1 requirements complete. Stage 2 has open questions (Q1–Q8). |
| **Feasibility** | Hardware is feasible. ISOBUS stack (AgIsoStack++) mitigates protocol complexity for Stage 2. |
| **Testability** | Phased approach with clear stage boundaries. Phase 1a completed. |
| **Safety** | Signal loss timeout, boot-up default (Stopped), speed clamping (40 km/h), and lift interlock (Stage 2) defined. |

---

*Please review the open questions (Q1–Q8) so this document can be finalized as version 1.0.*
