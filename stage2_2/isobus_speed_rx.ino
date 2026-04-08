/*
 * ESP32 ISOBUS Speed Receiver - Phase 2.2
 *
 * Receives PGN 65256 (Vehicle Direction/Speed) from ISOBUS
 * and decodes SPN 1859 (Ground-Based Vehicle Speed).
 *
 * This sketch is a standalone test - not yet integrated into
 * the motor controller. It just receives and displays speed.
 *
 * PGN 65256 format:
 *   29-bit CAN ID: 0x18FEE8xx (xx = source address)
 *   Byte 1-2: SPN 1859 - Ground-Based Vehicle Speed
 *             Little-endian, resolution 1/256 km/h per bit
 *   Byte 3-4: SPN 1860 - Ground-Based Vehicle Direction (not used here)
 *   Byte 5-8: Other SPNs (not used here)
 *
 * Hardware:
 *   - SN65HVD230 CAN transceiver
 *   - CAN TX: GPIO22, CAN RX: GPIO21
 *   - Bus speed: 250 kbit/s
 */

#include "driver/twai.h"

#define PIN_CAN_TX    22
#define PIN_CAN_RX    21

// PGN 65256 = 0xFEE8
// In a 29-bit CAN ID: Priority(3) + Reserved(1) + DP(1) + PF(8) + PS(8) + SA(8)
// For PGN 65256: PF=0xFE, PS=0xE8
// Mask: we want to match PGN regardless of priority and source address
// CAN ID format: PPP R DP PPPPPPPP SSSSSSSS AAAAAAAA
//                pri     PF        PS        SA
// We match bits 8-25 (PGN field) = 0x00FEE800, mask = 0x00FFFF00

// Simpler approach: just check the PGN from any received extended frame
#define PGN_VEHICLE_SPEED  65256  // 0xFEE8

unsigned long lastSpeedRx = 0;
float lastSpeedKmh = 0.0;
uint32_t rxCount = 0;
const unsigned long timeoutMs = 5000;  // 5 second timeout per FR-01.6

// Extract PGN from 29-bit CAN ID
uint32_t extractPGN(uint32_t canId) {
  // CAN ID bits: PPP R DP PF(8) PS(8) SA(8)
  uint8_t pf = (canId >> 16) & 0xFF;
  uint8_t ps = (canId >> 8) & 0xFF;
  uint8_t dp = (canId >> 24) & 0x01;

  uint32_t pgn;
  if (pf >= 240) {
    // PDU2: PS is group extension, part of PGN
    pgn = (uint32_t)(dp << 16) | ((uint32_t)pf << 8) | ps;
  } else {
    // PDU1: PS is destination address, not part of PGN
    pgn = (uint32_t)(dp << 16) | ((uint32_t)pf << 8);
  }
  return pgn;
}

// Decode SPN 1859 from PGN 65256 data
float decodeGroundSpeed(uint8_t* data) {
  // Bytes 1-2 (index 0-1), little-endian, resolution 1/256 km/h
  uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

  // 0xFFFF = not available
  if (raw == 0xFFFF) return -1.0;
  // 0xFFFE = error
  if (raw == 0xFFFE) return -2.0;

  return (float)raw / 256.0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ESP32 ISOBUS Speed Receiver - Phase 2.2 ===");
  Serial.println("Listening for PGN 65256 (Vehicle Direction/Speed)");
  Serial.println("Decoding SPN 1859 (Ground-Based Vehicle Speed)");
  Serial.println();

  // TWAI config - accept all extended frames
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)PIN_CAN_TX,
    (gpio_num_t)PIN_CAN_RX,
    TWAI_MODE_NORMAL
  );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("TWAI install FAILED: %d\n", err);
    while (1) delay(1000);
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("TWAI start FAILED: %d\n", err);
    while (1) delay(1000);
  }

  Serial.println("TWAI started at 250 kbit/s");
  Serial.println("Waiting for speed data...");
  Serial.println("------------------------------------------");
}

void loop() {
  twai_message_t rxMsg;

  if (twai_receive(&rxMsg, pdMS_TO_TICKS(100)) == ESP_OK) {
    // Only process extended (29-bit) frames - ISOBUS uses extended CAN
    if (!rxMsg.extd) return;

    uint32_t pgn = extractPGN(rxMsg.identifier);
    uint8_t sa = rxMsg.identifier & 0xFF;

    if (pgn == PGN_VEHICLE_SPEED) {
      rxCount++;
      lastSpeedRx = millis();

      float kmh = decodeGroundSpeed(rxMsg.data);
      lastSpeedKmh = kmh;

      // Print raw bytes for debugging
      Serial.printf("PGN 65256 from SA=0x%02X | raw: ", sa);
      for (int i = 0; i < rxMsg.data_length_code; i++) {
        Serial.printf("%02X ", rxMsg.data[i]);
      }

      // Print decoded speed
      if (kmh >= 0) {
        Serial.printf("| speed: %.2f km/h", kmh);
      } else if (kmh == -1.0) {
        Serial.print("| speed: N/A");
      } else {
        Serial.print("| speed: ERROR");
      }

      Serial.printf(" | count: %lu\n", rxCount);
    }
  }

  // Check for timeout
  if (lastSpeedRx > 0 && (millis() - lastSpeedRx) > timeoutMs) {
    Serial.println("WARNING: No speed data for >5 seconds!");
    lastSpeedRx = millis();  // reset to avoid spamming
  }
}
