/*
 * ESP32 ISOBUS Address Claim - Phase 2.3
 *
 * Implements ISO 11783-5 address claiming:
 *   1. On startup, sends Address Claimed (PGN 60928) with our NAME
 *   2. Listens for address conflicts (same SA from another node)
 *   3. Responds to Request for Address Claimed (PGN 59904)
 *   4. If conflict, compares NAMEs - lower NAME wins
 *   5. Also receives PGN 65256 (vehicle speed) from Phase 2.2
 *
 * Address Claim message:
 *   PGN 60928 (0x00EE00) - Address Claimed
 *   29-bit CAN ID: 0x18EEFFXX (priority 6, dest=global 0xFF, SA=XX)
 *   Data: 8 bytes = 64-bit NAME (little-endian)
 *
 * Our NAME (64-bit):
 *   - Self-configurable address: 1 (bit 63)
 *   - Industry group: 2 (Agriculture) (bits 62-60)
 *   - Device class instance: 0 (bits 59-56)
 *   - Device class: 25 (Implement) (bits 55-49)
 *   - Reserved: 0 (bit 48)
 *   - Function: 130 (Seeder/Planter) (bits 47-40)
 *   - Function instance: 0 (bits 39-35)
 *   - ECU instance: 0 (bits 34-32)
 *   - Manufacturer code: 0 (self-assigned/prototype) (bits 31-21)
 *   - Identity number: 1 (bits 20-0)
 *
 * Hardware:
 *   - SN65HVD230 CAN transceiver
 *   - CAN TX: GPIO22, CAN RX: GPIO21
 *   - Bus speed: 250 kbit/s
 */

#include "driver/twai.h"

#define PIN_CAN_TX    22
#define PIN_CAN_RX    21

// --- ISOBUS Address Configuration ---
#define PREFERRED_ADDRESS  0x80   // 128 - start of implement range
#define NULL_ADDRESS       0xFE   // 254 - used before claiming
#define GLOBAL_ADDRESS     0xFF   // 255 - broadcast

// --- PGN definitions ---
#define PGN_ADDRESS_CLAIMED  60928   // 0x00EE00
#define PGN_REQUEST          59904   // 0x00EA00
#define PGN_VEHICLE_SPEED    65256   // 0x00FEE8

// --- Our ISOBUS NAME (64-bit, little-endian in CAN frame) ---
// Built according to ISO 11783-5 Table 1
//
// Byte layout (data[0] = LSB, data[7] = MSB):
//   data[0]: Identity Number bits 0-7
//   data[1]: Identity Number bits 8-15
//   data[2]: Identity Number bits 16-20 (lower 5) | Manufacturer Code bits 0-2 (upper 3)
//   data[3]: Manufacturer Code bits 3-10
//   data[4]: ECU Instance bits 0-2 (lower 3) | Function Instance bits 0-4 (upper 5)
//   data[5]: Function (8 bits)
//   data[6]: Reserved bit 0 (lower 1) | Device Class bits 0-6 (upper 7)
//   data[7]: Device Class Instance bits 0-3 (lower 4) | Industry Group bits 0-2 (mid 3) | Self-config bit (upper 1)

// Identity number = 1 (prototype unit)
// Manufacturer code = 0 (not registered - prototype)
// ECU instance = 0, Function instance = 0
// Function = 130 (0x82) - Planter/Seeder
// Device class = 25 (0x19) - Implement (agriculture)
// Industry group = 2 (Agriculture)
// Self-configurable = 1

const uint8_t OUR_NAME[8] = {
  0x01, 0x00,       // Identity number LSB, MSB-low (= 1)
  0x00,             // Identity number MSB-high (5 bits) | Mfr code low (3 bits)
  0x00,             // Manufacturer code bits 3-10
  0x00,             // ECU instance (3 bits) | Function instance (5 bits)
  0x82,             // Function = 130 (Planter/Seeder)
  0x32,             // Reserved=0 (1 bit) | Device class=25 (7 bits) -> 25<<1 = 0x32
  0xA0              // Dev class instance=0 (4 bits) | Industry group=2 (3 bits) | Self-config=1 (1 bit)
                    // = (0<<4) | (2<<1) | (1<<0) ... wait, let me recalculate
                    // Bit 7 (MSB) = self-config = 1
                    // Bits 6-4 = industry group = 2 = 010
                    // Bits 3-0 = device class instance = 0 = 0000
                    // = 1_010_0000 = 0xA0
};

// --- State ---
uint8_t myAddress = NULL_ADDRESS;
bool addressClaimed = false;
unsigned long claimSentTime = 0;
const unsigned long CLAIM_TIMEOUT_MS = 250;  // Wait 250ms for conflicts after claiming

// Speed reception
unsigned long lastSpeedRx = 0;
float lastSpeedKmh = 0.0;
uint32_t speedRxCount = 0;

// --- Build 29-bit CAN ID for ISOBUS ---
uint32_t makeCanId(uint8_t priority, uint32_t pgn, uint8_t destOrGE, uint8_t sa) {
  uint8_t pf = (pgn >> 8) & 0xFF;
  uint8_t ps;
  if (pf < 240) {
    // PDU1: PS = destination address
    ps = destOrGE;
  } else {
    // PDU2: PS = group extension (part of PGN)
    ps = pgn & 0xFF;
  }
  uint8_t dp = (pgn >> 16) & 0x01;
  return ((uint32_t)priority << 26) | ((uint32_t)dp << 24) |
         ((uint32_t)pf << 16) | ((uint32_t)ps << 8) | sa;
}

// --- Extract PGN from 29-bit CAN ID ---
uint32_t extractPGN(uint32_t canId) {
  uint8_t pf = (canId >> 16) & 0xFF;
  uint8_t ps = (canId >> 8) & 0xFF;
  uint8_t dp = (canId >> 24) & 0x01;
  if (pf >= 240) {
    return ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps;
  } else {
    return ((uint32_t)dp << 16) | ((uint32_t)pf << 8);
  }
}

// --- Compare two 64-bit NAMEs (as 8-byte arrays, little-endian) ---
// Returns: -1 if a < b, 0 if equal, 1 if a > b
// Lower NAME = higher priority in address arbitration
int compareName(const uint8_t* a, const uint8_t* b) {
  // Compare from MSB (byte 7) to LSB (byte 0)
  for (int i = 7; i >= 0; i--) {
    if (a[i] < b[i]) return -1;
    if (a[i] > b[i]) return 1;
  }
  return 0;
}

// --- Send Address Claimed message ---
bool sendAddressClaim(uint8_t sa) {
  // PGN 60928: PDU1, PF=0xEE, destination=0xFF (global)
  // CAN ID = priority(6) | PF(0xEE) | dest(0xFF) | SA
  uint32_t canId = makeCanId(6, PGN_ADDRESS_CLAIMED, GLOBAL_ADDRESS, sa);

  twai_message_t msg;
  msg.identifier = canId;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 8;
  memcpy(msg.data, OUR_NAME, 8);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
  if (err == ESP_OK) {
    Serial.printf("TX Address Claim: SA=0x%02X CAN_ID=0x%08X\n", sa, canId);
    Serial.print("  NAME: ");
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", OUR_NAME[i]);
    Serial.println();
    return true;
  } else {
    Serial.printf("TX Address Claim FAILED: %d\n", err);
    return false;
  }
}

// --- Send Cannot Claim Address ---
void sendCannotClaim() {
  uint32_t canId = makeCanId(6, PGN_ADDRESS_CLAIMED, GLOBAL_ADDRESS, NULL_ADDRESS);
  twai_message_t msg;
  msg.identifier = canId;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 8;
  memcpy(msg.data, OUR_NAME, 8);
  twai_transmit(&msg, pdMS_TO_TICKS(100));
  Serial.println("TX Cannot Claim Address");
}

// --- Decode vehicle speed from PGN 65256 ---
float decodeGroundSpeed(uint8_t* data) {
  uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  if (raw == 0xFFFF) return -1.0;
  if (raw == 0xFFFE) return -2.0;
  return (float)raw / 256.0;
}

// --- Handle received CAN frame ---
void handleFrame(twai_message_t* msg) {
  if (!msg->extd) return;  // Only process extended (ISOBUS) frames

  uint32_t pgn = extractPGN(msg->identifier);
  uint8_t sa = msg->identifier & 0xFF;

  // --- PGN 60928: Address Claimed ---
  if (pgn == PGN_ADDRESS_CLAIMED) {
    Serial.printf("RX Address Claim from SA=0x%02X: ", sa);
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", msg->data[i]);
    Serial.println();

    // Check if someone claimed our address
    if (sa == myAddress && addressClaimed) {
      int cmp = compareName(OUR_NAME, msg->data);
      if (cmp < 0) {
        // Our NAME is lower (higher priority) - we win, re-claim
        Serial.println("  -> Conflict: WE WIN (lower NAME), re-claiming");
        sendAddressClaim(myAddress);
      } else {
        // Their NAME is lower or equal - we lose
        Serial.println("  -> Conflict: WE LOSE, must find new address");
        addressClaimed = false;
        // Try next address
        myAddress++;
        if (myAddress >= 247) {  // ISOBUS dynamic range is 128-247
          Serial.println("  -> No available addresses! Cannot claim.");
          sendCannotClaim();
          myAddress = NULL_ADDRESS;
        } else {
          Serial.printf("  -> Trying new address: 0x%02X\n", myAddress);
          sendAddressClaim(myAddress);
          claimSentTime = millis();
        }
      }
    }
  }

  // --- PGN 59904: Request ---
  if (pgn == PGN_REQUEST) {
    // Check if requesting PGN 60928 (address claim)
    uint32_t requestedPgn = (uint32_t)msg->data[0] |
                            ((uint32_t)msg->data[1] << 8) |
                            ((uint32_t)msg->data[2] << 16);
    if (requestedPgn == PGN_ADDRESS_CLAIMED) {
      Serial.println("RX Request for Address Claim - responding");
      if (addressClaimed) {
        sendAddressClaim(myAddress);
      } else {
        sendCannotClaim();
      }
    }
  }

  // --- PGN 65256: Vehicle Speed ---
  if (pgn == PGN_VEHICLE_SPEED) {
    speedRxCount++;
    lastSpeedRx = millis();
    float kmh = decodeGroundSpeed(msg->data);
    lastSpeedKmh = kmh;

    // Print every 10th message to avoid flooding serial
    if (speedRxCount % 10 == 0) {
      if (kmh >= 0) {
        Serial.printf("Speed: %.2f km/h (count: %lu)\n", kmh, speedRxCount);
      }
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 ISOBUS Address Claim - Phase 2.3 ===");
  Serial.printf("Preferred address: 0x%02X (%d)\n", PREFERRED_ADDRESS, PREFERRED_ADDRESS);
  Serial.print("NAME: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", OUR_NAME[i]);
  Serial.println();
  Serial.println();

  // TWAI config
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
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

  // Small delay before claiming (let bus settle)
  delay(250);

  // Send initial address claim
  myAddress = PREFERRED_ADDRESS;
  Serial.printf("\nClaiming address 0x%02X...\n", myAddress);
  sendAddressClaim(myAddress);
  claimSentTime = millis();

  Serial.println("Waiting 250ms for conflicts...");
  Serial.println("------------------------------------------");
}

// --- Loop ---
unsigned long lastStatus = 0;

void loop() {
  // Receive frames
  twai_message_t rxMsg;
  if (twai_receive(&rxMsg, pdMS_TO_TICKS(10)) == ESP_OK) {
    handleFrame(&rxMsg);
  }

  // Check if claim timeout passed (no conflict = success)
  if (!addressClaimed && myAddress != NULL_ADDRESS && claimSentTime > 0) {
    if ((millis() - claimSentTime) > CLAIM_TIMEOUT_MS) {
      addressClaimed = true;
      Serial.printf("\n*** ADDRESS CLAIMED: 0x%02X (%d) ***\n\n", myAddress, myAddress);
    }
  }

  // Periodic status
  if ((millis() - lastStatus) >= 5000) {
    lastStatus = millis();
    if (addressClaimed) {
      Serial.printf("Status: SA=0x%02X claimed | speed msgs: %lu",
                    myAddress, speedRxCount);
      if (lastSpeedKmh >= 0 && speedRxCount > 0) {
        Serial.printf(" | last: %.1f km/h", lastSpeedKmh);
      }
      Serial.println();
    } else {
      Serial.println("Status: Address NOT claimed");
    }
  }
}
