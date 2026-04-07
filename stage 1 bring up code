/*
 * Ali's Motor Controller - Stage 1 Bring-Up
 * 
 * Hardware:
 *   - ESP32 NodeMCU
 *   - KY-040 Rotary Encoder (CLK=27, DT=26, SW=25)
 *   - I2C 20x4 LCD Display (SDA=13, SCL=14)
 *   - MOSFET driver PWM output (PWM=4, 10kHz)
 *   - Reed sensor for wheel speed (GPIO19)
 *   - Inductive sensor via PCF817 optocoupler (GPIO23)
 *   - [Stage 2] CAN transceiver SN65HVD230 (TX=22, RX=21)
 *
 * Modes:
 *   - AUTO:   PWM proportional to speed (0 km/h=0%, 20 km/h=100%)
 *   - MANUAL: PWM set by rotary encoder (0-100%)
 *
 * Speed sources:
 *   - REED: 1 pulse per wheel revolution (410mm wheel)
 *          Filtered: median5 + rate limiter (2.5 km/h per 250ms)
 *   - IND:  Inductive sensor, 7.34 pulses per meter
 *          Filtered: median5 + rate limiter (2.5 km/h per 250ms)
 *
 * Controls:
 *   - Short press:  START / STOP
 *   - Double press: Switch speed sensor (REED <-> IND)
 *   - Long press (>1s): Switch AUTO <-> MANUAL
 *   - Rotate: Adjust PWM % (MANUAL mode only)
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- Pin Definitions ---------------------------------------------------------
#define PIN_SDA       13
#define PIN_SCL       14

#define PIN_ENC_CLK   27
#define PIN_ENC_DT    26
#define PIN_ENC_SW    25

#define PIN_PWM       4

#define PIN_REED      19
#define PIN_INDUCTIVE 23      // Inductive sensor via PCF817 optocoupler

// --- Stage 2 Pins (reserved, not yet used) -----------------------------------
#define PIN_CAN_TX    22
#define PIN_CAN_RX    21

// --- PWM Configuration -------------------------------------------------------
#define PWM_FREQ      10000   // 10 kHz
#define PWM_RESOLUTION 8      // 8-bit (0-255)

// --- Wheel Configuration -----------------------------------------------------
// >>> SET THIS TO YOUR WHEEL DIAMETER IN mm <<<
#define WHEEL_DIAMETER_MM  410

// Circumference in meters: pi * d
const float wheelCircumM = (float)WHEEL_DIAMETER_MM / 1000.0 * PI;

// --- Inductive Sensor Configuration ------------------------------------------
// Pulses per meter of ground travel
#define INDUCTIVE_PULSES_PER_M  7.34

// --- Speed Filtering (shared) ------------------------------------------------
#define MEDIAN_SIZE         5       // median filter window (both sensors)
#define RATE_LIMIT_KMH      2.5    // max speed change per 250ms = 10 km/h/s

// --- Inductive Sensor Filtering ----------------------------------------------
#define IND_MIN_PERIOD_US   12000   // ~40 km/h max
#define IND_MAX_PERIOD_US   1000000 // ~0.5 km/h min

// --- Reed Sensor Filtering ---------------------------------------------------
// At 40 km/h: 11.11 m/s / 1.288m circ = 8.63 Hz -> period 115,875 us
// At 0.3 km/h: 0.083 m/s / 1.288m = 0.065 Hz -> period ~15,400,000 us
#define REED_MIN_PERIOD_US  40000    // ~116 km/h safety margin
#define REED_MAX_PERIOD_US  10000000 // ~0.46 km/h

// --- Auto Mode Configuration -------------------------------------------------
// Speed at which PWM reaches 100%
#define AUTO_MAX_SPEED_KMH  20.0

// --- LCD (address 0x27 is most common, try 0x3F if not working) --------------
LiquidCrystal_I2C lcd(0x27, 20, 4);

// --- Mode / State ------------------------------------------------------------
enum Mode { MODE_MANUAL, MODE_AUTO };
enum SpeedSource { SRC_REED, SRC_INDUCTIVE };

Mode currentMode = MODE_AUTO;
SpeedSource speedSource = SRC_INDUCTIVE;
bool motorRunning = false;

// --- Encoder State -----------------------------------------------------------
volatile int  pwmPercent     = 0;       // 0 - 100 (manual)
volatile bool encoderChanged = false;
volatile int  lastEncCLK     = HIGH;

// --- Button State ------------------------------------------------------------
bool     buttonDown       = false;
unsigned long buttonDownTime  = 0;
bool     longPressHandled = false;
const unsigned long longPressMs = 1000;       // 1 second for long press
const unsigned long debounceDelay = 50;       // ms
const unsigned long doublePressMs = 400;      // max gap between double press
unsigned long lastButtonChange = 0;
unsigned long lastReleaseTime  = 0;
bool     waitingSecondPress = false;
bool     startStopPending   = false;

// --- Reed Sensor State -------------------------------------------------------
volatile unsigned long reedLastPulseUs = 0;
volatile unsigned long reedPeriodUs    = 0;
volatile bool          reedFirstPulse  = true;
volatile unsigned long reedPeriodBuf[MEDIAN_SIZE] = {0};
volatile int           reedBufIdx = 0;
volatile int           reedBufCount = 0;
float reedFilteredKmh = 0.0;

// --- Inductive Sensor State (polling, not interrupt) -------------------------
unsigned long indLastPulseUs = 0;
unsigned long indPeriodUs = 0;
unsigned long indPeriodBuf[MEDIAN_SIZE] = {0};
int           indBufIdx = 0;
int           indBufCount = 0;
float indFilteredKmh = 0.0;
bool indPinLast = HIGH;             // HIGH = PC817 off (pull-up)
bool indPinStable = HIGH;           // debounced state
unsigned long indPinChangeUs = 0;   // when pin last changed

// --- Timing ------------------------------------------------------------------
unsigned long startTime = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 250;

// Speed goes to 0 if no pulse within this timeout
const unsigned long sensorTimeoutUs = 2000000; // 2 seconds

// --- Encoder ISR -------------------------------------------------------------
void IRAM_ATTR encoderISR() {
  int clkState = digitalRead(PIN_ENC_CLK);
  int dtState  = digitalRead(PIN_ENC_DT);

  if (clkState != lastEncCLK && clkState == LOW) {
    if (dtState != clkState) {
      if (pwmPercent < 100) pwmPercent++;
    } else {
      if (pwmPercent > 0) pwmPercent--;
    }
    encoderChanged = true;
  }
  lastEncCLK = clkState;
}

// --- Reed Sensor ISR ---------------------------------------------------------
void IRAM_ATTR reedISR() {
  unsigned long now = micros();
  unsigned long elapsed = now - reedLastPulseUs;
  if (elapsed < 5000) return;         // 5ms debounce
  reedLastPulseUs = now;
  if (reedFirstPulse) { reedFirstPulse = false; return; }
  if (elapsed < REED_MIN_PERIOD_US || elapsed > REED_MAX_PERIOD_US) return;
  reedPeriodUs = elapsed;
  reedPeriodBuf[reedBufIdx] = elapsed;
  reedBufIdx = (reedBufIdx + 1) % MEDIAN_SIZE;
  if (reedBufCount < MEDIAN_SIZE) reedBufCount++;
}

// --- Inductive Sensor Polling (replaces unreliable interrupt) -----------------
// Only debounces the falling edge (HIGH->LOW). Rising edge passes immediately.
void pollInductive() {
  bool raw = digitalRead(PIN_INDUCTIVE);
  unsigned long now = micros();

  if (indPinStable == HIGH) {
    // Waiting for falling edge (sensor activating)
    if (raw == LOW) {
      // Pin just went low - start or continue debounce
      if (indPinLast == HIGH) {
        // First LOW reading, start timer
        indPinLast = LOW;
        indPinChangeUs = now;
      } else if ((now - indPinChangeUs) >= 10000) {
        // Stable LOW for 10ms - accept falling edge
        indPinStable = LOW;

        unsigned long elapsed = now - indLastPulseUs;
        indLastPulseUs = now;

        if (elapsed >= IND_MIN_PERIOD_US && elapsed <= IND_MAX_PERIOD_US) {
          indPeriodUs = elapsed;
          indPeriodBuf[indBufIdx] = elapsed;
          indBufIdx = (indBufIdx + 1) % MEDIAN_SIZE;
          if (indBufCount < MEDIAN_SIZE) indBufCount++;
        }
      }
    } else {
      // Still HIGH or bounced back to HIGH - reset
      indPinLast = HIGH;
    }
  } else {
    // Pin is confirmed LOW, waiting for release (rising edge)
    // No debounce needed - accept immediately
    if (raw == HIGH) {
      indPinStable = HIGH;
      indPinLast = HIGH;
    }
  }
}

// --- Get Raw Speed in km/h (from reed sensor, median filtered) ---------------
float getSpeedReedRaw() {
  unsigned long lastPulse;
  unsigned long bufCopy[MEDIAN_SIZE];
  int bc;

  noInterrupts();
  lastPulse = reedLastPulseUs;
  bc = reedBufCount;
  for (int i = 0; i < MEDIAN_SIZE; i++) bufCopy[i] = reedPeriodBuf[i];
  interrupts();

  if (lastPulse == 0) return 0.0;
  if ((micros() - lastPulse) > sensorTimeoutUs) return 0.0;
  if (bc == 0) return 0.0;

  int n = (bc < MEDIAN_SIZE) ? bc : MEDIAN_SIZE;
  unsigned long vb[MEDIAN_SIZE] = {0};
  int src = reedBufIdx;
  for (int i = n - 1; i >= 0; i--) {
    src = (src - 1 + MEDIAN_SIZE) % MEDIAN_SIZE;
    vb[i] = bufCopy[src];
  }
  for (int i = 1; i < n; i++) {
    unsigned long key = vb[i];
    int j = i - 1;
    while (j >= 0 && vb[j] > key) { vb[j + 1] = vb[j]; j--; }
    vb[j + 1] = key;
  }
  unsigned long medPeriod = vb[n / 2];
  if (medPeriod == 0) return 0.0;

  float rps = 1000000.0 / (float)medPeriod;
  float kmh = rps * wheelCircumM * 3.6;
  return (kmh > 40.0) ? 0.0 : kmh;
}

// --- Update rate-limited reed speed ------------------------------------------
void updateFilteredReedSpeed() {
  float raw = getSpeedReedRaw();
  float diff = raw - reedFilteredKmh;
  if (diff > RATE_LIMIT_KMH) reedFilteredKmh += RATE_LIMIT_KMH;
  else if (diff < -RATE_LIMIT_KMH) reedFilteredKmh -= RATE_LIMIT_KMH;
  else reedFilteredKmh = raw;
  if (reedFilteredKmh < 0.0) reedFilteredKmh = 0.0;
  if (reedFilteredKmh > 40.0) reedFilteredKmh = 40.0;
  if (raw == 0.0 && reedFilteredKmh < 1.0) reedFilteredKmh = 0.0;
}

// --- Get Speed in km/h (from inductive sensor, median filtered) --------------
float getSpeedInductiveRaw() {
  unsigned long lastPulse;
  unsigned long bufCopy[MEDIAN_SIZE];
  int bc;
  lastPulse = indLastPulseUs;
  bc = indBufCount;
  for (int i = 0; i < MEDIAN_SIZE; i++) bufCopy[i] = indPeriodBuf[i];
  if (lastPulse == 0) return 0.0;
  if ((micros() - lastPulse) > sensorTimeoutUs) return 0.0;
  if (bc == 0) return 0.0;
  int n = (bc < MEDIAN_SIZE) ? bc : MEDIAN_SIZE;
  unsigned long vb[MEDIAN_SIZE] = {0};
  int src = indBufIdx;
  for (int i = n - 1; i >= 0; i--) {
    src = (src - 1 + MEDIAN_SIZE) % MEDIAN_SIZE;
    vb[i] = bufCopy[src];
  }
  // Sort only the n valid entries, pick middle
  // Simple insertion sort on vb[0..n-1]
  for (int i = 1; i < n; i++) {
    unsigned long key = vb[i];
    int j = i - 1;
    while (j >= 0 && vb[j] > key) { vb[j + 1] = vb[j]; j--; }
    vb[j + 1] = key;
  }
  unsigned long medPeriod = vb[n / 2];
  if (medPeriod == 0) return 0.0;
  float pps = 1000000.0 / (float)medPeriod;
  float kmh = (pps / INDUCTIVE_PULSES_PER_M) * 3.6;
  return (kmh > 40.0) ? 0.0 : kmh;
}

// --- Update rate-limited inductive speed -------------------------------------
void updateFilteredIndSpeed() {
  float rawKmh = getSpeedInductiveRaw();
  float diff = rawKmh - indFilteredKmh;
  if (diff > RATE_LIMIT_KMH) indFilteredKmh += RATE_LIMIT_KMH;
  else if (diff < -RATE_LIMIT_KMH) indFilteredKmh -= RATE_LIMIT_KMH;
  else indFilteredKmh = rawKmh;
  if (indFilteredKmh < 0.0) indFilteredKmh = 0.0;
  if (indFilteredKmh > 40.0) indFilteredKmh = 40.0;
  if (rawKmh == 0.0 && indFilteredKmh < 1.0) indFilteredKmh = 0.0;
}

// --- Get Speed from active source --------------------------------------------
float getSpeedKmh() {
  if (speedSource == SRC_REED) {
    return reedFilteredKmh;
  } else {
    return indFilteredKmh;
  }
}

// --- Get Active PWM % --------------------------------------------------------
int getActivePwmPercent() {
  if (!motorRunning) return 0;

  if (currentMode == MODE_MANUAL) {
    return pwmPercent;
  } else {
    float kmh = getSpeedKmh();
    float pct = (kmh / AUTO_MAX_SPEED_KMH) * 100.0;
    if (pct < 0.0) pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    return (int)pct;
  }
}

// --- Apply PWM ---------------------------------------------------------------
void applyPWM() {
  int pct = getActivePwmPercent();
  uint32_t duty = map(pct, 0, 100, 0, 255);
  ledcWrite(PIN_PWM, duty);
}

// --- Update Display ----------------------------------------------------------
void updateDisplay() {
  char line[21];

  // Row 0: Mode + sensor source
  const char* modeStr   = (currentMode == MODE_AUTO) ? "AUTO" : "MANU";
  const char* sensorStr = (speedSource == SRC_REED) ? "REED" : "IND";
  snprintf(line, sizeof(line), "Ali [%-4s] spd:%-4s ", modeStr, sensorStr);
  lcd.setCursor(0, 0);
  lcd.print(line);

  // Row 1: PWM output %
  int activePwm = getActivePwmPercent();
  if (currentMode == MODE_MANUAL) {
    snprintf(line, sizeof(line), "PWM: %3d%%           ", activePwm);
  } else {
    snprintf(line, sizeof(line), "PWM: %3d%% (auto)    ", activePwm);
  }
  lcd.setCursor(0, 1);
  lcd.print(line);

  // Row 2: Speed
  float kmh = getSpeedKmh();
  char speedStr[8];
  dtostrf(kmh, 5, 1, speedStr);
  snprintf(line, sizeof(line), "Speed:%s km/h   ", speedStr);
  lcd.setCursor(0, 2);
  lcd.print(line);

  // Row 3: State + uptime
  unsigned long seconds = (millis() - startTime) / 1000;
  unsigned long mins = seconds / 60;
  unsigned long secs = seconds % 60;
  if (motorRunning) {
    snprintf(line, sizeof(line), "RUN  %4lu:%02lu        ", mins, secs);
  } else {
    snprintf(line, sizeof(line), "STOP %4lu:%02lu        ", mins, secs);
  }
  lcd.setCursor(0, 3);
  lcd.print(line);
}

// --- Execute Start/Stop ------------------------------------------------------
void doStartStop() {
  motorRunning = !motorRunning;
  applyPWM();
  Serial.print("Motor ");
  Serial.println(motorRunning ? "STARTED" : "STOPPED");
}

// --- Execute Sensor Switch ---------------------------------------------------
void doSensorSwitch() {
  if (speedSource == SRC_REED) {
    speedSource = SRC_INDUCTIVE;
  } else {
    speedSource = SRC_REED;
  }
  applyPWM();
  Serial.print("Sensor: ");
  Serial.println(speedSource == SRC_REED ? "REED" : "INDUCTIVE");
}

// --- Handle Button (short / double / long press) -----------------------------
void handleButton() {
  bool raw = digitalRead(PIN_ENC_SW);
  unsigned long now = millis();

  // Debounce
  if ((now - lastButtonChange) < debounceDelay) return;

  // Button just pressed
  if (raw == LOW && !buttonDown) {
    buttonDown = true;
    buttonDownTime = now;
    longPressHandled = false;
    lastButtonChange = now;

    // Check if this is the second press of a double-press
    if (waitingSecondPress && (now - lastReleaseTime) <= doublePressMs) {
      // -- Double press: switch sensor --
      doSensorSwitch();
      waitingSecondPress = false;
      startStopPending = false;
      longPressHandled = true;  // prevent long press from also firing
    }
  }

  // Button held -- check for long press
  if (raw == LOW && buttonDown && !longPressHandled) {
    if ((now - buttonDownTime) >= longPressMs) {
      // -- Long press: switch mode --
      if (currentMode == MODE_AUTO) {
        currentMode = MODE_MANUAL;
      } else {
        currentMode = MODE_AUTO;
      }
      longPressHandled = true;
      waitingSecondPress = false;
      startStopPending = false;
      applyPWM();

      Serial.print("Mode: ");
      Serial.println(currentMode == MODE_AUTO ? "AUTO" : "MANUAL");
    }
  }

  // Button released
  if (raw == HIGH && buttonDown) {
    buttonDown = false;
    lastButtonChange = now;

    if (!longPressHandled) {
      // First short press -- wait for possible second press
      if (!waitingSecondPress) {
        waitingSecondPress = true;
        startStopPending = true;
        lastReleaseTime = now;
      }
    }
  }

  // Timeout: if waiting for second press and it didn't come, execute start/stop
  if (waitingSecondPress && !buttonDown && startStopPending) {
    if ((now - lastReleaseTime) > doublePressMs) {
      doStartStop();
      waitingSecondPress = false;
      startStopPending = false;
    }
  }
}

// --- Setup -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Ali's Motor Controller - Stage 1 Bring-Up");

  // I2C on custom pins
  Wire.begin(PIN_SDA, PIN_SCL);

  // LCD init
  lcd.init();
  lcd.backlight();

  // Encoder pins
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);

  lastEncCLK = digitalRead(PIN_ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encoderISR, CHANGE);

  // Reed sensor pin
  pinMode(PIN_REED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), reedISR, FALLING);

  // Inductive sensor pin (PC817 output, polled - not interrupt)
  pinMode(PIN_INDUCTIVE, INPUT);

  // PWM setup (ESP32 Core 3.x API)
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PIN_PWM, 0);

  startTime = millis();
  updateDisplay();

  Serial.println("Setup complete.");
  Serial.print("Wheel: ");
  Serial.print(WHEEL_DIAMETER_MM);
  Serial.print("mm, circumference: ");
  Serial.print(wheelCircumM, 3);
  Serial.println("m");
  Serial.print("Inductive: ");
  Serial.print(INDUCTIVE_PULSES_PER_M);
  Serial.println(" pulses/m (debounce 10ms)");
  Serial.println("Short: START/STOP | Double: sensor | Long: AUTO/MANUAL");
}

// --- Loop --------------------------------------------------------------------
void loop() {
  // -- Poll inductive sensor (PC817 via software debounce) --
  pollInductive();

  // -- Handle button (short/double/long press) --
  handleButton();

  // -- Handle encoder rotation (MANUAL mode PWM adjustment) --
  if (encoderChanged) {
    encoderChanged = false;
    if (currentMode == MODE_MANUAL) {
      applyPWM();
    }
  }

  if ((millis() - lastDisplayUpdate) > displayInterval) {
    lastDisplayUpdate = millis();
    updateFilteredReedSpeed();
    updateFilteredIndSpeed();
    if (currentMode == MODE_AUTO && motorRunning) applyPWM();
    updateDisplay();
  }
}
