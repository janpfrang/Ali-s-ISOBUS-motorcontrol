/*
 * Ali's Motor Controller - Stage 1 Bring-Up working
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
 *
 * Changelog v3 (fixes for erratic inductive speed readings):
 *   Fix 1: Null-guard in getSpeedInductiveRaw() - zero entries in the ring
 *           buffer no longer corrupt the median (buffer not yet full).
 *   Fix 2: noInterrupts() guard when copying indPeriodBuf / indLastPulseUs
 *           in getSpeedInductiveRaw() - prevents torn reads of 4-byte longs.
 *   Fix 3: Debounce reduced from 10 ms to 2 ms - prevents pulse loss at
 *           higher speeds (10 ms debounce too close to 12 ms min period).
 *   Fix 4: Real falling-edge timestamp stored separately (indFallingEdgeUs).
 *           Period is now measured between actual edges, not between the
 *           moments the debounce window expires (~10 ms systematic offset
 *           per pulse in v1).
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
#define PWM_FREQ       10000  // 10 kHz
#define PWM_RESOLUTION 8      // 8-bit (0-255)

// --- Wheel Configuration -----------------------------------------------------
#define WHEEL_DIAMETER_MM  410
const float wheelCircumM = (float)WHEEL_DIAMETER_MM / 1000.0 * PI;

// --- Inductive Sensor Configuration ------------------------------------------
#define INDUCTIVE_PULSES_PER_M  7.34

// --- Speed Filtering (shared) ------------------------------------------------
#define MEDIAN_SIZE      5
#define RATE_LIMIT_KMH   2.5    // max speed change per 250 ms = 10 km/h/s

// --- Inductive Sensor Filtering ----------------------------------------------
#define IND_MIN_PERIOD_US   12000   // ~40 km/h max
#define IND_MAX_PERIOD_US   1000000 // ~0.5 km/h min
#define IND_DEBOUNCE_US     2000    // FIX 3: was 10000 (10 ms)

// --- Reed Sensor Filtering ---------------------------------------------------
#define REED_MIN_PERIOD_US  40000
#define REED_MAX_PERIOD_US  10000000

// --- Auto Mode Configuration -------------------------------------------------
#define AUTO_MAX_SPEED_KMH  20.0

// --- LCD ---------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 20, 4);

// --- Mode / State ------------------------------------------------------------
enum Mode        { MODE_MANUAL, MODE_AUTO };
enum SpeedSource { SRC_REED, SRC_INDUCTIVE };

Mode        currentMode  = MODE_AUTO;
SpeedSource speedSource  = SRC_INDUCTIVE;
bool        motorRunning = false;

// --- Encoder State -----------------------------------------------------------
volatile int  pwmPercent    = 0;
volatile bool encoderChanged = false;
volatile int  lastEncCLK    = HIGH;

// --- Button State ------------------------------------------------------------
bool          buttonDown       = false;
unsigned long buttonDownTime   = 0;
bool          longPressHandled = false;
const unsigned long longPressMs    = 1000;
const unsigned long debounceDelay  = 50;
const unsigned long doublePressMs  = 400;
unsigned long lastButtonChange = 0;
unsigned long lastReleaseTime  = 0;
bool          waitingSecondPress = false;
bool          startStopPending   = false;

// --- Reed Sensor State -------------------------------------------------------
volatile unsigned long reedLastPulseUs               = 0;
volatile unsigned long reedPeriodUs                  = 0;
volatile bool          reedFirstPulse                = true;
volatile unsigned long reedPeriodBuf[MEDIAN_SIZE]    = {0};
volatile int           reedBufIdx                    = 0;
volatile int           reedBufCount                  = 0;
float reedFilteredKmh = 0.0;

// --- Inductive Sensor State --------------------------------------------------
unsigned long indLastPulseUs              = 0;  // timestamp of last accepted edge
unsigned long indFallingEdgeUs            = 0;  // FIX 4: real edge time
unsigned long indPeriodUs                 = 0;
unsigned long indPeriodBuf[MEDIAN_SIZE]   = {0};
int           indBufIdx                   = 0;
int           indBufCount                 = 0;
float         indFilteredKmh              = 0.0;
bool          indPinLast                  = HIGH;
bool          indPinStable                = HIGH;
unsigned long indPinChangeUs              = 0;
unsigned long indRejectCount              = 0;  // counts out-of-range pulses

// --- Timing ------------------------------------------------------------------
unsigned long startTime        = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval  = 250;
const unsigned long sensorTimeoutUs  = 2000000; // 2 s → speed = 0

// --- Encoder ISR -------------------------------------------------------------
void IRAM_ATTR encoderISR() {
  int clkState = digitalRead(PIN_ENC_CLK);
  int dtState  = digitalRead(PIN_ENC_DT);
  if (clkState != lastEncCLK && clkState == LOW) {
    if (dtState != clkState) { if (pwmPercent < 100) pwmPercent++; }
    else                     { if (pwmPercent > 0)   pwmPercent--; }
    encoderChanged = true;
  }
  lastEncCLK = clkState;
}

// --- Reed Sensor ISR ---------------------------------------------------------
void IRAM_ATTR reedISR() {
  unsigned long now     = micros();
  unsigned long elapsed = now - reedLastPulseUs;
  if (elapsed < 5000) return;
  reedLastPulseUs = now;
  if (reedFirstPulse) { reedFirstPulse = false; return; }
  if (elapsed < REED_MIN_PERIOD_US || elapsed > REED_MAX_PERIOD_US) return;
  reedPeriodUs            = elapsed;
  reedPeriodBuf[reedBufIdx] = elapsed;
  reedBufIdx              = (reedBufIdx + 1) % MEDIAN_SIZE;
  if (reedBufCount < MEDIAN_SIZE) reedBufCount++;
}

// --- Inductive Sensor Polling ------------------------------------------------
// FIX 3: debounce reduced to IND_DEBOUNCE_US (2 ms).
// FIX 4: indFallingEdgeUs records the moment the pin first went LOW;
//        the period is measured from that real edge, not from when the
//        debounce window expired.
void pollInductive() {
  bool raw = digitalRead(PIN_INDUCTIVE);
  unsigned long now = micros();

  if (indPinStable == HIGH) {
    if (raw == LOW) {
      if (indPinLast == HIGH) {
        // First LOW sample - record real edge time (FIX 4)
        indPinLast       = LOW;
        indPinChangeUs   = now;
        indFallingEdgeUs = now;           // FIX 4: save true edge timestamp
      } else if ((now - indPinChangeUs) >= IND_DEBOUNCE_US) {
        // Debounce window passed - accept the edge
        indPinStable = LOW;

        // FIX 4: measure period from real edge to real edge
        unsigned long elapsed = indFallingEdgeUs - indLastPulseUs;
        indLastPulseUs = indFallingEdgeUs;  // FIX 4: anchor to real edge

        if (elapsed >= IND_MIN_PERIOD_US && elapsed <= IND_MAX_PERIOD_US) {
          indPeriodUs               = elapsed;
          indPeriodBuf[indBufIdx]   = elapsed;
          indBufIdx                 = (indBufIdx + 1) % MEDIAN_SIZE;
          if (indBufCount < MEDIAN_SIZE) indBufCount++;
        } else {
          indRejectCount++;   // non-blocking: printed in 250ms summary
        }
      }
    } else {
      // Bounced back to HIGH before debounce expired - reset
      indPinLast = HIGH;
    }
  } else {
    // Confirmed LOW - wait for release, no debounce needed on rising edge
    if (raw == HIGH) {
      indPinStable = HIGH;
      indPinLast   = HIGH;
    }
  }
}

// --- Get Raw Speed from Reed (median filtered) -------------------------------
float getSpeedReedRaw() {
  unsigned long lastPulse;
  unsigned long bufCopy[MEDIAN_SIZE];
  int bc;

  noInterrupts();
  lastPulse = reedLastPulseUs;
  bc        = reedBufCount;
  for (int i = 0; i < MEDIAN_SIZE; i++) bufCopy[i] = reedPeriodBuf[i];
  interrupts();

  if (lastPulse == 0) return 0.0;
  if ((micros() - lastPulse) > sensorTimeoutUs) return 0.0;
  if (bc == 0) return 0.0;

  int n = (bc < MEDIAN_SIZE) ? bc : MEDIAN_SIZE;
  unsigned long vb[MEDIAN_SIZE] = {0};
  int src = reedBufIdx;
  for (int i = n - 1; i >= 0; i--) {
    src    = (src - 1 + MEDIAN_SIZE) % MEDIAN_SIZE;
    vb[i]  = bufCopy[src];
  }
  for (int i = 1; i < n; i++) {
    unsigned long key = vb[i]; int j = i - 1;
    while (j >= 0 && vb[j] > key) { vb[j + 1] = vb[j]; j--; }
    vb[j + 1] = key;
  }
  unsigned long medPeriod = vb[n / 2];
  if (medPeriod == 0) return 0.0;
  float rps = 1000000.0 / (float)medPeriod;
  float kmh = rps * wheelCircumM * 3.6;
  return (kmh > 40.0) ? 0.0 : kmh;
}

void updateFilteredReedSpeed() {
  float raw  = getSpeedReedRaw();
  float diff = raw - reedFilteredKmh;
  if      (diff >  RATE_LIMIT_KMH) reedFilteredKmh += RATE_LIMIT_KMH;
  else if (diff < -RATE_LIMIT_KMH) reedFilteredKmh -= RATE_LIMIT_KMH;
  else                              reedFilteredKmh  = raw;
  if (reedFilteredKmh < 0.0)  reedFilteredKmh = 0.0;
  if (reedFilteredKmh > 40.0) reedFilteredKmh = 40.0;
  if (raw == 0.0 && reedFilteredKmh < 1.0) reedFilteredKmh = 0.0;
}

// --- Get Raw Speed from Inductive (median filtered) --------------------------
// FIX 1: skip any zero entries in the ring buffer (not-yet-filled slots).
// FIX 2: copy shared variables under noInterrupts() to avoid torn reads.
float getSpeedInductiveRaw() {
  unsigned long lastPulse;
  unsigned long bufCopy[MEDIAN_SIZE];
  int bc;

  // FIX 2: atomic copy of all shared inductive state
  noInterrupts();
  lastPulse = indLastPulseUs;
  bc        = indBufCount;
  for (int i = 0; i < MEDIAN_SIZE; i++) bufCopy[i] = indPeriodBuf[i];
  interrupts();

  if (lastPulse == 0) return 0.0;
  if ((micros() - lastPulse) > sensorTimeoutUs) return 0.0;
  if (bc == 0) return 0.0;

  int n = (bc < MEDIAN_SIZE) ? bc : MEDIAN_SIZE;

  // Walk back through the ring to get the n most-recent entries
  unsigned long vb[MEDIAN_SIZE] = {0};
  int src = indBufIdx;
  for (int i = n - 1; i >= 0; i--) {
    src   = (src - 1 + MEDIAN_SIZE) % MEDIAN_SIZE;
    vb[i] = bufCopy[src];
  }

  // FIX 1: remove zero entries (unfilled slots) and tighten n
  int validN = 0;
  unsigned long valid[MEDIAN_SIZE] = {0};
  for (int i = 0; i < n; i++) {
    if (vb[i] != 0) valid[validN++] = vb[i];
  }
  if (validN == 0) return 0.0;

  // Insertion sort on the valid entries
  for (int i = 1; i < validN; i++) {
    unsigned long key = valid[i]; int j = i - 1;
    while (j >= 0 && valid[j] > key) { valid[j + 1] = valid[j]; j--; }
    valid[j + 1] = key;
  }

  unsigned long medPeriod = valid[validN / 2];
  if (medPeriod == 0) return 0.0;

  float pps = 1000000.0 / (float)medPeriod;
  float kmh = (pps / INDUCTIVE_PULSES_PER_M) * 3.6;
  return (kmh > 40.0) ? 0.0 : kmh;
}

void updateFilteredIndSpeed() {
  float rawKmh = getSpeedInductiveRaw();
  float diff   = rawKmh - indFilteredKmh;
  if      (diff >  RATE_LIMIT_KMH) indFilteredKmh += RATE_LIMIT_KMH;
  else if (diff < -RATE_LIMIT_KMH) indFilteredKmh -= RATE_LIMIT_KMH;
  else                              indFilteredKmh  = rawKmh;
  if (indFilteredKmh < 0.0)  indFilteredKmh = 0.0;
  if (indFilteredKmh > 40.0) indFilteredKmh = 40.0;
  if (rawKmh == 0.0 && indFilteredKmh < 1.0) indFilteredKmh = 0.0;
}

// --- Get Speed from Active Source --------------------------------------------
float getSpeedKmh() {
  return (speedSource == SRC_REED) ? reedFilteredKmh : indFilteredKmh;
}

// --- Get Active PWM % --------------------------------------------------------
int getActivePwmPercent() {
  if (!motorRunning) return 0;
  if (currentMode == MODE_MANUAL) return pwmPercent;
  float pct = (getSpeedKmh() / AUTO_MAX_SPEED_KMH) * 100.0;
  if (pct < 0.0)   pct = 0.0;
  if (pct > 100.0) pct = 100.0;
  return (int)pct;
}

// --- Apply PWM ---------------------------------------------------------------
void applyPWM() {
  uint32_t duty = map(getActivePwmPercent(), 0, 100, 0, 255);
  ledcWrite(PIN_PWM, duty);
}

// --- Update Display ----------------------------------------------------------
void updateDisplay() {
  char line[21];

  const char* modeStr   = (currentMode == MODE_AUTO) ? "AUTO" : "MANU";
  const char* sensorStr = (speedSource == SRC_REED)  ? "REED" : "IND";
  snprintf(line, sizeof(line), "Ali [%-4s] spd:%-4s ", modeStr, sensorStr);
  lcd.setCursor(0, 0); lcd.print(line);

  int activePwm = getActivePwmPercent();
  if (currentMode == MODE_MANUAL)
    snprintf(line, sizeof(line), "PWM: %3d%%           ", activePwm);
  else
    snprintf(line, sizeof(line), "PWM: %3d%% (auto)    ", activePwm);
  lcd.setCursor(0, 1); lcd.print(line);

  char speedStr[8];
  dtostrf(getSpeedKmh(), 5, 1, speedStr);
  snprintf(line, sizeof(line), "Speed:%s km/h   ", speedStr);
  lcd.setCursor(0, 2); lcd.print(line);

  unsigned long seconds = (millis() - startTime) / 1000;
  snprintf(line, sizeof(line), "%s %4lu:%02lu        ",
           motorRunning ? "RUN " : "STOP",
           seconds / 60, seconds % 60);
  lcd.setCursor(0, 3); lcd.print(line);
}

// --- Execute Start/Stop ------------------------------------------------------
void doStartStop() {
  motorRunning = !motorRunning;
  applyPWM();
  Serial.println(motorRunning ? "Motor STARTED" : "Motor STOPPED");
}

// --- Execute Sensor Switch ---------------------------------------------------
void doSensorSwitch() {
  speedSource = (speedSource == SRC_REED) ? SRC_INDUCTIVE : SRC_REED;
  applyPWM();
  Serial.print("Sensor: ");
  Serial.println(speedSource == SRC_REED ? "REED" : "INDUCTIVE");
}

// --- Handle Button -----------------------------------------------------------
void handleButton() {
  bool raw = digitalRead(PIN_ENC_SW);
  unsigned long now = millis();
  if ((now - lastButtonChange) < debounceDelay) return;

  if (raw == LOW && !buttonDown) {
    buttonDown      = true;
    buttonDownTime  = now;
    longPressHandled = false;
    lastButtonChange = now;
    if (waitingSecondPress && (now - lastReleaseTime) <= doublePressMs) {
      doSensorSwitch();
      waitingSecondPress = false;
      startStopPending   = false;
      longPressHandled   = true;
    }
  }

  if (raw == LOW && buttonDown && !longPressHandled) {
    if ((now - buttonDownTime) >= longPressMs) {
      currentMode = (currentMode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
      longPressHandled   = true;
      waitingSecondPress = false;
      startStopPending   = false;
      applyPWM();
      Serial.print("Mode: ");
      Serial.println(currentMode == MODE_AUTO ? "AUTO" : "MANUAL");
    }
  }

  if (raw == HIGH && buttonDown) {
    buttonDown       = false;
    lastButtonChange = now;
    if (!longPressHandled) {
      if (!waitingSecondPress) {
        waitingSecondPress = true;
        startStopPending   = true;
        lastReleaseTime    = now;
      }
    }
  }

  if (waitingSecondPress && !buttonDown && startStopPending) {
    if ((now - lastReleaseTime) > doublePressMs) {
      doStartStop();
      waitingSecondPress = false;
      startStopPending   = false;
    }
  }
}

// --- Setup -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Ali's Motor Controller - Stage 1 v3");

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.backlight();

  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);
  lastEncCLK = digitalRead(PIN_ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encoderISR, CHANGE);

  pinMode(PIN_REED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), reedISR, FALLING);

  pinMode(PIN_INDUCTIVE, INPUT);

  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PIN_PWM, 0);

  startTime = millis();
  updateDisplay();

  Serial.println("Setup complete.");
  Serial.print("Wheel: "); Serial.print(WHEEL_DIAMETER_MM);
  Serial.print("mm  circ: "); Serial.print(wheelCircumM, 3); Serial.println("m");
  Serial.print("Inductive: "); Serial.print(INDUCTIVE_PULSES_PER_M);
  Serial.println(" pulses/m  debounce=2ms");
  Serial.println("Fixes: 1=null-guard  2=noInterrupts  3=debounce2ms  4=real-edge-ts");
  Serial.println("Short:START/STOP  Double:sensor  Long:AUTO/MANUAL");
}

// --- Loop --------------------------------------------------------------------
void loop() {
  pollInductive();
  handleButton();

  if (encoderChanged) {
    encoderChanged = false;
    if (currentMode == MODE_MANUAL) applyPWM();
  }

  if ((millis() - lastDisplayUpdate) > displayInterval) {
    lastDisplayUpdate = millis();
    updateFilteredReedSpeed();
    updateFilteredIndSpeed();
    if (currentMode == MODE_AUTO && motorRunning) applyPWM();
    updateDisplay();

    // Diagnostic serial output
    Serial.print("IND bc:");   Serial.print(indBufCount);
    Serial.print(" raw:");     Serial.print(getSpeedInductiveRaw(), 1);
    Serial.print(" flt:");     Serial.print(indFilteredKmh, 1);
    Serial.print(" per:");     Serial.print(indPeriodUs);
    Serial.print(" rej:");     Serial.print(indRejectCount);
    Serial.print(" stable:");  Serial.println(indPinStable ? "H" : "L");
  }
}
