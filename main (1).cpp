// Homing test: MT6701 (I2C) on NEMA17 shaft + hall sensor on output arm.
// Serial commands (115200):  h = home   r = repeatability test (10x)
//                            j / k = jog + / -   p = print status   x = motors off   m = motors on
//
// How it works: the hall sensor fires at one fixed output angle. We seek it in
// one direction (coarse/fast), back off until it releases, then re-approach
// slowly from the same direction. The MT6701 count at that slow trigger is the
// home reference, so homing is always an approach from the same side.

#include <Arduino.h>
#include <Wire.h>

// ---------------- CONFIG: adjust to your wiring ----------------
constexpr int PIN_STEP = 25;
constexpr int PIN_DIR  = 26;
constexpr int PIN_EN   = 27;
constexpr int PIN_HALL = 32;   // avoid 34-39: input-only, no internal pullup
constexpr int PIN_SDA  = 21;
constexpr int PIN_SCL  = 22;

constexpr uint8_t MT6701_ADDR = 0x06;

constexpr float GEAR_RATIO       = 29.0f;   // N=30 ring pins -> 29:1. Change if you alter the design
constexpr int   STEPS_PER_REV    = 200 * 16; // 1.8 deg motor * microstepping (set to your driver's setting)
constexpr bool  HALL_ACTIVE_LOW  = true;     // A3144/US1881-style open-collector sensors pull low at the magnet
constexpr bool  EN_ACTIVE_LOW    = true;     // A4988/DRV8825/TMC2209 enable pins are active low
constexpr int   HOME_DIR         = 1;        // DIR pin level that moves the arm toward the hall sensor
constexpr float SEEK_LIMIT_OUT_DEG = 120.0f; // max OUTPUT travel to search before giving up (protect hard stops!)
constexpr float AWAY_DEG         = 15.0f;    // output degrees to move away between repeatability runs

constexpr uint32_t SEEK_US = 600;   // step period, coarse seek
constexpr uint32_t FINE_US = 2500;  // step period, fine approach
// ----------------------------------------------------------------

constexpr int32_t ENC_CPR = 16384;  // MT6701 = 14 bit

// ---- encoder state (unwrapped multi-turn count on the motor shaft) ----
int32_t encTurns = 0;
int32_t lastRaw = 0;
int32_t encCounts = 0;
bool    encInit = false;
int32_t homeCounts = 0;
bool    homed = false;
int32_t cmdSteps = 0;   // net commanded steps (signed, +1 per step with DIR=HIGH)

bool readRaw(int32_t &raw) {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03);                      // ANGLE[13:6]; reg 0x04 holds ANGLE[5:0] in bits 7:2
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MT6701_ADDR, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = ((int32_t)hi << 6) | (lo >> 2);
  return true;
}

bool updateEncoder() {
  int32_t raw;
  if (!readRaw(raw)) return false;
  if (!encInit) { lastRaw = raw; encInit = true; }
  int32_t d = raw - lastRaw;
  if (d >  ENC_CPR / 2) encTurns--;      // wrapped backwards
  if (d < -ENC_CPR / 2) encTurns++;      // wrapped forwards
  lastRaw = raw;
  encCounts = encTurns * ENC_CPR + raw;
  return true;
}

float motorDeg(int32_t counts) { return counts * 360.0f / ENC_CPR; }
float outputDeg(int32_t counts) { return motorDeg(counts) / GEAR_RATIO; }

bool hallActive() {
  bool level = digitalRead(PIN_HALL);
  return HALL_ACTIVE_LOW ? !level : level;
}

void motorEnable(bool on) { digitalWrite(PIN_EN, (on == EN_ACTIVE_LOW) ? LOW : HIGH); }

void stepOnce(int dir, uint32_t periodUs) {
  digitalWrite(PIN_DIR, dir);
  delayMicroseconds(5);
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(3);
  digitalWrite(PIN_STEP, LOW);
  cmdSteps += dir ? 1 : -1;
  delayMicroseconds(periodUs);
  updateEncoder();                       // keep the unwrap tracker current (must see < half a turn between reads)
}

// Step in `dir` until the hall sensor reaches `wantActive`. Returns false if maxSteps is hit first.
bool seekHall(int dir, bool wantActive, uint32_t periodUs, uint32_t maxSteps) {
  for (uint32_t i = 0; i < maxSteps; i++) {
    if (hallActive() == wantActive) return true;
    stepOnce(dir, periodUs);
  }
  return hallActive() == wantActive;
}

void moveSteps(int dir, uint32_t steps, uint32_t periodUs) {
  for (uint32_t i = 0; i < steps; i++) stepOnce(dir, periodUs);
}

uint32_t outDegToSteps(float deg) { return (uint32_t)(deg / 360.0f * GEAR_RATIO * STEPS_PER_REV); }

bool homeOnce() {
  const int away = !HOME_DIR;
  const uint32_t coarseMax = outDegToSteps(SEEK_LIMIT_OUT_DEG);
  const uint32_t fineMax   = STEPS_PER_REV * 2;   // 2 motor revs of slack for the fine passes

  // If we start on the magnet, back off so the approach is always from the same side.
  if (hallActive()) {
    Serial.println("  starting on sensor, backing off...");
    if (!seekHall(away, false, SEEK_US, fineMax * 4)) { Serial.println("  FAIL: can't leave sensor zone"); return false; }
    moveSteps(away, STEPS_PER_REV / 4, SEEK_US);    // extra margin
  }

  Serial.println("  coarse seek...");
  if (!seekHall(HOME_DIR, true, SEEK_US, coarseMax)) {
    Serial.printf("  FAIL: no hall trigger within %.0f deg output. Check sensor/magnet/polarity/direction.\n", SEEK_LIMIT_OUT_DEG);
    return false;
  }

  Serial.println("  backing off...");
  if (!seekHall(away, false, FINE_US, fineMax)) { Serial.println("  FAIL: back-off"); return false; }

  Serial.println("  fine approach...");
  if (!seekHall(HOME_DIR, true, FINE_US, fineMax)) { Serial.println("  FAIL: fine approach"); return false; }

  homeCounts = encCounts;
  homed = true;
  Serial.printf("  HOMED. trigger @ %ld counts (motor %.3f deg, output %.4f deg from power-on ref)\n",
                (long)homeCounts, motorDeg(homeCounts), outputDeg(homeCounts));
  return true;
}

void repeatabilityTest() {
  const int N = 10;
  float results[N];
  int n = 0;
  for (int i = 0; i < N; i++) {
    Serial.printf("Run %d/%d\n", i + 1, N);
    if (i > 0) moveSteps(!HOME_DIR, outDegToSteps(AWAY_DEG), SEEK_US);
    if (!homeOnce()) { Serial.println("Aborted."); break; }
    results[n++] = outputDeg(homeCounts);
    delay(300);
  }
  if (n < 2) return;
  float mn = results[0], mx = results[0], sum = 0;
  for (int i = 0; i < n; i++) { mn = min(mn, results[i]); mx = max(mx, results[i]); sum += results[i]; }
  Serial.printf("\n%d runs: mean %.4f deg, min %.4f, max %.4f, spread %.4f deg (output)\n", n, sum / n, mn, mx, mx - mn);
}

void jog(int dir) {
  int32_t c0 = encCounts, s0 = cmdSteps;
  moveSteps(dir, STEPS_PER_REV / 4, FINE_US);   // quarter motor rev
  delay(50);
  updateEncoder();
  float cmdDeg = (cmdSteps - s0) * 360.0f / STEPS_PER_REV;
  float encDeg = motorDeg(encCounts - c0);
  Serial.printf("jog: commanded %.2f deg, encoder saw %.2f deg (motor shaft)\n", cmdDeg, encDeg);
  Serial.println("  (opposite sign = flip the encoder/motor direction; big mismatch = missed steps)");
}

void printStatus() {
  updateEncoder();
  Serial.printf("raw %ld  motor %.2f deg  hall %s", (long)(encCounts - encTurns * ENC_CPR),
                motorDeg(encCounts), hallActive() ? "ACTIVE" : "inactive");
  if (homed) Serial.printf("  output(from home) %.3f deg", outputDeg(encCounts - homeCounts));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_HALL, INPUT_PULLUP);
  motorEnable(true);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  delay(500);
  if (updateEncoder()) Serial.println("MT6701 OK");
  else Serial.println("MT6701 NOT FOUND - check SDA/SCL, 3.3V, and that the chip is in I2C mode");
  Serial.println("Commands: h home | r repeatability x10 | j/k jog +/- | p status | x motors off | m motors on");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'h': Serial.println("Homing..."); homeOnce(); break;
    case 'r': repeatabilityTest(); break;
    case 'j': jog(1); break;
    case 'k': jog(0); break;
    case 'p': printStatus(); break;
    case 'x': motorEnable(false); Serial.println("motors off"); break;
    case 'm': motorEnable(true);  Serial.println("motors on");  break;
    default: break;
  }
}
