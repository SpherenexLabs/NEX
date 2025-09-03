/*
  ============================================================
   Arduino UNO + CNC Shield V3.0 + A4988
   X axis CLONED to Z axis (same STEP & DIR as X)
   ➤ S-curve acceleration/deceleration (jerk-limited) for extra smoothness
   ➤ Safe direction changes (ramp-down → flip DIR → ramp-up)
   ➤ Live speed, RPM, cm/s, and distance (odometry) readout
  ============================================================

  Commands:
    F,B,L,R  : ramp to motion
    S        : smooth stop (ramp down to 0)
    + / -    : change speed setpoint (prints target & current)
    ?        : print speed + odometry
    0        : reset odometry
    1 / 2 / 3: smoothness presets (1=smooth, 2=smoother, 3=buttery)

  Tuning (edit these):
    STEPS_PER_REV, MICROSTEP, GEAR_RATIO, WHEEL_DIAMETER_MM
    MAX_ACCEL_SPS2, MAX_DECEL_SPS2, MAX_JERK_SPS3, DIR_CHANGE_THRESHOLD_SPS

  Behavior:
    • Direction changes ramp down first, then flip DIR, then ramp up.
    • S-curve: acceleration itself ramps at MAX_JERK_SPS3 (no abrupt jumps).
*/

const int X_STEP = 2,  X_DIR = 5;
const int Y_STEP = 3,  Y_DIR = 6;
const int Z_STEP = 4,  Z_DIR = 7;   // Clone of X
const int EN_PIN = 8;               // LOW = enabled

// ---- Motor & wheel constants ----
const unsigned int STEPS_PER_REV     = 200;   // 1.8° stepper
const unsigned int MICROSTEP         = 1;     // 1/2/4/8/16...
const float        GEAR_RATIO        = 1.0f;  // motor revs per wheel rev (e.g., 30.0 for 30:1)
const float        WHEEL_DIAMETER_MM = 65.0f;
const float        PI_F              = 3.14159265f;
const float        WHEEL_CIRC_MM     = WHEEL_DIAMETER_MM * PI_F;
const float        STEPS_PER_WHEEL_REV = (float)STEPS_PER_REV * (float)MICROSTEP * GEAR_RATIO;

// ---- Ramp tuning (increase smoothness by LOWERING these, especially JERK) ----
float MAX_ACCEL_SPS2  = 600.0f;   // max accel when speeding up   (steps/s^2)
float MAX_DECEL_SPS2  = 800.0f;   // max decel when slowing down  (steps/s^2)
float MAX_JERK_SPS3   = 1200.0f;  // max change of accel per sec  (steps/s^3)  ← lower = smoother
const float DIR_CHANGE_THRESHOLD_SPS = 12.0f; // flip DIR when speed below this

// ---- Speed setpoint & live value (steps per second) ----
unsigned int SETPOINT_SPS = 800; // user target via +/- (cap enforced below)
const unsigned int SPS_MIN = 100;
const unsigned int SPS_MAX = 5000;

float  targetSPS = 0.0f;  // the immediate target the ramp chases (0 during stop or dir-change)
float  curSPS    = 0.0f;  // live, smoothed speed
float  accelSPS  = 0.0f;  // live acceleration (steps/s^2)
unsigned long stepIntervalUs = 0;

// ---- State machine / direction signs ----
enum Mode { STOP_MODE = 0, FWD_MODE, BACK_MODE, LEFT_MODE, RIGHT_MODE };
Mode mode = STOP_MODE;

int signX = 0, signY = 0;  // +1 forward, -1 backward, 0 stopped

// Defer direction change until nearly stopped
bool pendingDirChange = false;
int  desiredSignX = 0, desiredSignY = 0;
bool desiredDirX  = LOW, desiredDirY  = HIGH; // actual pin levels to set later

// Track current DIR pins (so we only write when needed)
bool curDirX = LOW, curDirY = HIGH;

// ---- Odometry ----
volatile long left_steps = 0;   // X/Z
volatile long right_steps = 0;  // Y

// ---- Timing ----
unsigned long lastStepUs = 0;
unsigned long lastSpeedUpdateMs = 0;
unsigned long lastReportMs = 0;
const unsigned long REPORT_INTERVAL_MS = 1000;

// --- Helpers ---
inline float stepsToMm(long steps) {
  return (steps / STEPS_PER_WHEEL_REV) * WHEEL_CIRC_MM;
}
inline float mmToCm(float mm) { return mm * 0.1f; }

inline float currentRPM(float sps) {
  return sps * 60.0f / (float)(STEPS_PER_REV * MICROSTEP);
}
inline float linearSpeedCmPerS(float sps) {
  // cm/s = (steps/s) * (circumference / steps_per_wheel_rev) / 10
  return (sps * (WHEEL_CIRC_MM / STEPS_PER_WHEEL_REV)) / 10.0f * ((signX + signY) / 2.0f);
}

void printSpeed() {
  Serial.print(F("Target="));
  Serial.print(SETPOINT_SPS);
  Serial.print(F(" sps | Current="));
  Serial.print(curSPS, 1);
  Serial.print(F(" sps (RPM≈"));
  Serial.print(currentRPM(curSPS), 2);
  Serial.print(F(") | v≈ "));
  Serial.print(linearSpeedCmPerS(curSPS), 2);
  Serial.println(F(" cm/s"));
}

void printOdo() {
  float left_cm  = mmToCm(stepsToMm(left_steps));
  float right_cm = mmToCm(stepsToMm(right_steps));
  float avg_cm   = (left_cm + right_cm) * 0.5f;
  Serial.print(F("Odo: L=")); Serial.print(left_cm, 2);
  Serial.print(F(" cm, R=")); Serial.print(right_cm, 2);
  Serial.print(F(" cm, Avg=")); Serial.print(avg_cm, 2);
  Serial.println(F(" cm"));
}

void resetOdo() {
  left_steps = 0;
  right_steps = 0;
  Serial.println(F("Odometry reset."));
}

void clampSetpoint() {
  if (SETPOINT_SPS < SPS_MIN) SETPOINT_SPS = SPS_MIN;
  if (SETPOINT_SPS > SPS_MAX) SETPOINT_SPS = SPS_MAX;
}

void engageDesiredDirection() {
  curDirX = desiredDirX;  curDirY = desiredDirY;
  digitalWrite(X_DIR, curDirX);
  digitalWrite(Z_DIR, curDirX);   // clone X
  digitalWrite(Y_DIR, curDirY);
  signX = desiredSignX;   signY = desiredSignY;
  pendingDirChange = false;
}

void startDirChange(Mode newMode, bool dirX, bool dirY, int sX, int sY, const __FlashStringHelper* label) {
  mode = newMode;
  desiredDirX = dirX; desiredDirY = dirY;
  desiredSignX = sX;  desiredSignY = sY;

  if (signX == desiredSignX && signY == desiredSignY) {
    // Same direction—just ramp to setpoint
    targetSPS = (float)SETPOINT_SPS;
    Serial.print(F("Hold ")); Serial.print(label); Serial.println(F(", ramping to target."));
  } else {
    // Ramp down first, then flip
    pendingDirChange = true;
    targetSPS = 0.0f;
    Serial.print(F("Ramping down for ")); Serial.print(label); Serial.println(F("..."));
  }
}

// === JERK-LIMITED S-CURVE RAMP ===
void updateSpeedRamp() {
  unsigned long ms = millis();
  float dt = (ms - lastSpeedUpdateMs) / 1000.0f;
  if (dt <= 0) { return; }
  lastSpeedUpdateMs = ms;

  // Decide desired acceleration sign
  float accelWanted = 0.0f;
  if (targetSPS > curSPS + 0.5f) {
    accelWanted = MAX_ACCEL_SPS2;     // speed up
  } else if (targetSPS < curSPS - 0.5f) {
    accelWanted = -MAX_DECEL_SPS2;    // slow down
  } else {
    accelWanted = 0.0f;               // near target, hold/bleed accel
  }

  // Jerk limit: move accelSPS toward accelWanted, bounded by MAX_JERK_SPS3
  float dAmax = MAX_JERK_SPS3 * dt;
  float deltaA = accelWanted - accelSPS;
  if (deltaA >  dAmax) deltaA =  dAmax;
  if (deltaA < -dAmax) deltaA = -dAmax;
  accelSPS += deltaA;

  // Integrate acceleration → speed
  curSPS += accelSPS * dt;

  // Prevent overshoot & negative speeds
  if ((accelWanted >= 0.0f && curSPS > targetSPS) ||
      (accelWanted <= 0.0f && curSPS < targetSPS)) {
    curSPS = targetSPS;
    accelSPS = 0.0f;
  }
  if (curSPS < 0.0f) { curSPS = 0.0f; accelSPS = 0.0f; }

  // Safe direction flip when nearly stopped
  if (pendingDirChange && curSPS <= DIR_CHANGE_THRESHOLD_SPS) {
    engageDesiredDirection();
    targetSPS = (float)SETPOINT_SPS;    // now ramp up
    Serial.println(F("Direction engaged; ramping up."));
  }

  // derive pulse interval (guard zero)
  if (curSPS >= 1.0f) {
    stepIntervalUs = (unsigned long)(1000000.0f / curSPS);
  } else {
    stepIntervalUs = 0;
  }
}

void setup() {
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);   // Enable drivers
  digitalWrite(X_STEP, LOW); digitalWrite(Y_STEP, LOW); digitalWrite(Z_STEP, LOW);

  // Initial DIR (won't move until speed > 0)
  curDirX = LOW;  curDirY = HIGH;
  digitalWrite(X_DIR, curDirX); digitalWrite(Z_DIR, curDirX); digitalWrite(Y_DIR, curDirY);

  Serial.begin(115200);
  Serial.println(F("Commands: F,B,L,R,S, +,-, ?, 0 (reset odo)"));
  Serial.print(F("STEPS/REV=")); Serial.print(STEPS_PER_REV);
  Serial.print(F(", MICROSTEP=")); Serial.print(MICROSTEP);
  Serial.print(F(", GEAR_RATIO=")); Serial.print(GEAR_RATIO, 2);
  Serial.print(F(", WHEEL_DIA(mm)=")); Serial.println(WHEEL_DIAMETER_MM, 1);

  // Announce smoothing knobs
  Serial.print(F("S-curve: MAX_ACCEL=")); Serial.print(MAX_ACCEL_SPS2,0);
  Serial.print(F(", MAX_DECEL=")); Serial.print(MAX_DECEL_SPS2,0);
  Serial.print(F(", MAX_JERK=")); Serial.println(MAX_JERK_SPS3,0);

  clampSetpoint();

  // Start stopped
  targetSPS = 0.0f; curSPS = 0.0f; accelSPS = 0.0f;
  lastSpeedUpdateMs = millis();
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'F': case 'f': startDirChange(FWD_MODE,  LOW,  HIGH, +1, +1, F("Forward"));  break;
      case 'B': case 'b': startDirChange(BACK_MODE, HIGH, LOW,  -1, -1, F("Backward")); break;
      case 'R': case 'r': startDirChange(RIGHT_MODE,LOW,  LOW,  +1, -1, F("Right"));    break;
      case 'L': case 'l': startDirChange(LEFT_MODE, HIGH, HIGH, -1, +1, F("Left"));     break;

      case 'S': case 's':
        mode = STOP_MODE;
        pendingDirChange = false;   // cancel any pending flip
        targetSPS = 0.0f;           // ramp down to a halt (S-curve)
        Serial.println(F("Coasting to stop (S-curve)..."));
        break;

      case '+':
        if (SETPOINT_SPS < SPS_MAX) SETPOINT_SPS += 100;
        clampSetpoint();
        if (mode != STOP_MODE && !pendingDirChange) targetSPS = (float)SETPOINT_SPS;
        Serial.print(F("Target set to ")); Serial.print(SETPOINT_SPS); Serial.println(F(" sps"));
        printSpeed();
        break;

      case '-':
        if (SETPOINT_SPS > SPS_MIN) SETPOINT_SPS -= 100;
        clampSetpoint();
        if (mode != STOP_MODE && !pendingDirChange) targetSPS = (float)SETPOINT_SPS;
        Serial.print(F("Target set to ")); Serial.print(SETPOINT_SPS); Serial.println(F(" sps"));
        printSpeed();
        break;

      case '?':
        printSpeed();
        printOdo();
        break;

      case '0':
        resetOdo();
        break;

      case '1': // optional quick presets for smoothness (1=smooth, 2=smoother, 3=buttery)
        MAX_ACCEL_SPS2 = 800; MAX_DECEL_SPS2 = 1000; MAX_JERK_SPS3 = 2000; Serial.println(F("Preset 1 (smooth).")); break;
      case '2':
        MAX_ACCEL_SPS2 = 600; MAX_DECEL_SPS2 = 800;  MAX_JERK_SPS3 = 1200; Serial.println(F("Preset 2 (smoother).")); break;
      case '3':
        MAX_ACCEL_SPS2 = 400; MAX_DECEL_SPS2 = 600;  MAX_JERK_SPS3 = 600;  Serial.println(F("Preset 3 (buttery).")); break;
    }
  }

  // Update ramp and step timing
  updateSpeedRamp();

  // Generate steps if moving
  if (stepIntervalUs > 0) {
    unsigned long now = micros();
    if (now - lastStepUs >= stepIntervalUs) {
      lastStepUs = now;

      // Pulse all axes (X & Z cloned, plus Y)
      digitalWrite(X_STEP, HIGH);
      digitalWrite(Y_STEP, HIGH);
      digitalWrite(Z_STEP, HIGH);
      delayMicroseconds(4);
      digitalWrite(X_STEP, LOW);
      digitalWrite(Y_STEP, LOW);
      digitalWrite(Z_STEP, LOW);

      // Odometry: signed counts
      left_steps  += signX;
      right_steps += signY;
    }
  }

  // Periodic status
  unsigned long ms = millis();
  if (ms - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = ms;
    printSpeed();
    printOdo();
  }
}
