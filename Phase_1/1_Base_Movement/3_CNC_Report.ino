
/*
  === GLITCH-FREE + SESSION REPORT + TURN DRIFT CORRECTION ===
  - One-shot prints only (no background spam)
  - Press '1' to start logging, '0' to stop & print report
  - F/B/L/R for motion, S for smooth stop, +/- to change speed, ? for one-shot status
  - In-place turns get per-side scaling to keep net distance ≈ 0 cm
  - STOP 'S' creates its own segment with distance=0 (steps ignored during decel)
  - Ramp tuned to Preset 3 (buttery smooth)
*/
#include <Arduino.h>
#include <math.h>
const int X_STEP = 2,  X_DIR = 5;
const int Y_STEP = 3,  Y_DIR = 6;
const int Z_STEP = 4,  Z_DIR = 7;   // clone X
const int EN_PIN = 8;               // LOW = enabled
// ---- Motor & wheel constants ----
const unsigned int STEPS_PER_REV     = 200;
const unsigned int MICROSTEP         = 1;       // set to your jumper setting!
const float        GEAR_RATIO        = 1.0f;
const float        WHEEL_DIAMETER_MM = 65.0f;
const float        PI_F              = 3.14159265f;
const float        WHEEL_CIRC_MM     = WHEEL_DIAMETER_MM * PI_F;
const float        STEPS_PER_WHEEL_REV = (float)STEPS_PER_REV * (float)MICROSTEP * GEAR_RATIO;
// ---- Ramp tuning (Preset 3 "buttery") ----
float MAX_ACCEL_SPS2  = 200.0f;
float MAX_DECEL_SPS2  = 400.0f;
float MAX_JERK_SPS3   = 400.0f;
const float DIR_CHANGE_THRESHOLD_SPS = 12.0f;
// ---- Speed / timing ----
unsigned int SETPOINT_SPS = 300;
const unsigned int SPS_MIN = 100;
const unsigned int SPS_MAX = 5000;
float  targetSPS = 0.0f;     // baseline SPS both sides derive from
float  curSPS    = 0.0f;     // ramped SPS (baseline)
float  accelSPS  = 0.0f;
unsigned long stepIntervalUsL = 0;  // LEFT side (X,Z)
unsigned long stepIntervalUsR = 0;  // RIGHT side (Y)
// Per-side turn scaling (tune these!)
float TURN_RIGHT_SCALE = 0.96f;  // when turning RIGHT, scale RIGHT side SPS (0.90–1.10)
float TURN_LEFT_SCALE  = 0.96f;  // when turning LEFT,  scale LEFT  side SPS (0.90–1.10)
enum Mode { STOP_MODE = 0, FWD_MODE, BACK_MODE, LEFT_MODE, RIGHT_MODE };
Mode mode = STOP_MODE;
int  signX = 0, signY = 0;  // +1 forward, -1 backward, 0 stopped
bool pendingDirChange = false;
int  desiredSignX = 0, desiredSignY = 0;
bool desiredDirX  = LOW, desiredDirY  = HIGH;
bool curDirX      = LOW, curDirY      = HIGH;
// ---- Odometry (raw counts keep running) ----
volatile long left_steps = 0;   // X/Z
volatile long right_steps = 0;  // Y
// ---- Timing ----
unsigned long lastStepUsL = 0;  // LEFT timer
unsigned long lastStepUsR = 0;  // RIGHT timer
unsigned long lastSpeedUpdateMs = 0;
/* ------------------ SESSION LOGGER ------------------ */
struct Segment {
  char cmd;                    // 'F','B','L','R','S'
  bool ignoreSteps;            // true for 'S' so distance=0
  unsigned long t0_ms, t1_ms;
  long l0, r0, l1, r1;
};
const uint8_t MAX_SEGMENTS = 64;
Segment segs[MAX_SEGMENTS];
uint8_t segCount = 0;
bool sessionActive = false;
bool segmentOpen = false;
// forward declare closeSegment() since updateSpeedRamp calls it
void closeSegment();
/* ---------------------------------------------------- */
// --- Helpers ---
inline float stepsToMm(long steps) {
  return (steps / STEPS_PER_WHEEL_REV) * WHEEL_CIRC_MM;
}
inline float mmToCm(float mm) { return mm * 0.1f; }
inline float currentRPM(float sps) {
  return sps * 60.0f / (float)(STEPS_PER_REV * MICROSTEP);
}
inline float linearSpeedCmPerS(float sps) {
  // Only meaningful for F/B; for L/R (in-place) average becomes ~0
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
void startDirChange(Mode newMode, bool dirX, bool dirY, int sX, int sY, char label) {
  mode = newMode;
  desiredDirX = dirX; desiredDirY = dirY;
  desiredSignX = sX;  desiredSignY = sY;
  if (signX == desiredSignX && signY == desiredSignY) {
    targetSPS = (float)SETPOINT_SPS;
  } else {
    pendingDirChange = true;
    targetSPS = 0.0f; // ramp down first
  }
  Serial.print(F("[CMD] Started "));
  Serial.println(label);
}
// === JERK-LIMITED S-CURVE RAMP ===
void updateSpeedRamp() {
  unsigned long ms = millis();
  float dt = (ms - lastSpeedUpdateMs) / 1000.0f;
  if (dt <= 0) { return; }
  lastSpeedUpdateMs = ms;
  float accelWanted = 0.0f;
  if (targetSPS > curSPS + 0.5f)      accelWanted = MAX_ACCEL_SPS2;
  else if (targetSPS < curSPS - 0.5f) accelWanted = -MAX_DECEL_SPS2;
  else                                accelWanted = 0.0f;
  float dAmax = MAX_JERK_SPS3 * dt;
  float deltaA = accelWanted - accelSPS;
  if (deltaA >  dAmax) deltaA =  dAmax;
  if (deltaA < -dAmax) deltaA = -dAmax;
  accelSPS += deltaA;
  curSPS += accelSPS * dt;
  if ((accelWanted >= 0.0f && curSPS > targetSPS) ||
      (accelWanted <= 0.0f && curSPS < targetSPS)) {
    curSPS = targetSPS;
    accelSPS = 0.0f;
  }
  if (curSPS < 0.0f) { curSPS = 0.0f; accelSPS = 0.0f; }
  // Only auto-close when the OPEN segment is 'S' and we've decelerated to ~0
  if (sessionActive && segmentOpen && segCount > 0 && segs[segCount-1].cmd == 'S' && curSPS < 1.0f) {
    closeSegment();
  }
  // Safe direction flip when nearly stopped
  if (pendingDirChange && curSPS <= DIR_CHANGE_THRESHOLD_SPS) {
    engageDesiredDirection();
    targetSPS = (float)SETPOINT_SPS; // ramp up after flip
  }
  // ---- derive per-side intervals (allow slight mismatch on turns) ----
  float spsL = curSPS;
  float spsR = curSPS;
  if (mode == RIGHT_MODE) {
    spsR *= TURN_RIGHT_SCALE;
  } else if (mode == LEFT_MODE) {
    spsL *= TURN_LEFT_SCALE;
  }
  stepIntervalUsL = (spsL >= 1.0f) ? (unsigned long)(1000000.0f / spsL) : 0;
  stepIntervalUsR = (spsR >= 1.0f) ? (unsigned long)(1000000.0f / spsR) : 0;
}
/* ------------------ SESSION LOGGER IMPLEMENTATION ------------------ */
void closeSegment() {
  if (!sessionActive || !segmentOpen || segCount == 0) return;
  Segment &s = segs[segCount-1];
  s.t1_ms = millis();
  if (s.ignoreSteps) {
    // For 'S', force zero distance
    s.l1 = s.l0;
    s.r1 = s.r0;
  } else {
    s.l1 = left_steps;
    s.r1 = right_steps;
  }
  segmentOpen = false;
}
void openSegment(char c, bool ignore=false) {
  if (!sessionActive) return;
  closeSegment();
  if (segCount >= MAX_SEGMENTS) return;
  Segment &s = segs[segCount++];
  s.cmd = c;
  s.ignoreSteps = ignore;
  s.t0_ms = millis();
  s.l0 = left_steps;
  s.r0 = right_steps;
  s.l1 = s.l0;
  s.r1 = s.r0;
  s.t1_ms = s.t0_ms;
  segmentOpen = true;
}
void printReportAndClear() {
  closeSegment();
  Serial.println(F("----------- SESSION REPORT -----------"));
  Serial.println(F("Idx CMD dur(s) Lsteps Rsteps avgSPS avgRPM dist(cm)"));
  for (uint8_t i = 0; i < segCount; i++) {
    Segment &s = segs[i];
    unsigned long dt_ms = (s.t1_ms >= s.t0_ms) ? (s.t1_ms - s.t0_ms) : 0;
    float dt_s = dt_ms / 1000.0f;
    long dL = s.l1 - s.l0;
    long dR = s.r1 - s.r0;
    float avgSteps = (fabs((float)dL) + fabs((float)dR)) * 0.5f;
    float avgSPS = (dt_s > 0.0f) ? (avgSteps / dt_s) : 0.0f;
    float avgRPM = currentRPM(avgSPS);
    float left_cm  = mmToCm(stepsToMm(dL));
    float right_cm = mmToCm(stepsToMm(dR));
    float dist_cm  = (left_cm + right_cm) * 0.5f;
    Serial.print(i); Serial.print("  ");
    Serial.print(s.cmd); Serial.print("   ");
    Serial.print(dt_s,2); Serial.print("   ");
    Serial.print(dL); Serial.print("   ");
    Serial.print(dR); Serial.print("   ");
    Serial.print(avgSPS,1); Serial.print("   ");
    Serial.print(avgRPM,2); Serial.print("   ");
    Serial.println(dist_cm,2);
  }
  Serial.println(F("-------------------------------------"));
  segCount = 0; sessionActive = false; segmentOpen = false;
}
/* ---------------------------------------------------- */
void clampSetpoint();
void engageDesiredDirection();
void startDirChange(Mode newMode, bool dirX, bool dirY, int sX, int sY, char label);
void updateSpeedRamp();
void setup() {
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(X_STEP, LOW); digitalWrite(Y_STEP, LOW); digitalWrite(Z_STEP, LOW);
  // Initial DIR (X forward LOW, Y forward HIGH) – adjust if your wiring differs
  curDirX = LOW;  curDirY = HIGH;
  digitalWrite(X_DIR, curDirX); digitalWrite(Z_DIR, curDirX); digitalWrite(Y_DIR, curDirY);
  Serial.begin(9600);
  Serial.println(F("Commands: 1=start log, 0=stop+report, F/B/L/R moves, S=stop, +/- speed, ?=status"));
  clampSetpoint();
  targetSPS = 0.0f; curSPS = 0.0f; accelSPS = 0.0f;
  lastSpeedUpdateMs = millis();
}
void loop() {
  // Commands
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1':
        sessionActive = true; segCount=0; segmentOpen=false;
        Serial.println(F("[LOG] Session started."));
        break;
      case '0':
        if (sessionActive) {
          Serial.println(F("[LOG] Report:"));
          printReportAndClear();
        } else Serial.println(F("[LOG] No session."));
        break;
      case 'F': case 'f':
        startDirChange(FWD_MODE,  LOW,  HIGH, +1, +1, 'F');
        if (sessionActive) openSegment('F');
        break;
      case 'B': case 'b':
        startDirChange(BACK_MODE, HIGH, LOW,  -1, -1, 'B');
        if (sessionActive) openSegment('B');
        break;
      case 'R': case 'r':
        startDirChange(RIGHT_MODE,LOW,  LOW,  +1, -1, 'R');
        if (sessionActive) openSegment('R');
        break;
      case 'L': case 'l':
        startDirChange(LEFT_MODE, HIGH, HIGH, -1, +1, 'L');
        if (sessionActive) openSegment('L');
        break;
      case 'S': case 's':   // Smooth stop — create an 'S' segment with steps ignored
        pendingDirChange = false;
        if (sessionActive && segmentOpen && segCount > 0 && segs[segCount-1].cmd != 'S') {
          closeSegment();
        }
        if (sessionActive) openSegment('S', /*ignore=*/true);
        targetSPS = 0.0f;
        Serial.println(F("[CMD] Stop"));
        break;
      case '+':
        if (SETPOINT_SPS < SPS_MAX) SETPOINT_SPS += 100;
        clampSetpoint();
        if (mode != STOP_MODE && !pendingDirChange) targetSPS = (float)SETPOINT_SPS;
        break;
      case '-':
        if (SETPOINT_SPS > SPS_MIN) SETPOINT_SPS -= 100;
        clampSetpoint();
        if (mode != STOP_MODE && !pendingDirChange) targetSPS = (float)SETPOINT_SPS;
        break;
      case '?':
        printSpeed(); printOdo();
        break;
    }
  }
  // Ramp baseline speed
  updateSpeedRamp();
  // ---- LEFT side timing (X & Z) ----
  if (stepIntervalUsL > 0) {
    unsigned long nowL = micros();
    if (nowL - lastStepUsL >= stepIntervalUsL) {
      lastStepUsL = nowL;
      digitalWrite(X_STEP, HIGH);
      digitalWrite(Z_STEP, HIGH);
      delayMicroseconds(4);
      digitalWrite(X_STEP, LOW);
      digitalWrite(Z_STEP, LOW);
      left_steps += signX;
    }
  }
  // ---- RIGHT side timing (Y) ----
  if (stepIntervalUsR > 0) {
    unsigned long nowR = micros();
    if (nowR - lastStepUsR >= stepIntervalUsR) {
      lastStepUsR = nowR;
      digitalWrite(Y_STEP, HIGH);
      delayMicroseconds(4);
      digitalWrite(Y_STEP, LOW);
      right_steps += signY;
    }
  }
}
