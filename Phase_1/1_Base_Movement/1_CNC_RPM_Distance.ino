/*
  X cloned to Z. Live speed + distance (odometry) readout added.

  Commands:
    F,B,L,R,S  = motion modes
    + / -      = faster / slower  (prints steps/s, RPM, cm/s)
    ?          = print current speed + odometry
    0          = reset odometry (distance = 0)

  Edit these to match your hardware:
    STEPS_PER_REV   (e.g., 200)
    MICROSTEP       (1,2,4,8,16)
    GEAR_RATIO      (motor revs per 1 wheel rev; 1.0 if direct)
    WHEEL_DIAMETER_MM (wheel size)
*/

const int X_STEP = 2,  X_DIR = 5;
const int Y_STEP = 3,  Y_DIR = 6;
const int Z_STEP = 4,  Z_DIR = 7;   // Clone of X
const int EN_PIN = 8;               // LOW = enabled

// ---- Motor & wheel constants ----
const unsigned int STEPS_PER_REV   = 200;   // 1.8° stepper
const unsigned int MICROSTEP       = 1;     // 1/2/4/8/16...
const float        GEAR_RATIO      = 1.0f;  // motor revs per 1 wheel rev (30.0 if 30:1 gearbox)
const float        WHEEL_DIAMETER_MM = 65.0f; // set your wheel diameter
const float        PI_F = 3.14159265f;

const float WHEEL_CIRC_MM = WHEEL_DIAMETER_MM * PI_F;
const float STEPS_PER_WHEEL_REV = (float)STEPS_PER_REV * (float)MICROSTEP * GEAR_RATIO;

// ---- Speed control (steps per second) ----
volatile unsigned int STEPS_PER_SEC = 400;
unsigned long stepIntervalUs = 1000000UL / 800;

// ---- State machine for drive mode ----
enum Mode { STOP_MODE = 0, FWD_MODE, BACK_MODE, LEFT_MODE, RIGHT_MODE };
volatile Mode mode = STOP_MODE;

// ---- Odometry ----
volatile long left_steps = 0;   // X/Z
volatile long right_steps = 0;  // Y
int signX = 0, signY = 0;       // direction (+1 forward, -1 backward)

unsigned long lastStepUs = 0;
unsigned long lastReportMs = 0;
const unsigned long REPORT_INTERVAL_MS = 1000;

// --- Helpers ---
inline float currentRPM() {
  return (float)STEPS_PER_SEC * 60.0f / (float)(STEPS_PER_REV * MICROSTEP);
}
inline float stepsToMm(long steps) {
  return (steps / STEPS_PER_WHEEL_REV) * WHEEL_CIRC_MM;
}
inline float mmToCm(float mm) { return mm * 0.1f; }
inline float currentLinearSpeedCmPerS() {
  // Average of both wheels if they were both forward; sign depends on mode
  // cm/s = (steps/s) * (circumference / steps_per_wheel_rev) / 10
  float cm_per_s = (float)STEPS_PER_SEC * (WHEEL_CIRC_MM / STEPS_PER_WHEEL_REV) / 10.0f;
  // If pivoting, average forward speed is ~0; we show signed average using signs:
  float avgSign = (signX + signY) / 2.0f; // +1, 0, or -1
  return cm_per_s * avgSign;
}

void printSpeed() {
  Serial.print(F("Speed = "));
  Serial.print(STEPS_PER_SEC);
  Serial.print(F(" steps/s  |  RPM ≈ "));
  Serial.print(currentRPM(), 2);
  Serial.print(F("  |  v ≈ "));
  Serial.print(currentLinearSpeedCmPerS(), 2);
  Serial.println(F(" cm/s"));
}

void printOdo() {
  float left_mm  = stepsToMm(left_steps);
  float right_mm = stepsToMm(right_steps);
  float avg_mm   = (left_mm + right_mm) * 0.5f;

  Serial.print(F("Odo: Left="));
  Serial.print(mmToCm(left_mm), 2);
  Serial.print(F(" cm, Right="));
  Serial.print(mmToCm(right_mm), 2);
  Serial.print(F(" cm, Avg="));
  Serial.print(mmToCm(avg_mm), 2);
  Serial.println(F(" cm"));
}

void resetOdo() {
  left_steps = 0;
  right_steps = 0;
  Serial.println(F("Odometry reset (distance = 0)."));
}

void updateStepInterval() {
  stepIntervalUs = 1000000UL / (unsigned long)STEPS_PER_SEC;
}

void setup() {
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);   // Enable drivers
  digitalWrite(X_STEP, LOW);
  digitalWrite(Y_STEP, LOW);
  digitalWrite(Z_STEP, LOW);

  Serial.begin(115200);
  Serial.println(F("Commands: F,B,L,R,S, +,-, ?, 0(reset odo)"));
  Serial.print(F("STEPS/REV=")); Serial.print(STEPS_PER_REV);
  Serial.print(F(", MICROSTEP=")); Serial.print(MICROSTEP);
  Serial.print(F(", GEAR_RATIO=")); Serial.print(GEAR_RATIO, 2);
  Serial.print(F(", WHEEL_DIA(mm)=")); Serial.println(WHEEL_DIAMETER_MM, 1);

  updateStepInterval();
  printSpeed();
  printOdo();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    handleCommand(c);
  }

  if (mode != STOP_MODE) {
    unsigned long now = micros();
    if (now - lastStepUs >= stepIntervalUs) {
      lastStepUs = now;

      // One step pulse on all active axes (X & Z cloned, plus Y)
      digitalWrite(X_STEP, HIGH);
      digitalWrite(Y_STEP, HIGH);
      digitalWrite(Z_STEP, HIGH);
      delayMicroseconds(4);
      digitalWrite(X_STEP, LOW);
      digitalWrite(Y_STEP, LOW);
      digitalWrite(Z_STEP, LOW);

      // --- Odometry update: count signed steps ---
      left_steps  += signX; // X (and Z cloned) share the same sign
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

void handleCommand(char c) {
  switch (c) {
    case 'F': case 'f':
      digitalWrite(X_DIR, LOW);
      digitalWrite(Z_DIR, LOW);   // clone DIR to Z
      digitalWrite(Y_DIR, HIGH);
      signX = +1; signY = +1;
      mode = FWD_MODE;
      Serial.println(F("Forward"));
      break;

    case 'B': case 'b':
      digitalWrite(X_DIR, HIGH);
      digitalWrite(Z_DIR, HIGH);  // clone DIR to Z
      digitalWrite(Y_DIR, LOW);
      signX = -1; signY = -1;
      mode = BACK_MODE;
      Serial.println(F("Backward"));
      break;

    case 'R': case 'r':
      // Pivot right: left forward, right backward
      digitalWrite(X_DIR, LOW);
      digitalWrite(Z_DIR, LOW);
      digitalWrite(Y_DIR, LOW);
      signX = +1; signY = -1;
      mode = RIGHT_MODE;
      Serial.println(F("Right (pivot)"));
      break;

    case 'L': case 'l':
      // Pivot left: left backward, right forward
      digitalWrite(X_DIR, HIGH);
      digitalWrite(Z_DIR, HIGH);
      digitalWrite(Y_DIR, HIGH);
      signX = -1; signY = +1;
      mode = LEFT_MODE;
      Serial.println(F("Left (pivot)"));
      break;

    case 'S': case 's':
      mode = STOP_MODE;
      Serial.println(F("Stop"));
      break;

    case '+':
      if (STEPS_PER_SEC < 5000) STEPS_PER_SEC += 100;
      updateStepInterval();
      Serial.println(F("Faster"));
      printSpeed();
      break;

    case '-':
      if (STEPS_PER_SEC > 100) STEPS_PER_SEC -= 100;
      updateStepInterval();
      Serial.println(F("Slower"));
      printSpeed();
      break;

    case '?':
      printSpeed();
      printOdo();
      break;

    case '0':
      resetOdo();
      break;
  }
}
