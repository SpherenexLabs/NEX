/*
  ============================================================
   🚗 Arduino UNO + CNC Shield V3.0 + A4988 Stepper Drivers
  ============================================================

  🔌 Wiring Connections
  ---------------------

  | Arduino Pin | CNC Shield | A4988 Pin | Connection / Purpose       |
  |-------------|------------|-----------|----------------------------|
  | D2          | X.STEP     | STEP      | Left wheel step signal     |
  | D5          | X.DIR      | DIR       | Left wheel direction       |
  | D3          | Y.STEP     | STEP      | Right wheel step signal    |
  | D6          | Y.DIR      | DIR       | Right wheel direction      |
  | D8          | EN         | EN        | Enable all drivers (LOW=ON)|
  | VMOT (+)    | VMOT       | Power     | 12–36V motor supply (+)    |
  | GND         | GND        | Power     | Motor supply ground        |
  | GND         | GND        | Logic     | Arduino GND ↔ Shield GND   |
  | X Motor     | X Output   | 4-pin     | Left stepper motor         |
  | Y Motor     | Y Output   | 4-pin     | Right stepper motor        |

  🕹️ Serial Commands
  -------------------
  F → Forward  (both wheels forward)
  B → Backward (both wheels backward)
  R → Pivot Right (left fwd, right back)
  L → Pivot Left  (left back, right fwd)
  S → Stop
  + → Increase speed
  - → Decrease speed

  📝 Notes
  --------
  • Set microstepping jumpers (MS1/MS2/MS3) the same for X & Y.
  • Adjust Vref on each A4988 for your motor current rating.
  • If one wheel spins opposite, flip its DIR logic in code
    OR rotate its 4-pin motor connector.



  🕹️ Serial Commands:
  -------------------
  F → Forward (both wheels forward)
  B → Backward (both wheels backward)
  R → Pivot Right (left forward, right backward)
  L → Pivot Left  (left backward, right forward)
  S → Stop
  + → Increase speed
  - → Decrease speed

  📝 Notes:
  ---------
  • Adjust Vref on each A4988 for your motor current.
  • Microstepping jumpers (MS1/MS2/MS3) same for X & Y.
  • If one wheel spins opposite, swap HIGH/LOW in code
    OR flip the motor connector.
*/

// ===== File: xy_serial_drive.ino =====

const int X_STEP = 2,  X_DIR = 5;
const int Y_STEP = 3,  Y_DIR = 6;
const int EN_PIN = 8;     // LOW = enabled

// ---- Speed control (steps per second) ----
volatile unsigned int STEPS_PER_SEC = 800;
unsigned long stepIntervalUs = 1000000UL / 800;

// ---- State machine for drive mode ----
enum Mode { STOP_MODE = 0, FWD_MODE, BACK_MODE, LEFT_MODE, RIGHT_MODE };
volatile Mode mode = STOP_MODE;

unsigned long lastStepUs = 0;

void setup() {
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW); // Enable drivers

  Serial.begin(115200);
  Serial.println(F("Commands: F=forward, B=backward, L=left, R=right, S=stop, +=faster, -=slower"));
  updateStepInterval();
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
      digitalWrite(X_STEP, HIGH);
      digitalWrite(Y_STEP, HIGH);
      delayMicroseconds(4);
      digitalWrite(X_STEP, LOW);
      digitalWrite(Y_STEP, LOW);
    }
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'F': case 'f':
      digitalWrite(X_DIR, LOW);
      digitalWrite(Y_DIR, HIGH);
      mode = FWD_MODE;
      Serial.println(F("Forward"));
      break;

    case 'B': case 'b':
      digitalWrite(X_DIR, HIGH);
      digitalWrite(Y_DIR, LOW);
      mode = BACK_MODE;
      Serial.println(F("Backward"));
      break;

    case 'R': case 'r':
      digitalWrite(X_DIR, LOW);
      digitalWrite(Y_DIR, LOW);
      mode = RIGHT_MODE;
      Serial.println(F("Right (pivot)"));
      break;

    case 'L': case 'l':
      digitalWrite(X_DIR, HIGH);
      digitalWrite(Y_DIR, HIGH);
      mode = LEFT_MODE;
      Serial.println(F("Left (pivot)"));
      break;

    case 'S': case 's':
      mode = STOP_MODE;
      Serial.println(F("Stop"));
      break;

    case '+': if (STEPS_PER_SEC < 5000) STEPS_PER_SEC += 100; updateStepInterval(); break;
    case '-': if (STEPS_PER_SEC > 100) STEPS_PER_SEC -= 100; updateStepInterval(); break;
  }
}

void updateStepInterval() {
  stepIntervalUs = 1000000UL / (unsigned long)STEPS_PER_SEC;
}
