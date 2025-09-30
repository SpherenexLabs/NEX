// SmartElex 10D/15D + Arduino Mega — ONE MOTOR
// 10 s delay, then exactly 2 rotations (smooth), then stop.

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ---------- PINS ----------
const uint8_t PWM1 = 9;   // to S1 (PWM1)
const uint8_t DIR1 = 7;   // to DIR1
const uint8_t ENC_A = 2;  // encoder A (INT)
const uint8_t ENC_B = 3;  // encoder B (INT)

// ---------- USER SETTINGS ----------
const float    COUNTS_PER_REV = 2540; //1980.0f; // <-- REPLACE with your measured CPR
const float    ROTATIONS       = 1.0f;   // move 2 full turns
const uint16_t START_DELAY_MS  = 10000;  // 10 s delay before starting

// ⬇ Speed knobs (lower/slower, higher/faster) ⬇
const uint8_t  SPEED_LIMIT_DUTY   = 10;   // main speed cap (0..255)
const int      RAMP_STEP          = 3;    // accel/decel smoothness (smaller = gentler)
const float    KP_DUTY_PER_COUNT  = 6.0f; // maps remaining counts -> duty (smaller = gentler)
const int      MIN_MOVE_DUTY      = 22;   // minimal duty near the end to avoid stalling
// ⬆ Speed knobs ⬆

// Driver-friendly PWM window
const uint8_t MIN_DUTY = 20;              // ~8% of 255
const uint8_t MAX_DUTY = 240;             // ~94% of 255

// ---------- ROBUST ENCODER (x4 LUT) ----------
volatile long enc = 0;
volatile uint8_t prevAB = 0;
inline void encUpdate(){
  uint8_t a = digitalRead(ENC_A), b = digitalRead(ENC_B);
  uint8_t curr = (a<<1) | b;
  static const int8_t lut[16] = {0,-1,+1,0, +1,0,0,-1, -1,0,0,+1, 0,+1,-1,0};
  enc += lut[(prevAB<<2) | curr];
  prevAB = curr;
}
void IRAM_ATTR isrA(){ encUpdate(); }
void IRAM_ATTR isrB(){ encUpdate(); }

long encRead(){ noInterrupts(); long c=enc; interrupts(); return c; }

uint8_t mapForDriver(int mag0_255){
  if (mag0_255 <= 0) return 0;
  return map(constrain(mag0_255, 1, 255), 1, 255, MIN_DUTY, MAX_DUTY);
}
void applyMotor(int duty0_255){
  int mag = constrain(duty0_255, 0, 255);
  mag = min(mag, (int)SPEED_LIMIT_DUTY);
  digitalWrite(DIR1, HIGH);                 // forward
  analogWrite(PWM1, mapForDriver(mag));
}

// ---------- CONTROL ----------
enum State { WAITING, MOVING, DONE };
State state = WAITING;

const long TARGET_COUNTS = (long)(ROTATIONS * COUNTS_PER_REV + 0.5f);

int  currentDuty = 0;           // ramped duty (0..255)
long startCnt = 0;
unsigned long tLast = 0, tPrint = 0;

void setup(){
  Serial.begin(115200);
  Serial.println(F("\nOne-motor 2-rotation test | 10s delay | smooth"));
  Serial.print(F("COUNTS_PER_REV = ")); Serial.println(COUNTS_PER_REV);
  Serial.print(F("Target counts   = ")); Serial.println(TARGET_COUNTS);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  analogWrite(PWM1, 0);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  prevAB = (digitalRead(ENC_A)<<1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  // zero counts
  noInterrupts(); enc = 0; interrupts();
  applyMotor(0);

  // 10-second countdown
  for (int s = START_DELAY_MS/1000; s > 0; --s){
    Serial.print(F("Starting in ")); Serial.print(s); Serial.println(F(" s"));
    delay(1000);
  }

  startCnt = encRead();
  state = MOVING;
  tLast = millis();
  tPrint = millis();
  Serial.println(F("Moving 2 rotations..."));
}

void loop(){
  unsigned long now = millis();

  // Control tick ~10 ms
  if (now - tLast >= 10){
    tLast += 10;

    long c = encRead();
    long progressed = c - startCnt;               // signed progress (forward positive)
    long remaining  = TARGET_COUNTS - progressed; // counts left

    if (state == MOVING){
      if (remaining <= 0){
        currentDuty = 0;
        applyMotor(0);
        state = DONE;
        Serial.println(F("Done."));
      } else {
        // P-control duty from remaining counts (gentle), with min-duty near the end
        int desired = int(KP_DUTY_PER_COUNT * (float)remaining);
        desired = constrain(desired, 0, 255);
        if (desired > 0 && desired < MIN_MOVE_DUTY) desired = MIN_MOVE_DUTY;

        // Smooth ramp toward desired
        if (currentDuty < desired) currentDuty = min(currentDuty + RAMP_STEP, desired);
        else if (currentDuty > desired) currentDuty = max(currentDuty - RAMP_STEP, desired);

        applyMotor(currentDuty); // forward
      }
    } else {
      applyMotor(0);
    }
  }

  // Telemetry every 200 ms
  if (now - tPrint >= 200){
    tPrint = now;
    long c = encRead();
    long prog = c - startCnt;
    Serial.print(F("state=")); Serial.print(state==WAITING?F("WAIT"): state==MOVING?F("MOV"):F("DONE"));
    Serial.print(F("  prog=")); Serial.print(prog);
    Serial.print(F("/"));       Serial.print(TARGET_COUNTS);
    Serial.print(F("  duty=")); Serial.print(mapForDriver(min(currentDuty,(int)SPEED_LIMIT_DUTY)));
    Serial.print(F("  enc="));  Serial.println(c);
  }
}
