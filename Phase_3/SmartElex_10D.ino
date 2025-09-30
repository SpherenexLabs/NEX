#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ================== PINS (Arduino Mega) ==================
const uint8_t PWM1 = 9,  DIR1 = 7;     // M1 (left)  -> S1, DIR1
const uint8_t PWM2 = 10, DIR2 = 8;     // M2 (right) -> S2, DIR2
const uint8_t ENC1_A = 2,  ENC1_B = 3;     // M1 encoder (INT0/INT1)
const uint8_t ENC2_A = 18, ENC2_B = 19;    // M2 encoder (INT5/INT4)

// ================== USER SETTINGS ==================
const bool  INVERT_M1      = false;     // set true if your left motor spins inverted
const bool  INVERT_M2      = false;     // set true if your right motor spins inverted
const float COUNTS_PER_REV = 1320.0;    // <-- replace with your measured encoder CPR
const int   SPEED_STEP     = 15;        // +/- step per key press (0..255)
const uint8_t MIN_DUTY     = 20;        // ~8% duty (overcome dead-zone)
const uint8_t MAX_DUTY     = 240;       // ~94% duty (avoid saturation)
const int   RAMP_STEP      = 5;         // counts/10ms (higher = snappier accel/decel)

// ================== TYPES ==================
enum class DriveMode : uint8_t { STOP, FWD, BACK, LEFT, RIGHT };

// ================== FORWARD DECLARATIONS ==================
uint8_t pwmForDriver(int mag0_255);
void     setMode(DriveMode m);
void     bumpSpeed(int delta);
void     handleKey(char c);
void     startStepMove(long stepsAbs, int dirSign);

// ================== ENCODER STATE/ISRs ==================
volatile long enc1 = 0, enc2 = 0;
void IRAM_ATTR on1A(){ bool a=digitalRead(ENC1_A), b=digitalRead(ENC1_B); enc1 += (a==b)? +1 : -1; }
void IRAM_ATTR on1B(){ bool a=digitalRead(ENC1_A), b=digitalRead(ENC1_B); enc1 += (a!=b)? +1 : -1; }
void IRAM_ATTR on2A(){ bool a=digitalRead(ENC2_A), b=digitalRead(ENC2_B); enc2 += (a==b)? +1 : -1; }
void IRAM_ATTR on2B(){ bool a=digitalRead(ENC2_A), b=digitalRead(ENC2_B); enc2 += (a!=b)? +1 : -1; }

// ================== DRIVE/STEP STATE ==================
DriveMode mode = DriveMode::STOP;
int  speedMag = 180;                 // 0..255 (changed by + / -)
int  target1  = 0, current1 = 0;     // -255..+255 (sign = direction)
int  target2  = 0, current2 = 0;

// Step-move controller
bool stepActive = false;
long stepTargetAbs = 0;              // target counts (absolute)
long stepE1Start = 0, stepE2Start = 0;
int  stepDir = +1;                   // +1=fwd, -1=back

// For parsing "20S"
String numBuf;

// ================== HELPERS ==================
uint8_t pwmForDriver(int mag0_255){
  if (mag0_255 <= 0) return 0;
  return map(constrain(mag0_255, 1, 255), 1, 255, MIN_DUTY, MAX_DUTY);
}

void setMode(DriveMode m){
  mode = m;
  switch (mode){
    case DriveMode::STOP:  target1=0;            target2=0;            break;
    case DriveMode::FWD:   target1=+speedMag;    target2=+speedMag;    break;
    case DriveMode::BACK:  target1=-speedMag;    target2=-speedMag;    break;
    case DriveMode::LEFT:  target1=-speedMag;    target2=+speedMag;    break; // pivot
    case DriveMode::RIGHT: target1=+speedMag;    target2=-speedMag;    break; // pivot
  }
}

void bumpSpeed(int delta){
  speedMag = constrain(speedMag + delta, 0, 255);
  if (!stepActive) setMode(mode);  // re-apply only in free-drive (step logic sets its own)
  Serial.print(F("SpeedMag=")); Serial.println(speedMag);
}

void startStepMove(long stepsAbs, int dirSign){
  if (stepsAbs <= 0) return;
  noInterrupts();
  stepE1Start = enc1;
  stepE2Start = enc2;
  interrupts();

  stepTargetAbs = stepsAbs;
  stepDir = (dirSign >= 0) ? +1 : -1;
  stepActive = true;

  // enter straight FWD/BACK for stepping
  setMode(stepDir > 0 ? DriveMode::FWD : DriveMode::BACK);

  Serial.print(F("StepMove start: target=")); Serial.print(stepTargetAbs);
  Serial.print(F(" dir=")); Serial.println(stepDir>0 ? F("FWD"):F("BACK"));
}

void handleKey(char c){
  switch(c){
    case 'F': case 'f': setMode(DriveMode::FWD);   break;
    case 'B': case 'b': setMode(DriveMode::BACK);  break;
    case 'L': case 'l': setMode(DriveMode::LEFT);  break;
    case 'R': case 'r': setMode(DriveMode::RIGHT); break;
    case 'S': case 's': // plain Stop (step terminator handled in input loop)
      if (!stepActive && numBuf.length()==0) setMode(DriveMode::STOP);
      break;
    case '+': case '=': bumpSpeed(+SPEED_STEP);    break;
    case '-': case '_': bumpSpeed(-SPEED_STEP);    break;
    default:
      if (c>='0' && c<='9'){ speedMag = map(c-'0', 0,9, 0,255); setMode(mode);
        Serial.print(F("SpeedMag=")); Serial.println(speedMag);
      }
  }
}

// ================== SETUP/LOOP ==================
void setup(){
  Serial.begin(115200);
  Serial.setTimeout(5);

  pinMode(DIR1,OUTPUT); pinMode(PWM1,OUTPUT);
  pinMode(DIR2,OUTPUT); pinMode(PWM2,OUTPUT);
  analogWrite(PWM1,0);  analogWrite(PWM2,0);

  pinMode(ENC1_A,INPUT_PULLUP); pinMode(ENC1_B,INPUT_PULLUP);
  pinMode(ENC2_A,INPUT_PULLUP); pinMode(ENC2_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), on1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), on1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), on2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), on2B, CHANGE);

  Serial.println(F("Keys: F/B/L/R/S  |  +/- speed  |  NNS = step (e.g., 20S)"));
  setMode(DriveMode::STOP);

  // Short self-test (forward then reverse both)
  digitalWrite(DIR1, (true  ^ INVERT_M1)?HIGH:LOW);
  digitalWrite(DIR2, (true  ^ INVERT_M2)?HIGH:LOW);
  analogWrite(PWM1, pwmForDriver(200)); analogWrite(PWM2, pwmForDriver(200)); delay(600);
  analogWrite(PWM1,0); analogWrite(PWM2,0); delay(250);
  digitalWrite(DIR1, (false ^ INVERT_M1)?HIGH:LOW);
  digitalWrite(DIR2, (false ^ INVERT_M2)?HIGH:LOW);
  analogWrite(PWM1, pwmForDriver(200)); analogWrite(PWM2, pwmForDriver(200)); delay(600);
  analogWrite(PWM1,0); analogWrite(PWM2,0); delay(250);
}

void loop(){
  // ---- Serial input ----
  while (Serial.available()){
    char c = Serial.read();
    if (c=='\r' || c=='\n') { /* keep numBuf on newline; Enter is ok */ continue; }

    if (c==' ') continue;                    // ignore spaces

    if (c>='0' && c<='9'){
      numBuf += c;                           // accumulate for "NNNS"
      Serial.print(c);
      continue;
    }

    if (c=='S' || c=='s'){
      if (numBuf.length()>0){
        long n = numBuf.toInt();
        // choose direction from current FWD/BACK (default FWD if turning/stop)
        int dirSign = +1;
        if (mode == DriveMode::BACK) dirSign = -1;
        startStepMove(n, dirSign);
        numBuf = "";
      } else {
        handleKey(c);                        // plain Stop
      }
      continue;
    }

    // any other non-digit key cancels number accumulation
    numBuf = "";
    handleKey(c);
  }

  // ---- Step-move controller ----
  if (stepActive){
    // progress in counts (absolute), averaged across both wheels
    noInterrupts(); long e1 = enc1, e2 = enc2; interrupts();
    long p1 = labs(e1 - stepE1Start);
    long p2 = labs(e2 - stepE2Start);
    long prog = (p1 + p2) / 2;               // average progress
    long rem  = stepTargetAbs - prog;

    if (rem <= 0){
      setMode(DriveMode::STOP);
      stepActive = false;
      Serial.println(F("StepMove done."));
    } else {
      // adaptive decel near the end
      long brakeStart = stepTargetAbs / 3;
      if (brakeStart < 100) brakeStart = 100;   // minimum braking window

      int minMag = 60;                          // minimal moving magnitude
      int desMag = speedMag;
      if (rem < brakeStart){
        desMag = map(rem, 0, brakeStart, minMag, speedMag);
        desMag = constrain(desMag, minMag, speedMag);
      }
      target1 = (stepDir>0) ? +desMag : -desMag;
      target2 = (stepDir>0) ? +desMag : -desMag;

      // progress print (every ~200 ms)
      static unsigned long ts=0; unsigned long now=millis();
      if (now - ts >= 200){
        ts = now;
        Serial.print(F("StepMove prog=")); Serial.print(prog);
        Serial.print(F("/")); Serial.print(stepTargetAbs);
        Serial.print(F(" rem=")); Serial.println(rem);
      }
    }
  }

  // ---- Smooth ramps ----
  if (current1 < target1) current1 = min(current1 + RAMP_STEP, target1);
  else if (current1 > target1) current1 = max(current1 - RAMP_STEP, target1);

  if (current2 < target2) current2 = min(current2 + RAMP_STEP, target2);
  else if (current2 > target2) current2 = max(current2 - RAMP_STEP, target2);

  // ---- Apply to driver ----
  bool fwd1 = (current1 >= 0);
  bool fwd2 = (current2 >= 0);
  int m1 = abs(current1), m2 = abs(current2);

  digitalWrite(DIR1, (fwd1 ^ INVERT_M1) ? HIGH : LOW);
  digitalWrite(DIR2, (fwd2 ^ INVERT_M2) ? HIGH : LOW);
  analogWrite(PWM1, pwmForDriver(m1));
  analogWrite(PWM2, pwmForDriver(m2));

  // ---- Telemetry (RPM + status) every 250 ms ----
  static unsigned long tPrev = millis();
  static long e1Prev=0, e2Prev=0;
  unsigned long now = millis();
  if (now - tPrev >= 250){
    float dt = (now - tPrev) / 1000.0f;  tPrev = now;

    noInterrupts(); long e1c=enc1, e2c=enc2; interrupts();
    long d1 = e1c - e1Prev; e1Prev = e1c;
    long d2 = e2c - e2Prev; e2Prev = e2c;

    float rpm1 = (COUNTS_PER_REV>0) ? ((d1 / COUNTS_PER_REV) / dt) * 60.0f : 0.0f;
    float rpm2 = (COUNTS_PER_REV>0) ? ((d2 / COUNTS_PER_REV) / dt) * 60.0f : 0.0f;

    Serial.print(F("Mode="));
    switch(mode){
      case DriveMode::STOP:  Serial.print(F("STOP"));  break;
      case DriveMode::FWD:   Serial.print(F("FWD"));   break;
      case DriveMode::BACK:  Serial.print(F("BACK"));  break;
      case DriveMode::LEFT:  Serial.print(F("LEFT"));  break;
      case DriveMode::RIGHT: Serial.print(F("RIGHT")); break;
    }
    Serial.print(F("  Mag="));  Serial.print(speedMag);
    Serial.print(F(" | M1 duty=")); Serial.print(pwmForDriver(abs(current1)));
    Serial.print(F(" rpm="));      Serial.print(rpm1,1);
    Serial.print(F(" | M2 duty=")); Serial.print(pwmForDriver(abs(current2)));
    Serial.print(F(" rpm="));       Serial.print(rpm2,1);
    Serial.print(F(" | C1="));      Serial.print(e1c);
    Serial.print(F(" C2="));        Serial.print(e2c);
    if (stepActive){
      long p1 = labs(e1c - stepE1Start);
      long p2 = labs(e2c - stepE2Start);
      long pavg = (p1 + p2) / 2;
      Serial.print(F(" | Step ")); Serial.print(pavg);
      Serial.print(F("/"));        Serial.print(stepTargetAbs);
    }
    Serial.println();
  }

  delay(10);
}
