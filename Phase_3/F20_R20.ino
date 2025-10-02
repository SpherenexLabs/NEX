// ====== Mega + SmartElex 10D/15D + Encoders + MPU6050 — Synced + Trapezoid Ramps ======
// Serial commands: Fnn/Bnn (cm), Rnn/Lnn (deg), Z (zero yaw), S (stop now)

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"

struct MoveCmd{ char type; float value; }; // 'F','B','R','L','S','Z'

// ===================== Pin mapping =====================
constexpr bool RIGHT_ON_CH1 = false;  // toggle if channels are swapped

constexpr uint8_t PWM_R = RIGHT_ON_CH1 ? 9  : 10;
constexpr uint8_t DIR_R = RIGHT_ON_CH1 ? 7  : 8;
constexpr uint8_t PWM_L = RIGHT_ON_CH1 ? 10 : 9;
constexpr uint8_t DIR_L = RIGHT_ON_CH1 ? 8  : 7;

// Encoders (Mega external interrupts)
const uint8_t ENC_R_A=2,  ENC_R_B=3;
const uint8_t ENC_L_A=18, ENC_L_B=19;

// ===================== Direction polarity & geometry =====================
// -------- CHANGE HERE: if forward is the opposite for any wheel, flip the bools
const bool DIR_FWD_HIGH_R = true;
const bool DIR_FWD_HIGH_L = true;

// -------- CHANGE HERE: distance scaling (cm/rev and encoder CPR)
const float CM_PER_REV_R = 35.0f;   // cm travelled per revolution (right)
const float CM_PER_REV_L = 35.0f;   // cm travelled per revolution (left)
const float CPR_R        = 2400.0f; // encoder counts per revolution (right)
const float CPR_L        = 2400.0f; // encoder counts per revolution (left)

//===================== PWM limits and mapping =====================
const uint8_t MIN_DUTY = 20;
const uint8_t MAX_DUTY = 240;
const int     DUTY_ZERO_BELOW = 10;
const uint8_t SPEED_LIMIT_DUTY = 120;

// -------- CHANGE HERE: startup kick to break static friction
const uint8_t  START_KICK_R   = 30;   // right kick duty (pre-map)
const uint8_t  START_KICK_L   = 46;   // left kick duty (usually needs more)
const uint16_t START_KICK_MS  = 300;  // kick duration in ms

// -------- CHANGE HERE: straight-line cruise duty (open-loop)
const uint8_t DUTY_SYNC_CM_R  = 90;   // right cruise duty
const uint8_t DUTY_SYNC_CM_L  = 106;  // left cruise duty (slightly higher)

// -------- CHANGE HERE: turning cruise duty (spin in place)
const uint8_t DUTY_SYNC_DEG_R = 80;
const uint8_t DUTY_SYNC_DEG_L = 80;

// -------- CHANGE HERE: trapezoid profile distances & smoothing
const float   RAMP_CM         = 20.0f; // accel first 10 cm, decel last 10 cm
const int     SLEW_STEP_CM    = 4;     // max duty change per control tick (≈10 ms)

const float   RAMP_DEG        = 15.0f; // accel/decel span for turning (degrees)
const int     SLEW_STEP_DEG   = 4;     // smoothing for turns

// Optional end-hold to avoid rollback
const uint16_t HOLD_MS        = 100;
const uint8_t  HOLD_DUTY      = 16;

// Turn finish tolerance
const float   DONE_TOL_DEG    = 1.5f;
bool YAW_RIGHT_POSITIVE       = false;

// ===================== Quadrature (x4) =====================
inline int8_t lut16(uint8_t i){ static const int8_t L[16]={0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0}; return L[i]; }
volatile long encR=0, encL=0; volatile uint8_t prR=0, prL=0;
void IRAM_ATTR isrR_AB(){ uint8_t a=digitalRead(ENC_R_A),b=digitalRead(ENC_R_B),c=(a<<1)|b; encR+=lut16((prR<<2)|c); prR=c; }
void IRAM_ATTR isrL_AB(){ uint8_t a=digitalRead(ENC_L_A),b=digitalRead(ENC_L_B),c=(a<<1)|b; encL+=lut16((prL<<2)|c); prL=c; }
long readR(){ noInterrupts(); long c=encR; interrupts(); return c; }
long readL(){ noInterrupts(); long c=encL; interrupts(); return c; }

// ===================== Driver helpers =====================
uint8_t drvMap(int d){
  if (d <= 0) return 0;
  if (d < DUTY_ZERO_BELOW) return 0;
  return map(constrain(d,1,255), 1,255, MIN_DUTY, MAX_DUTY);
}
void driveR(int duty, bool fwd){
  int m=constrain(duty,0,255); if(m>SPEED_LIMIT_DUTY)m=SPEED_LIMIT_DUTY;
  digitalWrite(DIR_R, fwd ? (DIR_FWD_HIGH_R?HIGH:LOW) : (DIR_FWD_HIGH_R?LOW:HIGH));
  analogWrite(PWM_R, drvMap(m));
}
void driveL(int duty, bool fwd){
  int m=constrain(duty,0,255); if(m>SPEED_LIMIT_DUTY)m=SPEED_LIMIT_DUTY;
  digitalWrite(DIR_L, fwd ? (DIR_FWD_HIGH_L?HIGH:LOW) : (DIR_FWD_HIGH_L?LOW:HIGH));
  analogWrite(PWM_L, drvMap(m));
}
void driveStopBoth(){ analogWrite(PWM_R,0); analogWrite(PWM_L,0); }

// ===================== IMU (yaw only) =====================
MPU6050 mpu;
#define ORIENT 0
const float GYRO_SENS = 131.0f;
float yaw_deg = 0.0f, gyro_bias_z=0.0f;
uint32_t imuPrevUs=0;

static inline void remapIMU(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
                            float &bx,float &by,float &bz,float &wx,float &wy,float &wz){
#if   ORIENT==0
  bx=ax;  by=ay;  bz=az;  wx=gx;  wy=gy;  wz=gz;
#else
  bx=ax;  by=ay;  bz=az;  wx=gx;  wy=gy;  wz=gz;
#endif
}
void imuInit(){
  Wire.begin(); delay(10);
  mpu.initialize();
  mpu.setFullScaleGyroRange (MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(3);
  mpu.setRate(9);

  double sZ=0; int n=0; uint32_t t0=millis();
  while (millis()-t0<300){
    int16_t ax,ay,az,gx,gy,gz; mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sZ += (double)gz/GYRO_SENS; n++; delay(5);
  }
  if(n>0){ gyro_bias_z = sZ/n; }
  yaw_deg = 0.0f;
  imuPrevUs = micros();
}
void imuUpdate(){
  uint32_t t = micros(); if (t - imuPrevUs < 8000UL) return;
  float dt = (t - imuPrevUs) / 1.0e6f; imuPrevUs = t;

  int16_t ax,ay,az,gx,gy,gz; mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float bx,by,bz, wx,wy,wz;  remapIMU(ax,ay,az,gx,gy,gz, bx,by,bz, wx,wy,wz);
  float wz_dps = wz / GYRO_SENS - gyro_bias_z;

  if (fabsf(wz_dps) < 1.0f){
    const float beta=0.002f;
    gyro_bias_z = (1.0f - beta)*gyro_bias_z + beta*(wz/GYRO_SENS);
  }
  float sgn = YAW_RIGHT_POSITIVE ? +1.0f : -1.0f;
  yaw_deg += sgn * wz_dps * dt;
}

// ===================== Modes & state =====================
enum Phase{WAIT_CMD, MOVE_CM, TURN_DEG}; Phase phase=WAIT_CMD;

unsigned long tPr=0;

// Targets
long  TARGET_CNT_AVG = 0; // straight: average target counts
float TARGET_DEG     = 0.0f;

// Directions for current move
int   signR=+1, signL=+1;
bool  lastFwdR=true, lastFwdL=true;

// Encoder baselines
long startR=0, startL=0;

// Ramping duties (smoothed)
int dutyR_cur=0, dutyL_cur=0;

// Kick/hold state
unsigned long holdUntil=0;
unsigned long kickUntil_ms = 0;
bool inKickPhase = false;

// Command queue (optional)
static const uint8_t QN=8;
static MoveCmd q[QN]; static uint8_t qHead=0, qTail=0;
inline bool cmdQEmpty(){ return qHead==qTail; }
inline bool cmdQFull(){ return (uint8_t)((qTail+1)%QN)==qHead; }
inline void cmdQClear(){ qHead=qTail=0; }
inline void cmdQPush(const MoveCmd& c){ if(!cmdQFull()){ q[qTail]=c; qTail=(uint8_t)((qTail+1)%QN); } }
inline bool cmdQPop (MoveCmd& c){ if(cmdQEmpty()) return false; c=q[qHead]; qHead=(uint8_t)((qHead+1)%QN); return true; }

// ===================== Helpers =====================
long cmToCountsR(float cm){ float rev=fabs(cm)/CM_PER_REV_R; return (long)(rev*CPR_R+0.5f); }
long cmToCountsL(float cm){ float rev=fabs(cm)/CM_PER_REV_L; return (long)(rev*CPR_L+0.5f); }

// Trapezoid profile (straight): build on average target
struct RampCounts {
  long accel;       // counts for accel
  long cruiseStart; // == accel
  long cruiseEnd;   // target - accel
  long target;      // TARGET_CNT_AVG
} rampCnt;

void buildProfileCountsAvg(long targetCntAvg){
  long rampR = (cmToCountsR(RAMP_CM) + cmToCountsL(RAMP_CM))/2;
  long half  = targetCntAvg/2;
  long ramp  = (rampR < half) ? rampR : half;
  rampCnt.accel       = ramp;
  rampCnt.cruiseStart = ramp;
  rampCnt.cruiseEnd   = targetCntAvg - ramp;
  if (rampCnt.cruiseStart > rampCnt.cruiseEnd) rampCnt.cruiseStart = rampCnt.cruiseEnd;
  rampCnt.target      = targetCntAvg;
}

// Return duty cap based on progress inside trapezoid (straight motion)
int capFromProgressCountsAvg(long prog){
  if (rampCnt.target <= 0) return 0;
  if (prog <= 0) return max(START_KICK_R, START_KICK_L); // ensure some thrust
  if (prog <= rampCnt.accel){
    long denom = rampCnt.accel < 1 ? 1 : rampCnt.accel;
    // ramp toward the lower of the two cruise duties to avoid overspeeding the faster side
    int cruise = min(DUTY_SYNC_CM_R, DUTY_SYNC_CM_L);
    int span   = cruise - max(START_KICK_R, START_KICK_L);
    int cap    = max(START_KICK_R, START_KICK_L) + (int)((1L * span * prog) / denom);
    return constrain(cap, 0, cruise);
  }
  if (prog < rampCnt.cruiseEnd){
    return min(DUTY_SYNC_CM_R, DUTY_SYNC_CM_L);
  }
  long decSpan = (rampCnt.target - rampCnt.cruiseEnd < 1) ? 1 : (rampCnt.target - rampCnt.cruiseEnd);
  long down    = rampCnt.target - prog; if (down < 0) down = 0;
  int cap      = (int)((1L * min(DUTY_SYNC_CM_R, DUTY_SYNC_CM_L) * down) / decSpan);
  return max(0, cap);
}

// Trapezoid profile (turns): in degrees
struct RampDeg {
  float accel;
  float cruiseStart;
  float cruiseEnd;
  float target;
} rampDeg;

void buildProfileDegrees(float targetDegAbs){
  float half  = targetDegAbs * 0.5f;
  float ramp  = (RAMP_DEG < half) ? RAMP_DEG : half;
  rampDeg.accel       = ramp;
  rampDeg.cruiseStart = ramp;
  rampDeg.cruiseEnd   = targetDegAbs - ramp;
  if (rampDeg.cruiseStart > rampDeg.cruiseEnd) rampDeg.cruiseStart = rampDeg.cruiseEnd;
  rampDeg.target      = targetDegAbs;
}

int capFromProgressDeg(float prog){
  if (rampDeg.target <= 0) return 0;
  if (prog <= 0) return max(START_KICK_R, START_KICK_L);
  if (prog <= rampDeg.accel){
    float denom = rampDeg.accel < 1e-3f ? 1.0f : rampDeg.accel;
    int cruise  = min(DUTY_SYNC_DEG_R, DUTY_SYNC_DEG_L);
    float span  = (float)cruise - (float)max(START_KICK_R, START_KICK_L);
    float cap   = (float)max(START_KICK_R, START_KICK_L) + (span * prog) / denom;
    return (int)constrain((int)cap, 0, cruise);
  }
  if (prog < rampDeg.cruiseEnd) return min(DUTY_SYNC_DEG_R, DUTY_SYNC_DEG_L);
  float decSpan = (rampDeg.target - rampDeg.cruiseEnd < 1e-3f) ? 1.0f : (rampDeg.target - rampDeg.cruiseEnd);
  float down    = rampDeg.target - prog; if (down < 0) down = 0;
  int cap       = (int)((float)min(DUTY_SYNC_DEG_R, DUTY_SYNC_DEG_L) * down / decSpan);
  return max(0, cap);
}

// ===================== Starters =====================
void startMoveCM(float cm){
  signR = (cm>=0)? +1 : -1;
  signL = (cm>=0)? +1 : -1;
  lastFwdR = (signR>0); lastFwdL = (signL>0);

  long tR = labs(cmToCountsR(cm));
  long tL = labs(cmToCountsL(cm));
  TARGET_CNT_AVG = (tR + tL)/2;

  startR=readR(); startL=readL();
  dutyR_cur = dutyL_cur = 0;

  // Build trapezoid
  buildProfileCountsAvg(TARGET_CNT_AVG);

  // Kick
  inKickPhase  = true;
  kickUntil_ms = millis() + START_KICK_MS;

  phase = MOVE_CM;
}

void startTurnDeg(float deg){
  float targetAbs = fabs(deg);
  int   turnSign  = (deg>=0)? +1 : -1;

  imuUpdate();
  yaw_deg = 0.0f; // zero relative yaw for this turn

  // Right turn (deg > 0): left forward, right backward
  lastFwdL = (turnSign > 0);
  lastFwdR = !lastFwdL;

  startR=readR(); startL=readL();
  dutyR_cur = dutyL_cur = 0;

  // Build turn trapezoid
  buildProfileDegrees(targetAbs);
  TARGET_DEG = targetAbs;

  inKickPhase  = true;
  kickUntil_ms = millis() + START_KICK_MS;

  phase = TURN_DEG;
}

// ===================== Parser =====================
bool parseCommand(const String& sIn, MoveCmd &out){
  String s = sIn; s.trim(); if(!s.length()) return false;
  String up = s; up.toUpperCase(); up.replace(" ","");
  if (!up.length()) return false;
  if (up=="S"){ out.type='S'; out.value=0; return true; }
  if (up=="Z"){ out.type='Z'; out.value=0; return true; }
  if (up.endsWith("CM")) up = up.substring(0, up.length()-2);
  if (!up.length()) return false;
  char c = up[0];
  if (c=='F' || c=='B' || c=='R' || c=='L'){
    String num = up.substring(1); if(!num.length()) return false;
    float v = num.toFloat(); if (v<=0) return false;
    out.type=c; out.value=v; return true;
  }
  float v = up.toFloat();
  if (v!=0.0f){ out.type = (v>0)? 'F':'B'; out.value=fabs(v); return true; }
  return false;
}

// ===================== Setup =====================
void setup(){
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(DIR_R,OUTPUT); pinMode(PWM_R,OUTPUT); analogWrite(PWM_R,0);
  pinMode(DIR_L,OUTPUT); pinMode(PWM_L,OUTPUT); analogWrite(PWM_L,0);

  pinMode(ENC_R_A,INPUT_PULLUP); pinMode(ENC_R_B,INPUT_PULLUP);
  pinMode(ENC_L_A,INPUT_PULLUP); pinMode(ENC_L_B,INPUT_PULLUP);

  prR=(digitalRead(ENC_R_A)<<1)|digitalRead(ENC_R_B);
  prL=(digitalRead(ENC_L_A)<<1)|digitalRead(ENC_L_B);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrR_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrL_AB, CHANGE);

  noInterrupts(); encR=0; encL=0; interrupts();

  imuInit();

  Serial.println(F("Ready. Fnn/Bnn (cm), Rnn/Lnn (deg), Z (zero yaw), S (STOP)."));
}

// ===================== Loop =====================
void loop(){
  unsigned long now=millis();

  // -------- Serial handling --------
  while (Serial.available()){
    String line = Serial.readStringUntil('\n');
    MoveCmd cmd;
    if (!parseCommand(line, cmd)) continue;

    if (cmd.type=='Z'){ yaw_deg = 0.0f; Serial.println(F("[Yaw zeroed]")); continue; }
    if (cmd.type=='S'){
      cmdQClear(); driveStopBoth(); holdUntil=0; inKickPhase=false;
      dutyR_cur = dutyL_cur = 0;
      phase = WAIT_CMD; Serial.println(F("[STOP]")); continue;
    }

    Serial.print(F("CMD: ")); Serial.print(cmd.type); Serial.print(' '); Serial.println(cmd.value,1);
    if (phase==WAIT_CMD && cmdQEmpty()){
      if (cmd.type=='F') startMoveCM(+cmd.value);
      else if (cmd.type=='B') startMoveCM(-cmd.value);
      else if (cmd.type=='R') startTurnDeg(+cmd.value);
      else if (cmd.type=='L') startTurnDeg(-cmd.value);
    } else {
      cmdQPush(cmd);
    }
  }

  // IMU update
  imuUpdate();

  // -------- Control tick (10 ms) --------
  static unsigned long t10 = millis();
  if (now - t10 >= 10){
    t10 += 10;

    // ----- Startup kick (MOVE & TURN) -----
    if ((phase==MOVE_CM || phase==TURN_DEG) && inKickPhase){
      if ((long)(millis() - kickUntil_ms) < 0){
        if (phase==MOVE_CM){
          driveR(START_KICK_R, (signR > 0));
          driveL(START_KICK_L, (signL > 0));
        } else {
          driveR(START_KICK_R, lastFwdR);
          driveL(START_KICK_L, lastFwdL);
        }
        return; // skip rest during kick
      } else {
        inKickPhase = false;
      }
    }

    if (phase==MOVE_CM){
      // progress from average of absolute wheel progresses (with direction)
      long cR=readR(), cL=readL();
      long dR = (signR>0)? (cR - startR) : (startR - cR); if (dR < 0) dR = 0;
      long dL = (signL>0)? (cL - startL) : (startL - cL); if (dL < 0) dL = 0;
      long prog = (dR + dL)/2;

      // trapezoid cap (shared)
      int cap = capFromProgressCountsAvg(prog);

      // ramp each wheel smoothly toward its own target duty but never exceed cap
      int tgtR = min(DUTY_SYNC_CM_R, cap);
      int tgtL = min(DUTY_SYNC_CM_L, cap);

      dutyR_cur = (dutyR_cur < tgtR) ? min(dutyR_cur + SLEW_STEP_CM, tgtR)
                                     : max(dutyR_cur - SLEW_STEP_CM, tgtR);
      dutyL_cur = (dutyL_cur < tgtL) ? min(dutyL_cur + SLEW_STEP_CM, tgtL)
                                     : max(dutyL_cur - SLEW_STEP_CM, tgtL);

      driveR(dutyR_cur, (signR > 0));
      driveL(dutyL_cur, (signL > 0));

      // stop condition (average)
      if (prog >= rampCnt.target){
        if (HOLD_MS>0 && HOLD_DUTY>0){
          holdUntil = millis() + HOLD_MS;
          driveR(HOLD_DUTY, (signR>0));
          driveL(HOLD_DUTY, (signL>0));
        } else { driveStopBoth(); }
        phase = WAIT_CMD;
        dutyR_cur = dutyL_cur = 0;

        // next command if queued
        MoveCmd nx;
        if (cmdQPop(nx)){
          if (nx.type=='F') startMoveCM(+nx.value);
          else if (nx.type=='B') startMoveCM(-nx.value);
          else if (nx.type=='R') startTurnDeg(+nx.value);
          else if (nx.type=='L') startTurnDeg(-nx.value);
          else if (nx.type=='S'){ driveStopBoth(); phase=WAIT_CMD; cmdQClear(); }
        }
      }
    }
    else if (phase==TURN_DEG){
      float progDeg = fabs(yaw_deg);

      // trapezoid cap for turning
      int cap = capFromProgressDeg(progDeg);

      int tgtR = min(DUTY_SYNC_DEG_R, cap);
      int tgtL = min(DUTY_SYNC_DEG_L, cap);

      dutyR_cur = (dutyR_cur < tgtR) ? min(dutyR_cur + SLEW_STEP_DEG, tgtR)
                                     : max(dutyR_cur - SLEW_STEP_DEG, tgtR);
      dutyL_cur = (dutyL_cur < tgtL) ? min(dutyL_cur + SLEW_STEP_DEG, tgtL)
                                     : max(dutyL_cur - SLEW_STEP_DEG, tgtL);

      driveR(dutyR_cur, lastFwdR);
      driveL(dutyL_cur, lastFwdL);

      if (progDeg >= (rampDeg.target - DONE_TOL_DEG)){
        driveStopBoth();
        phase = WAIT_CMD;
        dutyR_cur = dutyL_cur = 0;

        MoveCmd nx;
        if (cmdQPop(nx)){
          if (nx.type=='F') startMoveCM(+nx.value);
          else if (nx.type=='B') startMoveCM(-nx.value);
          else if (nx.type=='R') startTurnDeg(+nx.value);
          else if (nx.type=='L') startTurnDeg(-nx.value);
          else if (nx.type=='S'){ driveStopBoth(); phase=WAIT_CMD; cmdQClear(); }
        }
      }
    }
    else { // WAIT_CMD
      if (holdUntil && millis() < holdUntil){
        driveR(HOLD_DUTY, lastFwdR);
        driveL(HOLD_DUTY, lastFwdL);
      } else {
        holdUntil = 0;
        driveStopBoth();
      }
    }
  }

  // -------- Telemetry (500 ms) --------
  if (millis() - tPr >= 500){
    tPr = millis();
    const char* st = (phase==WAIT_CMD)?"WAIT":(phase==MOVE_CM)?"MOVE":"TURN";
    Serial.print(F("status=")); Serial.print(st);
    Serial.print(F(" | yaw=")); Serial.print(yaw_deg,1);
    Serial.print(F(" | dutyR=")); Serial.print(dutyR_cur);
    Serial.print(F(" dutyL=")); Serial.print(dutyL_cur);
    if (phase==MOVE_CM){
      long cR=readR(), cL=readL();
      long dR = (signR>0)? (cR - startR) : (startR - cR); if (dR < 0) dR = 0;
      long dL = (signL>0)? (cL - startL) : (startL - cL); if (dL < 0) dL = 0;
      long prog = (dR + dL)/2;
      Serial.print(F(" | progCnt=")); Serial.print(prog);
      Serial.print(F("/")); Serial.print(rampCnt.target);
    } else if (phase==TURN_DEG){
      Serial.print(F(" | targetDeg=")); Serial.print(rampDeg.target,1);
    }
    Serial.println();
  }
}
