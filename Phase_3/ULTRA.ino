// ====== Mega + SmartElex 10D/15D + Encoders + MPU6050 — Synced + Trapezoid + Ultrasonic + Speed Levels ======
// Serial: Fnn/Bnn (cm), Rnn/Lnn (deg), Z (zero yaw), S (stop now)

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// -------------------------------------------------------------------------------------------------
//                                      USER TUNABLES
// -------------------------------------------------------------------------------------------------

// -------- Board & pins --------
constexpr bool RIGHT_ON_CH1 = false;
constexpr uint8_t PWM_R = RIGHT_ON_CH1 ? 9  : 10;
constexpr uint8_t DIR_R = RIGHT_ON_CH1 ? 7  : 8;
constexpr uint8_t PWM_L = RIGHT_ON_CH1 ? 10 : 9;
constexpr uint8_t DIR_L = RIGHT_ON_CH1 ? 8  : 7;
const uint8_t ENC_R_A=2, ENC_R_B=3, ENC_L_A=18, ENC_L_B=19;

// -------- Ultrasonic pins & thresholds --------
const uint8_t US_TRIG = 6;        // HC-SR04 TRIG
const uint8_t US_ECHO = 5;        // HC-SR04 ECHO
const int     OBST_CM = 20;       // pause if <= 20 cm
const int     CLEAR_HYST_CM = 5;  // resume when > OBST_CM + 5
const uint16_t US_CHECK_MS = 60;  // measure every ~60 ms
const uint16_t US_PULSE_TIMEOUT_US = 15000;
const uint16_t RESUME_KICK_MS = 180; // short kick when resuming

// -------- Motor polarity --------
const bool DIR_FWD_HIGH_R = true;
const bool DIR_FWD_HIGH_L = true;

// -------- Geometry / encoders --------
const float CM_PER_REV_R = 35.0f;
const float CM_PER_REV_L = 35.0f;
const float CPR_R        = 2400.0f;
const float CPR_L        = 2400.0f;

// -------- PWM mapping & limits --------
const uint8_t MIN_DUTY = 20;
const uint8_t MAX_DUTY = 240;
const int     DUTY_ZERO_BELOW = 10;
const uint8_t SPEED_LIMIT_DUTY = 120;

// -------- Startup kick (launch) --------
const uint8_t  START_KICK_R   = 30;
const uint8_t  START_KICK_L   = 46;
const uint16_t START_KICK_MS  = 300;

// ===== SPEED LEVEL KNOB (straight moves) =====
// <<< CHANGE HERE: set 1..10 (4 = baseline). Each step = ±40 duty.
int  SPEED_LEVEL = 8;       // 1..10
const int SPEED_STEP = 40;  // duty change per level step
const uint8_t BASE_DUTY_R = 90;   // duties at SPEED_LEVEL = 4
const uint8_t BASE_DUTY_L = 106;

// (turn speed stays constant; change if you want)
int DUTY_SYNC_DEG_R = 80;   // turning duty right
int DUTY_SYNC_DEG_L = 80;   // turning duty left

// -------- Trapezoid ramps & smoothing --------
const float   RAMP_CM         = 10.0f; // accel first 10 cm, decel last 10 cm
const int     SLEW_STEP_CM    = 4;     // max duty change per 10ms tick
const float   RAMP_DEG        = 15.0f; // accel/decel span for turns (deg)
const int     SLEW_STEP_DEG   = 4;

// Hold a little torque at the end
const uint16_t HOLD_MS        = 100;
const uint8_t  HOLD_DUTY      = 16;

// Turn finish tolerance
const float   DONE_TOL_DEG    = 1.5f;
bool YAW_RIGHT_POSITIVE       = false;

// -------------------------------------------------------------------------------------------------
//                                     INTERNALS (no tune)
// -------------------------------------------------------------------------------------------------

struct MoveCmd{ char type; float value; };

inline int8_t lut16(uint8_t i){ static const int8_t L[16]={0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0}; return L[i]; }
volatile long encR=0, encL=0; volatile uint8_t prR=0, prL=0;
void IRAM_ATTR isrR_AB(){ uint8_t a=digitalRead(ENC_R_A),b=digitalRead(ENC_R_B),c=(a<<1)|b; encR+=lut16((prR<<2)|c); prR=c; }
void IRAM_ATTR isrL_AB(){ uint8_t a=digitalRead(ENC_L_A),b=digitalRead(ENC_L_B),c=(a<<1)|b; encL+=lut16((prL<<2)|c); prL=c; }
long readR(){ noInterrupts(); long c=encR; interrupts(); return c; }
long readL(){ noInterrupts(); long c=encL; interrupts(); return c; }

// Motor helpers
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

// IMU
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

// Modes & state
enum Phase{WAIT_CMD, MOVE_CM, TURN_DEG}; Phase phase=WAIT_CMD;
unsigned long tPr=0;

// Straight targets & profile
long  TARGET_CNT_AVG = 0;
struct RampCounts { long accel, cruiseStart, cruiseEnd, target; } rampCnt;

// Turn targets & profile
float TARGET_DEG = 0.0f;
struct RampDeg { float accel, cruiseStart, cruiseEnd, target; } rampDeg;

// Directions
int   signR=+1, signL=+1;
bool  lastFwdR=true, lastFwdL=true;

// Baselines & duties
long startR=0, startL=0;
int dutyR_cur=0, dutyL_cur=0;

// Kick/hold
unsigned long holdUntil=0;
unsigned long kickUntil_ms = 0;
bool inKickPhase = false;

// Ultrasonic pause state
bool obstaclePause = false;
unsigned long usLastMs = 0;
int  usLastCm = 9999;
unsigned long resumeKickUntil = 0;

// Command queue
static const uint8_t QN=8;
static MoveCmd q[QN]; static uint8_t qHead=0, qTail=0;
inline bool cmdQEmpty(){ return qHead==qTail; }
inline bool cmdQFull(){ return (uint8_t)((qTail+1)%QN)==qHead; }
inline void cmdQClear(){ qHead=qTail=0; }
inline void cmdQPush(const MoveCmd& c){ if(!cmdQFull()){ q[qTail]=c; qTail=(uint8_t)((qTail+1)%QN); } }
inline bool cmdQPop (MoveCmd& c){ if(cmdQEmpty()) return false; c=q[qHead]; qHead=(uint8_t)((qHead+1)%QN); return true; }

// Helpers
long cmToCountsR(float cm){ float rev=fabs(cm)/CM_PER_REV_R; return (long)(rev*CPR_R+0.5f); }
long cmToCountsL(float cm){ float rev=fabs(cm)/CM_PER_REV_L; return (long)(rev*CPR_L+0.5f); }

// Compute straight-line cruise duties from SPEED_LEVEL (1..10)
void computeStraightDuties(int &dutyR, int &dutyL){
  int delta = (SPEED_LEVEL - 4) * SPEED_STEP; // 4 is baseline
  dutyR = BASE_DUTY_R + delta;
  dutyL = BASE_DUTY_L + delta;
  dutyR = constrain(dutyR, MIN_DUTY, MAX_DUTY);
  dutyL = constrain(dutyL, MIN_DUTY, MAX_DUTY);
}

// Build trapezoid on average counts
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

// Duty cap based on progress (uses MIN of the two computed duties)
int capFromProgressCountsAvg(long prog, int dutyCruiseMin){
  if (rampCnt.target <= 0) return 0;
  if (prog <= 0) return max(START_KICK_R, START_KICK_L);
  if (prog <= rampCnt.accel){
    long denom = rampCnt.accel < 1 ? 1 : rampCnt.accel;
    int span   = dutyCruiseMin - max(START_KICK_R, START_KICK_L);
    int cap    = max(START_KICK_R, START_KICK_L) + (int)((1L * span * prog) / denom);
    return constrain(cap, 0, dutyCruiseMin);
  }
  if (prog < rampCnt.cruiseEnd){
    return dutyCruiseMin;
  }
  long decSpan = (rampCnt.target - rampCnt.cruiseEnd < 1) ? 1 : (rampCnt.target - rampCnt.cruiseEnd);
  long down    = rampCnt.target - prog; if (down < 0) down = 0;
  int cap      = (int)((1L * dutyCruiseMin * down) / decSpan);
  return max(0, cap);
}

// Turn trapezoid
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

// Starters
void startMoveCM(float cm){
  signR = (cm>=0)? +1 : -1;
  signL = (cm>=0)? +1 : -1;
  lastFwdR = (signR>0); lastFwdL = (signL>0);

  long tR = labs(cmToCountsR(cm));
  long tL = labs(cmToCountsL(cm));
  TARGET_CNT_AVG = (tR + tL)/2;

  startR=readR(); startL=readL();
  dutyR_cur = dutyL_cur = 0;

  buildProfileCountsAvg(TARGET_CNT_AVG);

  inKickPhase  = true;
  kickUntil_ms = millis() + START_KICK_MS;

  phase = MOVE_CM;
}
void startTurnDeg(float deg){
  float targetAbs = fabs(deg);
  int   turnSign  = (deg>=0)? +1 : -1;

  imuUpdate();
  yaw_deg = 0.0f; // zero relative yaw for this turn

  lastFwdL = (turnSign > 0);
  lastFwdR = !lastFwdL;

  startR=readR(); startL=readL();
  dutyR_cur = dutyL_cur = 0;

  buildProfileDegrees(targetAbs);
  TARGET_DEG = targetAbs;

  inKickPhase  = true;
  kickUntil_ms = millis() + START_KICK_MS;

  phase = TURN_DEG;
}

// Parser
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

// Ultrasonic read
int usMeasureCm(){
  digitalWrite(US_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  unsigned long dur = pulseIn(US_ECHO, HIGH, US_PULSE_TIMEOUT_US);
  if (dur == 0) return 9999; // timeout = no object
  return (int)(dur / 58.0);  // cm
}

// Setup
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

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  imuInit();

  Serial.println(F("Ready. Fnn/Bnn (cm), Rnn/Lnn (deg), Z (zero yaw), S (STOP). Speed Levels enabled."));
}

// Main loop
void loop(){
  unsigned long now=millis();

  // Serial commands
  while (Serial.available()){
    String line = Serial.readStringUntil('\n');
    MoveCmd cmd;
    if (!parseCommand(line, cmd)) continue;

    if (cmd.type=='Z'){ yaw_deg = 0.0f; Serial.println(F("[Yaw zeroed]")); continue; }
    if (cmd.type=='S'){
      cmdQClear(); driveStopBoth();
      holdUntil=0; inKickPhase=false; obstaclePause=false; resumeKickUntil=0;
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

  // IMU
  imuUpdate();

  // Ultrasonic
  if (now - usLastMs >= US_CHECK_MS){
    usLastMs = now;
    usLastCm = usMeasureCm();

    if (!obstaclePause && (phase==MOVE_CM || phase==TURN_DEG) && usLastCm <= OBST_CM){
      obstaclePause = true;
      driveStopBoth();
      Serial.println(F("[PAUSE] Obstacle ahead"));
    }
    if (obstaclePause && usLastCm > (OBST_CM + CLEAR_HYST_CM)){
      obstaclePause = false;
      resumeKickUntil = millis() + RESUME_KICK_MS;
      Serial.println(F("[RESUME] Path clear"));
    }
  }

  // Control tick (10 ms)
  static unsigned long t10 = millis();
  if (now - t10 >= 10){
    t10 += 10;

    if (obstaclePause){
      driveStopBoth();
      return;
    }

    // Compute straight cruise duties from SPEED_LEVEL (R/L)
    int DUTY_SYNC_CM_R, DUTY_SYNC_CM_L;
    computeStraightDuties(DUTY_SYNC_CM_R, DUTY_SYNC_CM_L);
    // (turn duties stay from DUTY_SYNC_DEG_R/L; change them if you want similar scaling)

    bool inResumeKick = (resumeKickUntil && (long)(millis() - resumeKickUntil) < 0);
    bool inStartKick  = ((phase==MOVE_CM || phase==TURN_DEG) && inKickPhase && (long)(millis()-kickUntil_ms) < 0);

    if (inStartKick || inResumeKick){
      if (phase==MOVE_CM){
        driveR(START_KICK_R, (signR > 0));
        driveL(START_KICK_L, (signL > 0));
      } else if (phase==TURN_DEG){
        driveR(START_KICK_R, lastFwdR);
        driveL(START_KICK_L, lastFwdL);
      }
      if (!inStartKick && inKickPhase) inKickPhase = false;
      return;
    } else {
      resumeKickUntil = 0;
      if (inKickPhase) inKickPhase = false;
    }

    if (phase==MOVE_CM){
      long cR=readR(), cL=readL();
      long dR = (signR>0)? (cR - startR) : (startR - cR); if (dR < 0) dR = 0;
      long dL = (signL>0)? (cL - startL) : (startL - cL); if (dL < 0) dL = 0;
      long prog = (dR + dL)/2;

      int cap = capFromProgressCountsAvg(prog, min(DUTY_SYNC_CM_R, DUTY_SYNC_CM_L));
      int tgtR = min(DUTY_SYNC_CM_R, cap);
      int tgtL = min(DUTY_SYNC_CM_L, cap);

      dutyR_cur = (dutyR_cur < tgtR) ? min(dutyR_cur + SLEW_STEP_CM, tgtR)
                                     : max(dutyR_cur - SLEW_STEP_CM, tgtR);
      dutyL_cur = (dutyL_cur < tgtL) ? min(dutyL_cur + SLEW_STEP_CM, tgtL)
                                     : max(dutyL_cur - SLEW_STEP_CM, tgtL);

      driveR(dutyR_cur, (signR > 0));
      driveL(dutyL_cur, (signL > 0));

      if (prog >= rampCnt.target){
        if (HOLD_MS>0 && HOLD_DUTY>0){
          holdUntil = millis() + HOLD_MS;
          driveR(HOLD_DUTY, (signR>0));
          driveL(HOLD_DUTY, (signL>0));
        } else { driveStopBoth(); }
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
    else if (phase==TURN_DEG){
      float progDeg = fabs(yaw_deg);

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

  // Telemetry
  if (millis() - tPr >= 400){
    tPr = millis();
    const char* st = (phase==WAIT_CMD)?"WAIT":(phase==MOVE_CM)?"MOVE":"TURN";
    Serial.print(F("status=")); Serial.print(st);
    Serial.print(F(" | yaw=")); Serial.print(yaw_deg,1);
    Serial.print(F(" | dutyR=")); Serial.print(dutyR_cur);
    Serial.print(F(" dutyL=")); Serial.print(dutyL_cur);
    Serial.print(F(" | US=")); Serial.print(usLastCm); Serial.print(F("cm"));
    Serial.print(F(" | SPEED_LEVEL=")); Serial.print(SPEED_LEVEL);
    if (obstaclePause) Serial.print(F(" [PAUSED]"));
    Serial.println();
  }
}
