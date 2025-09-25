// ---- Pins ----
const uint8_t ENC_A = 2;   // INT0
const uint8_t ENC_B = 3;   // INT1

volatile long encCount = 0;

void onA() {                            // ISR on rising edge of A
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  if (a && !b) encCount++;              // forward
  else if (a && b) encCount--;          // reverse
}

long lastCount = 0;
unsigned long lastMs = 0;
const long CPR_OUT = 600;               // set your measured counts/rev

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onA, RISING);
  lastMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastMs >= 500) {
    noInterrupts();
    long c = encCount;
    interrupts();

    long delta = c - lastCount;
    float dt = (now - lastMs) / 1000.0f;
    float rpm = (CPR_OUT > 0) ? (delta * 60.0f / (CPR_OUT * dt)) : 0;

    Serial.print("Count="); Serial.print(c);
    Serial.print("  dC=");  Serial.print(delta);
    Serial.print("  RPM="); Serial.println(rpm, 2);

    lastCount = c;
    lastMs = now;
  }
}
