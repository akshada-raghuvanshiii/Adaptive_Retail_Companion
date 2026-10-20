// ARC motor bridge - Arduino Uno
// Receives wheel speed targets from the PC/Pi over USB serial, drives two DC motors,
// counts Hall-sensor pulses (49E + LM393 module, magnets on the wheel) and reports them.
//
// Serial 115200, one command per line:
//   V <left_mm_s> <right_mm_s>   wheel speed targets (signed, mm/s)
//   P <left_pwm> <right_pwm>     raw PWM -255..255, no control (calibration only)
//   K <pwm_min> <ff> <ki>        pwm_min = PWM where the wheel starts turning,
//                                ff = PWM per mm/s above pwm_min, ki = integral gain (0 = open loop)
//   M <mm_per_pulse>             wheel travel per Hall pulse (circumference / magnets)
//   S                            stop
// Reply every 50 ms:
//   E <ms> <left_count> <right_count> <left_mm_s> <right_mm_s> <left_pwm> <right_pwm>
//   counts are signed totals (sign taken from the commanded direction: the sensor has one channel).
//
// SAFETY: if no V/P command arrives for WATCHDOG_MS the motors stop. Keep wheels off the ground
// for bench tests. The physical E-stop must cut motor power directly, not go through this code.

// ---------------- motor driver: choose ONE ----------------
#define DRIVER_L298N       // ENA/IN1/IN2 + ENB/IN3/IN4 (also L293D, TB6612 with STBY tied high)
// #define DRIVER_BTS7960  // two PWM inputs per motor (RPWM/LPWM), R_EN/L_EN tied to 5 V

#ifdef DRIVER_L298N
const uint8_t L_EN = 5, L_IN1 = 7, L_IN2 = 8;     // left motor: ENA (PWM), IN1, IN2
const uint8_t R_EN = 6, R_IN1 = 9, R_IN2 = 10;    // right motor: ENB (PWM), IN3, IN4
#endif
#ifdef DRIVER_BTS7960
const uint8_t L_RPWM = 5, L_LPWM = 6;             // left motor
const uint8_t R_RPWM = 9, R_LPWM = 10;            // right motor
#endif

const uint8_t HALL_L = 2, HALL_R = 3;             // LM393 digital outputs (D0) - interrupt pins on the Uno
const bool INVERT_L = false, INVERT_R = false;    // flip if a wheel turns the wrong way
const unsigned long WATCHDOG_MS = 500;
const unsigned long REPORT_MS = 50;
const unsigned long DEBOUNCE_US = 3000;           // ignore pulses closer than 3 ms (switch noise)
const unsigned long SPEED_TIMEOUT_MS = 1200;      // no pulse for this long -> wheel counted as stopped

// tunables (also settable with K / M)
float pwm_min = 60, ff = 0.6, ki = 0.0;           // 0.6 PWM per mm/s: placeholder until calibrated
float mm_per_pulse = 39.3;                         // e.g. 100 mm wheel, 8 magnets: pi*100/8

volatile long cnt_l = 0, cnt_r = 0;
volatile unsigned long last_us_l = 0, last_us_r = 0, per_us_l = 0, per_us_r = 0;
volatile int8_t dir_l = 1, dir_r = 1;

float tgt_l = 0, tgt_r = 0, integ_l = 0, integ_r = 0;
int pwm_l = 0, pwm_r = 0;
bool raw_mode = false;
unsigned long last_cmd_ms = 0, last_report_ms = 0, last_ctrl_ms = 0;
char buf[48];
uint8_t blen = 0;

void isr_l() {
  unsigned long t = micros();
  if (t - last_us_l < DEBOUNCE_US) return;
  per_us_l = t - last_us_l; last_us_l = t; cnt_l += dir_l;
}
void isr_r() {
  unsigned long t = micros();
  if (t - last_us_r < DEBOUNCE_US) return;
  per_us_r = t - last_us_r; last_us_r = t; cnt_r += dir_r;
}

void drive(int pwm, bool left) {
  pwm = constrain(pwm, -255, 255);
  if (left ? INVERT_L : INVERT_R) pwm = -pwm;
#ifdef DRIVER_L298N
  uint8_t en = left ? L_EN : R_EN, a = left ? L_IN1 : R_IN1, b = left ? L_IN2 : R_IN2;
  digitalWrite(a, pwm > 0); digitalWrite(b, pwm < 0); analogWrite(en, abs(pwm));
#endif
#ifdef DRIVER_BTS7960
  uint8_t f = left ? L_RPWM : R_RPWM, r = left ? L_LPWM : R_LPWM;
  analogWrite(f, pwm > 0 ? pwm : 0); analogWrite(r, pwm < 0 ? -pwm : 0);
#endif
}

// measured speed (mm/s, unsigned) from the time between the last two pulses
float speed_mm_s(unsigned long last_us, unsigned long per_us) {
  if (per_us == 0 || (micros() - last_us) > SPEED_TIMEOUT_MS * 1000UL) return 0;
  unsigned long since = micros() - last_us;          // if the gap is growing, the wheel is slowing
  unsigned long p = since > per_us ? since : per_us;
  return mm_per_pulse * 1e6 / p;
}

int control(float tgt, float meas, float &integ, float dt) {
  if (fabs(tgt) < 1) { integ = 0; return 0; }
  float s = tgt > 0 ? 1 : -1;
  float err = fabs(tgt) - meas;
  if (ki > 0 && meas > 0) integ = constrain(integ + ki * err * dt, -100, 100);
  float u = pwm_min + ff * fabs(tgt) + integ;
  return (int)(s * constrain(u, 0, 255));
}

void stop_all() { tgt_l = tgt_r = 0; integ_l = integ_r = 0; pwm_l = pwm_r = 0; drive(0, true); drive(0, false); }

void handle(char *line) {
  char c = line[0];
  if (c == 'V' || c == 'P') {
    char *a = strtok(line + 1, " "), *b = strtok(NULL, " ");
    if (!a || !b) return;                          // malformed: ignore (watchdog still runs)
    if (c == 'V') { raw_mode = false; tgt_l = atof(a); tgt_r = atof(b); }
    else { raw_mode = true; pwm_l = constrain(atoi(a), -255, 255); pwm_r = constrain(atoi(b), -255, 255); }
    last_cmd_ms = millis();
  }
  else if (c == 'K') { char *a = strtok(line + 1, " "), *b = strtok(NULL, " "), *d = strtok(NULL, " ");
                       if (a && b && d) { pwm_min = atof(a); ff = atof(b); ki = atof(d); integ_l = integ_r = 0; } }
  else if (c == 'M') { char *a = strtok(line + 1, " "); if (a) mm_per_pulse = atof(a); }
  else if (c == 'S') { raw_mode = false; stop_all(); }
}

void setup() {
  Serial.begin(115200);
#ifdef DRIVER_L298N
  pinMode(L_EN, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
#endif
#ifdef DRIVER_BTS7960
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
#endif
  pinMode(HALL_L, INPUT_PULLUP); pinMode(HALL_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_L), isr_l, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_R), isr_r, FALLING);
  stop_all();
  Serial.println("ARC motor bridge ready");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (blen) { buf[blen] = 0; handle(buf); blen = 0; } }
    else if (blen < sizeof(buf) - 1) buf[blen++] = ch;
  }
  unsigned long now = millis();
  if ((tgt_l != 0 || tgt_r != 0 || raw_mode) && now - last_cmd_ms > WATCHDOG_MS) { raw_mode = false; stop_all(); }

  noInterrupts();
  unsigned long lu_l = last_us_l, pu_l = per_us_l, lu_r = last_us_r, pu_r = per_us_r;
  long c_l = cnt_l, c_r = cnt_r;
  interrupts();
  float m_l = speed_mm_s(lu_l, pu_l), m_r = speed_mm_s(lu_r, pu_r);

  if (now - last_ctrl_ms >= 20) {
    float dt = (now - last_ctrl_ms) / 1000.0; last_ctrl_ms = now;
    if (!raw_mode) { pwm_l = control(tgt_l, m_l, integ_l, dt); pwm_r = control(tgt_r, m_r, integ_r, dt); }
    // direction for pulse counting = direction we are driving
    if (pwm_l != 0) dir_l = pwm_l > 0 ? 1 : -1;
    if (pwm_r != 0) dir_r = pwm_r > 0 ? 1 : -1;
    drive(pwm_l, true); drive(pwm_r, false);
  }
  if (now - last_report_ms >= REPORT_MS) {
    last_report_ms = now;
    Serial.print("E "); Serial.print(now); Serial.print(' ');
    Serial.print(c_l); Serial.print(' '); Serial.print(c_r); Serial.print(' ');
    Serial.print((dir_l > 0 ? 1 : -1) * m_l, 1); Serial.print(' ');
    Serial.print((dir_r > 0 ? 1 : -1) * m_r, 1); Serial.print(' ');
    Serial.print(pwm_l); Serial.print(' '); Serial.println(pwm_r);
  }
}
