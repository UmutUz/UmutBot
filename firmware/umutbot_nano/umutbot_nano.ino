// UmutBot - Arduino Nano motor controller
//
// Drives two DC motors through an L298N and counts the wheel encoders.
// Each wheel has its own speed controller (feedforward + PI on the encoder
// speed), so both wheels hold the commanded speed even when the battery
// sags or one motor is weaker than the other.
// Talks to the Raspberry Pi (scripts/pico_ros.py) over the USB serial port.
//
// Serial protocol (115200 baud, ASCII, one message per line):
//   Pi   -> Nano : cmd_vel:<linear m/s>:<angular rad/s>
//   Nano -> Pi   : encoder1:<ticks>,encoder2:<ticks>     (cumulative, 20 Hz)
// Bench-test commands (e.g. from the Serial Monitor):
//   pwm:<left>:<right>   raw PWM -255..255, speed controller bypassed
//   debug:1 / debug:0    extra line per cycle:
//                        dbg:<target>:<measured>:<pwm>,<target>:<measured>:<pwm>
//                        (left, right; speeds in m/s)
//
// Motor A / encoder 1 = LEFT wheel, motor B / encoder 2 = RIGHT wheel
// (encoder_odom.py uses encoder 1 as V_L and encoder 2 as V_R).
//
// Pinout:
//   L298N  ENA -> D9 (PWM)   IN1 -> D8   IN2 -> D7    (motor A)
//          ENB -> D5 (PWM)   IN3 -> D4   IN4 -> D6    (motor B)
//   Enc 1  A (yellow) -> D2 (INT0)   B (green) -> D10   VCC (blue) -> 5V
//   Enc 2  A (yellow) -> D3 (INT1)   B (green) -> D11   GND (black) -> GND

// ---------------------------------------------------------------- pins
const uint8_t PIN_ENA = 9;
const uint8_t PIN_IN1 = 8;
const uint8_t PIN_IN2 = 7;
const uint8_t PIN_ENB = 5;
const uint8_t PIN_IN3 = 4;
const uint8_t PIN_IN4 = 6;

const uint8_t PIN_ENC1_A = 2;   // hardware interrupt INT0
const uint8_t PIN_ENC1_B = 10;
const uint8_t PIN_ENC2_A = 3;   // hardware interrupt INT1
const uint8_t PIN_ENC2_B = 11;

// ---------------------------------------------------------------- robot
// Keep these in sync with navigation.launch (encoder_odometry_node params)
const float WHEEL_RADIUS = 0.035;     // m
const float WHEEL_BASE = 0.255;       // m
const float TICKS_PER_REV = 403;      // encoder ticks per wheel revolution
const float METERS_PER_TICK = 2.0 * PI * WHEEL_RADIUS / TICKS_PER_REV;

// Flip these if a wheel spins the wrong way or counts backwards.
// Goal: driving forward -> both wheels forward AND both counts increase.
const bool MOTOR_A_REVERSED = true;   // left motor is wired the other way round
const bool MOTOR_B_REVERSED = false;
const bool ENC1_REVERSED = false;
const bool ENC2_REVERSED = true;      // right encoder is mounted mirrored

// ---------------------------------------------------------------- tuning
// Measured 2026-11-05 on the bench: with both motors at PWM 255 the wheels
// only reach ~0.17 m/s (supply sags). Keep the limit a bit below that so the
// weaker wheel can still keep up. Re-measure with "pwm:255:255" + "debug:1"
// after changing the battery/supply.
const float MAX_WHEEL_SPEED = 0.15;   // m/s, wheel targets get scaled down to this
const float MIN_WHEEL_SPEED = 0.01;   // m/s, smaller targets count as "stop"

// Speed controller: PWM = feedforward(target) + KP * error + integral
const uint8_t PWM_MIN = 60;           // feedforward: lowest PWM at which the motors still turn
const float FF_GAIN = 780.0;          // feedforward: PWM per m/s on top of PWM_MIN (measured)
const float KP = 250.0;               // PWM per m/s of speed error
const float KI = 1500.0;              // PWM per (m/s * s) of accumulated error
const float I_LIMIT = 150.0;          // max PWM the integrator may add or remove

const unsigned long CONTROL_PERIOD_US = 50000;  // 20 Hz control loop + encoder report
const unsigned long CMD_TIMEOUT_MS = 500;       // stop if no command for this long

// ---------------------------------------------------------------- state
volatile long enc1Ticks = 0;
volatile long enc2Ticks = 0;

struct Wheel {
  uint8_t pinEn, pinFwd, pinRev;
  bool motorReversed;
  bool encReversed;
  volatile long *ticks;
  float target;      // commanded speed, m/s
  float measured;    // speed over the last control period, m/s
  float integral;    // integrator output, PWM
  long lastTicks;    // tick count at the last control update
  int pwm;           // last output, -255..255
};

Wheel left  = {PIN_ENA, PIN_IN1, PIN_IN2, MOTOR_A_REVERSED, ENC1_REVERSED, &enc1Ticks, 0, 0, 0, 0, 0};
Wheel right = {PIN_ENB, PIN_IN3, PIN_IN4, MOTOR_B_REVERSED, ENC2_REVERSED, &enc2Ticks, 0, 0, 0, 0, 0};

bool rawPwmMode = false;    // true while a pwm: bench-test command is active
bool debugOutput = false;

unsigned long lastCmdMs = 0;
unsigned long lastControlUs = 0;

char lineBuf[48];
uint8_t lineLen = 0;
bool lineOverflow = false;

// ---------------------------------------------------------------- encoders
// Counting rising edges of phase A only (1x decoding). Phase B gives the
// direction. If TICKS_PER_REV doesn't match, turn a wheel exactly once by
// hand and read the count in the serial monitor.
void enc1ISR() {
  if (digitalRead(PIN_ENC1_B)) enc1Ticks--; else enc1Ticks++;
}

void enc2ISR() {
  if (digitalRead(PIN_ENC2_B)) enc2Ticks--; else enc2Ticks++;
}

long readTicks(const Wheel &w) {
  noInterrupts();
  long t = *w.ticks;
  interrupts();
  return w.encReversed ? -t : t;
}

// ---------------------------------------------------------------- motors
// pwm: -255..255, positive = forward
void writeMotor(const Wheel &w, int pwm) {
  if (w.motorReversed) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);

  if (pwm == 0) {
    digitalWrite(w.pinFwd, LOW);
    digitalWrite(w.pinRev, LOW);
    analogWrite(w.pinEn, 0);
    return;
  }

  digitalWrite(w.pinFwd, pwm > 0 ? HIGH : LOW);
  digitalWrite(w.pinRev, pwm > 0 ? LOW : HIGH);
  analogWrite(w.pinEn, abs(pwm));
}

// Open-loop guess of the PWM needed for a speed; the PI part fixes the rest.
// Starts at PWM_MIN to get past the motor deadband.
float feedforward(float target) {
  float ff = min(PWM_MIN + FF_GAIN * fabs(target), 255.0);
  return target > 0 ? ff : -ff;
}

void setTarget(Wheel &w, float v) {
  if (fabs(v) < MIN_WHEEL_SPEED) v = 0;
  // Start the integrator fresh when stopping or changing direction
  if (v == 0 || (v > 0) != (w.target > 0)) w.integral = 0;
  w.target = v;
}

void updateWheel(Wheel &w, float dt) {
  long t = readTicks(w);
  w.measured = (t - w.lastTicks) * METERS_PER_TICK / dt;
  w.lastTicks = t;

  if (rawPwmMode) return;   // bench test: PWM was set directly

  if (w.target == 0) {
    w.integral = 0;
    w.pwm = 0;
    writeMotor(w, 0);
    return;
  }

  float err = w.target - w.measured;
  float out = feedforward(w.target) + KP * err + w.integral;

  // Anti-windup: only integrate while the output isn't saturated,
  // or when the error pulls it back out of saturation
  if ((out < 255 || err < 0) && (out > -255 || err > 0)) {
    w.integral = constrain(w.integral + KI * err * dt, -I_LIMIT, I_LIMIT);
  }

  // Never push against the commanded direction (no hard reversing to brake)
  if (w.target > 0) out = constrain(out, 0, 255);
  else out = constrain(out, -255, 0);

  w.pwm = (int)out;
  writeMotor(w, w.pwm);
}

void stopAll() {
  rawPwmMode = false;
  setTarget(left, 0);
  setTarget(right, 0);
  left.pwm = 0;
  right.pwm = 0;
  writeMotor(left, 0);
  writeMotor(right, 0);
}

bool moving() {
  return left.target != 0 || right.target != 0 || left.pwm != 0 || right.pwm != 0;
}

// Differential drive kinematics: body velocity -> left/right wheel target speed
void drive(float linear, float angular) {
  float vLeft = linear - angular * WHEEL_BASE / 2.0;
  float vRight = linear + angular * WHEEL_BASE / 2.0;

  // Scale both wheels down together if one exceeds the limit, so the
  // turning radius stays the same instead of clipping one side
  float maxV = max(fabs(vLeft), fabs(vRight));
  if (maxV > MAX_WHEEL_SPEED) {
    vLeft *= MAX_WHEEL_SPEED / maxV;
    vRight *= MAX_WHEEL_SPEED / maxV;
  }

  rawPwmMode = false;
  setTarget(left, vLeft);
  setTarget(right, vRight);
}

// ---------------------------------------------------------------- serial
// Parses "<a>:<b>" into two floats
bool parsePair(char *s, float &a, float &b) {
  char *sep = strchr(s, ':');
  if (sep == NULL) return false;
  *sep = '\0';
  a = atof(s);
  b = atof(sep + 1);
  return true;
}

void handleLine(char *line) {
  float a, b;

  if (strncmp(line, "cmd_vel:", 8) == 0) {
    if (!parsePair(line + 8, a, b)) return;
    drive(a, b);
    lastCmdMs = millis();
  } else if (strncmp(line, "pwm:", 4) == 0) {
    if (!parsePair(line + 4, a, b)) return;
    rawPwmMode = true;
    setTarget(left, 0);
    setTarget(right, 0);
    left.pwm = constrain((int)a, -255, 255);
    right.pwm = constrain((int)b, -255, 255);
    writeMotor(left, left.pwm);
    writeMotor(right, right.pwm);
    lastCmdMs = millis();
  } else if (strncmp(line, "debug:", 6) == 0) {
    debugOutput = (line[6] == '1');
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (lineLen > 0 && !lineOverflow) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;  // drop the rest of this garbage line
    }
  }
}

void printWheel(const Wheel &w) {
  Serial.print(w.target, 3);
  Serial.print(':');
  Serial.print(w.measured, 3);
  Serial.print(':');
  Serial.print(w.pwm);
}

void sendReport() {
  Serial.print(F("encoder1:"));
  Serial.print(left.lastTicks);
  Serial.print(F(",encoder2:"));
  Serial.println(right.lastTicks);

  if (debugOutput) {
    Serial.print(F("dbg:"));
    printWheel(left);
    Serial.print(',');
    printWheel(right);
    Serial.println();
  }
}

// ---------------------------------------------------------------- main
void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  stopAll();

  pinMode(PIN_ENC1_A, INPUT_PULLUP);
  pinMode(PIN_ENC1_B, INPUT_PULLUP);
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), enc2ISR, RISING);

  Serial.begin(115200);
  lastControlUs = micros();
}

void loop() {
  readSerial();

  // Safety: stop if the Pi stops sending (crash, unplugged, teleop released)
  if (moving() && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopAll();
  }

  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    float dt = (nowUs - lastControlUs) / 1e6;
    lastControlUs = nowUs;
    updateWheel(left, dt);
    updateWheel(right, dt);
    sendReport();
  }
}
