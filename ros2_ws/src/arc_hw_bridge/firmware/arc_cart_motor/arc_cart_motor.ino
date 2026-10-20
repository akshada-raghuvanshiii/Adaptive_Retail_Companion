#include <Arduino.h>

// ARC cart motor control - Arduino Uno
// SmartElex 15S motor drivers
// Serial protocol: 115200 baud

const uint8_t LEFT_PWM  = 5;
const uint8_t LEFT_DIR  = 8;
const uint8_t RIGHT_PWM = 6;
const uint8_t RIGHT_DIR = 9;

const uint8_t LEFT_ENC  = 2;
const uint8_t RIGHT_ENC = 3;
const uint8_t ESTOP_PIN = 4;

const int MAX_PWM = 60;
const unsigned long WATCHDOG_MS = 300;
const unsigned long TELEMETRY_MS = 1000;

volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;

unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;

int currentLeft = 0;
int currentRight = 0;
bool estopHealthy = false;

char inputBuffer[64];
uint8_t inputIndex = 0;

void leftEncoderISR() {
  leftPulses++;
}

void rightEncoderISR() {
  rightPulses++;
}

bool readEstopHealthy() {
  // NC loop closed = LOW = healthy
  return digitalRead(ESTOP_PIN) == LOW;
}

void setMotor(uint8_t pwmPin, uint8_t dirPin, int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);

  digitalWrite(dirPin, command >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(command));
}

void stopMotors() {
  currentLeft = 0;
  currentRight = 0;

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // Direction pins are left in a defined state.
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
}

void applyMotorCommands(int left, int right) {
  estopHealthy = readEstopHealthy();

  if (!estopHealthy) {
    stopMotors();
    Serial.println("ERR,ESTOP");
    return;
  }

  currentLeft = constrain(left, -MAX_PWM, MAX_PWM);
  currentRight = constrain(right, -MAX_PWM, MAX_PWM);

  setMotor(LEFT_PWM, LEFT_DIR, currentLeft);
  setMotor(RIGHT_PWM, RIGHT_DIR, currentRight);

  lastCommandMs = millis();
  Serial.println("OK");
}

void processCommand(char *command) {
  if (strcmp(command, "STOP") == 0) {
    stopMotors();
    lastCommandMs = millis();
    Serial.println("OK,STOP");
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println("ARC_MOTOR_READY");
    return;
  }

  int left = 0;
  int right = 0;

  if (sscanf(command, "M,%d,%d", &left, &right) == 2) {
    applyMotorCommands(left, right);
    return;
  }

  Serial.println("ERR,COMMAND");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';

      if (inputIndex > 0) {
        processCommand(inputBuffer);
      }

      inputIndex = 0;
      continue;
    }

    if (inputIndex < sizeof(inputBuffer) - 1) {
      inputBuffer[inputIndex++] = c;
    } else {
      inputIndex = 0;
      stopMotors();
      Serial.println("ERR,TOO_LONG");
    }
  }
}

void publishEncoderTelemetry() {
  unsigned long left;
  unsigned long right;

  noInterrupts();
  left = leftPulses;
  right = rightPulses;
  leftPulses = 0;
  rightPulses = 0;
  interrupts();

  Serial.print("ENC,");
  Serial.print(left);
  Serial.print(",");
  Serial.println(right);
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  pinMode(LEFT_ENC, INPUT_PULLUP);
  pinMode(RIGHT_ENC, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENC),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENC),
    rightEncoderISR,
    RISING
  );

  stopMotors();

  Serial.begin(115200);
  lastCommandMs = millis();
  lastTelemetryMs = millis();

  Serial.println("ARC_MOTOR_READY");
}

void loop() {
  readSerialCommands();

  // Hardware safety input is checked continuously.
  estopHealthy = readEstopHealthy();

  if (!estopHealthy) {
    stopMotors();
  }

  // Stop if fresh motor commands stop arriving.
  if (millis() - lastCommandMs > WATCHDOG_MS) {
    stopMotors();
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = millis();
    publishEncoderTelemetry();
  }
}
