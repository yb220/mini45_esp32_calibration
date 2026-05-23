// Arduino Mega 3-axis motion firmware for Mini45 calibration.
// This file is a serial-command version of UMLP42_3Axis_SpdCtr.ino.
// The original joystick firmware is left unchanged.

#include <math.h>

struct AxisPins {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enPin;
  uint8_t limitPin;
};

AxisPins axisXPins = {2, 5, 8, 22};
AxisPins axisYPins = {3, 6, 9, 23};
AxisPins axisZPins = {4, 7, 10, 24};

const int JOY_X1 = A0;
const int JOY_Y1 = A1;
const int JOY_X2 = A2;
const int JOY_Y2 = A3;

const int SK1 = 30;
const int SK2 = 31;
const int SWA = 32;
const int SWB = 33;

const float LEAD_MM = 2.0f;
const long PULSE_PER_REV = 400;
const float PULSE_PER_MM = PULSE_PER_REV / LEAD_MM;

const float MAX_SPEED_STEPS = 100000.0f;
const float JOYSTICK_SPEED_SCALE = 1.8f;
const float DEADZONE_RATIO = 0.08f;
const unsigned int STEP_HIGH_US = 5;
const unsigned int DIR_SETUP_US = 100;
const unsigned long JOYSTICK_UPDATE_MS = 2;
const float HOMING_BASE_SPEED_STEPS = 16000.0f;
const float HOMING_SPEED_SCALE = 0.5f;
const float HOMING_SPEED_STEPS = HOMING_BASE_SPEED_STEPS * HOMING_SPEED_SCALE;
const float HOMING_BACKOFF_MM = 2.0f;
const unsigned long DEBOUNCE_MS = 30;

enum ControlMode {
  MODE_MANUAL,
  MODE_PC
};

struct AxisState {
  AxisPins pins;
  char name;
  float targetSpeed;
  bool dirPositive;
  bool stepHigh;
  unsigned long lastMicros;
  unsigned long stepIntervalUs;
  unsigned long dirChangeMicros;
  long positionSteps;
  long remainingSteps;
  bool fixedMove;
};

AxisState X;
AxisState Y;
AxisState Z;

ControlMode controlMode = MODE_MANUAL;
bool systemLocked = false;
bool driversEnabled = true;

float cachedCmdX = 0.0f;
float cachedCmdY = 0.0f;
float cachedCmdZ = 0.0f;
unsigned long lastJoystickUpdateMs = 0;

bool lastSK1Reading = HIGH;
bool lastSK2Reading = HIGH;
bool stableSK1State = HIGH;
bool stableSK2State = HIGH;
unsigned long lastSK1DebounceTime = 0;
unsigned long lastSK2DebounceTime = 0;

String serialBuffer;

AxisState *axisByName(const String &axisName) {
  if (axisName == "X") return &X;
  if (axisName == "Y") return &Y;
  if (axisName == "Z") return &Z;
  return NULL;
}

float joystickToNormalized(int adcValue) {
  float x = (adcValue - 512.0f) / 512.0f;
  if (fabs(x) < DEADZONE_RATIO) {
    return 0.0f;
  }
  if (x > 0.0f) {
    x = (x - DEADZONE_RATIO) / (1.0f - DEADZONE_RATIO);
  } else {
    x = (x + DEADZONE_RATIO) / (1.0f - DEADZONE_RATIO);
  }
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  return x;
}

float readAxisCommandX() {
  return -joystickToNormalized(analogRead(JOY_X1));
}

float readAxisCommandY() {
  return -joystickToNormalized(analogRead(JOY_Y1));
}

float readAxisCommandZ() {
  return -joystickToNormalized(analogRead(JOY_Y2));
}

float normalizedToSpeed(float norm) {
  float speed = norm * MAX_SPEED_STEPS * JOYSTICK_SPEED_SCALE;
  if (speed > MAX_SPEED_STEPS) speed = MAX_SPEED_STEPS;
  if (speed < -MAX_SPEED_STEPS) speed = -MAX_SPEED_STEPS;
  return speed;
}

void initAxis(AxisState &axis, AxisPins pins, char name) {
  axis.pins = pins;
  axis.name = name;
  axis.targetSpeed = 0.0f;
  axis.dirPositive = true;
  axis.stepHigh = false;
  axis.lastMicros = micros();
  axis.stepIntervalUs = 0;
  axis.dirChangeMicros = 0;
  axis.positionSteps = 0;
  axis.remainingSteps = 0;
  axis.fixedMove = false;

  pinMode(axis.pins.stepPin, OUTPUT);
  pinMode(axis.pins.dirPin, OUTPUT);
  pinMode(axis.pins.enPin, OUTPUT);
  pinMode(axis.pins.limitPin, INPUT_PULLUP);

  digitalWrite(axis.pins.stepPin, LOW);
  digitalWrite(axis.pins.dirPin, HIGH);
  digitalWrite(axis.pins.enPin, LOW);
}

void enableAxis(AxisState &axis, bool enableState) {
  digitalWrite(axis.pins.enPin, enableState ? LOW : HIGH);
}

void enableAllAxes(bool enableState) {
  driversEnabled = enableState;
  enableAxis(X, enableState);
  enableAxis(Y, enableState);
  enableAxis(Z, enableState);
}

void stopAxis(AxisState &axis) {
  axis.targetSpeed = 0.0f;
  axis.stepIntervalUs = 0;
  axis.stepHigh = false;
  axis.remainingSteps = 0;
  axis.fixedMove = false;
  digitalWrite(axis.pins.stepPin, LOW);
}

void stopAllAxes() {
  stopAxis(X);
  stopAxis(Y);
  stopAxis(Z);
}

bool anyAxisBusy() {
  return X.targetSpeed != 0.0f || Y.targetSpeed != 0.0f || Z.targetSpeed != 0.0f;
}

void applyAxisDirection(AxisState &axis, bool newDirPositive) {
  if (axis.dirPositive != newDirPositive) {
    axis.dirPositive = newDirPositive;
    digitalWrite(axis.pins.dirPin, axis.dirPositive ? HIGH : LOW);
    axis.dirChangeMicros = micros();
  }
}

bool axisDirSetupReady(const AxisState &axis) {
  return ((unsigned long)(micros() - axis.dirChangeMicros) >= DIR_SETUP_US);
}

void setAxisSpeed(AxisState &axis, float speedStepsPerSec) {
  axis.fixedMove = false;
  axis.remainingSteps = 0;
  axis.targetSpeed = speedStepsPerSec;
  if (speedStepsPerSec == 0.0f) {
    axis.stepIntervalUs = 0;
    return;
  }
  bool newDirPositive = (speedStepsPerSec > 0.0f);
  applyAxisDirection(axis, newDirPositive);
  float absSpeed = fabs(speedStepsPerSec);
  if (absSpeed > MAX_SPEED_STEPS) absSpeed = MAX_SPEED_STEPS;
  unsigned long periodUs = (unsigned long)(1000000.0f / absSpeed);
  if (periodUs <= STEP_HIGH_US + 1) periodUs = STEP_HIGH_US + 1;
  axis.stepIntervalUs = periodUs;
}

void startFixedMove(AxisState &axis, long signedSteps, float speedStepsPerSec) {
  if (signedSteps == 0 || speedStepsPerSec <= 0.0f) {
    stopAxis(axis);
    return;
  }
  axis.fixedMove = true;
  axis.remainingSteps = labs(signedSteps);
  float signedSpeed = signedSteps > 0 ? fabs(speedStepsPerSec) : -fabs(speedStepsPerSec);
  axis.targetSpeed = signedSpeed;
  bool newDirPositive = (signedSpeed > 0.0f);
  applyAxisDirection(axis, newDirPositive);
  float absSpeed = fabs(signedSpeed);
  if (absSpeed > MAX_SPEED_STEPS) absSpeed = MAX_SPEED_STEPS;
  unsigned long periodUs = (unsigned long)(1000000.0f / absSpeed);
  if (periodUs <= STEP_HIGH_US + 1) periodUs = STEP_HIGH_US + 1;
  axis.stepIntervalUs = periodUs;
}

bool axisBlockedByLimit(const AxisState &axis) {
  bool limitTriggered = (digitalRead(axis.pins.limitPin) == LOW);
  bool towardLimit = !axis.dirPositive;
  return (limitTriggered && towardLimit);
}

void updateAxis(AxisState &axis) {
  if (!driversEnabled || systemLocked) {
    stopAxis(axis);
    return;
  }

  if (axis.targetSpeed == 0.0f || axis.stepIntervalUs == 0) {
    if (axis.stepHigh) {
      digitalWrite(axis.pins.stepPin, LOW);
      axis.stepHigh = false;
    }
    return;
  }

  if (axis.fixedMove && axis.remainingSteps <= 0 && !axis.stepHigh) {
    stopAxis(axis);
    return;
  }

  if (axisBlockedByLimit(axis)) {
    stopAxis(axis);
    Serial.print("ERR LIMIT axis=");
    Serial.println(axis.name);
    return;
  }

  if (!axisDirSetupReady(axis)) {
    return;
  }

  unsigned long nowUs = micros();
  if (!axis.stepHigh) {
    if ((unsigned long)(nowUs - axis.lastMicros) >= axis.stepIntervalUs) {
      digitalWrite(axis.pins.stepPin, HIGH);
      axis.stepHigh = true;
      axis.lastMicros = nowUs;
      axis.positionSteps += axis.dirPositive ? 1 : -1;
      if (axis.fixedMove && axis.remainingSteps > 0) {
        axis.remainingSteps--;
      }
    }
  } else {
    if ((unsigned long)(nowUs - axis.lastMicros) >= STEP_HIGH_US) {
      digitalWrite(axis.pins.stepPin, LOW);
      axis.stepHigh = false;
      axis.lastMicros = nowUs;
      if (axis.fixedMove && axis.remainingSteps <= 0) {
        stopAxis(axis);
      }
    }
  }
}

void stepPulseNoLimit(AxisState &axis, long steps, bool dirPositive) {
  digitalWrite(axis.pins.dirPin, dirPositive ? HIGH : LOW);
  axis.dirPositive = dirPositive;
  delayMicroseconds(DIR_SETUP_US);
  unsigned long periodUs = (unsigned long)(1000000.0f / HOMING_SPEED_STEPS);
  if (periodUs <= STEP_HIGH_US + 1) periodUs = STEP_HIGH_US + 1;
  for (long i = 0; i < steps; i++) {
    digitalWrite(axis.pins.stepPin, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(axis.pins.stepPin, LOW);
    delayMicroseconds(periodUs - STEP_HIGH_US);
  }
}

bool checkButtonPressed(int pin, bool &lastReading, bool &stableState, unsigned long &lastDebounceTime) {
  bool reading = digitalRead(pin);
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) {
        lastReading = reading;
        return true;
      }
    }
  }
  lastReading = reading;
  return false;
}

void toggleLockState() {
  systemLocked = !systemLocked;
  if (systemLocked) {
    stopAllAxes();
    enableAllAxes(false);
    Serial.println("OK LOCKED");
  } else {
    enableAllAxes(true);
    Serial.println("OK UNLOCKED");
  }
}

bool serialStopRequestedDuringBlocking() {
  if (Serial.available() <= 0) {
    return false;
  }
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  if (command == "STOP" || command == "STOP ALL" || command == "ENABLE 0" || command == "LOCK") {
    systemLocked = true;
    stopAllAxes();
    enableAllAxes(false);
    Serial.println("OK LOCKED");
    return true;
  }
  Serial.println("ERR BUSY homing");
  return false;
}

void homeOneAxis(AxisState &axis) {
  if (systemLocked) {
    Serial.println("ERR LOCKED");
    return;
  }
  enableAxis(axis, true);
  digitalWrite(axis.pins.dirPin, LOW);
  axis.dirPositive = false;
  delayMicroseconds(DIR_SETUP_US);

  unsigned long homingPeriodUs = (unsigned long)(1000000.0f / HOMING_SPEED_STEPS);
  if (homingPeriodUs <= STEP_HIGH_US + 1) homingPeriodUs = STEP_HIGH_US + 1;

  while (digitalRead(axis.pins.limitPin) == HIGH) {
    if (checkButtonPressed(SK2, lastSK2Reading, stableSK2State, lastSK2DebounceTime) || serialStopRequestedDuringBlocking()) {
      systemLocked = true;
      stopAxis(axis);
      enableAllAxes(false);
      Serial.print("ERR HOME_ABORT axis=");
      Serial.println(axis.name);
      return;
    }
    digitalWrite(axis.pins.stepPin, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(axis.pins.stepPin, LOW);
    delayMicroseconds(homingPeriodUs - STEP_HIGH_US);
  }

  long backoffPulses = (long)(HOMING_BACKOFF_MM * PULSE_PER_MM);
  stepPulseNoLimit(axis, backoffPulses, true);
  axis.positionSteps = 0;
  stopAxis(axis);
  Serial.print("OK HOME axis=");
  Serial.println(axis.name);
}

void homeAllAxes() {
  stopAllAxes();
  homeOneAxis(X);
  if (!systemLocked) homeOneAxis(Y);
  if (!systemLocked) homeOneAxis(Z);
}

int tokenize(String command, String tokens[], int maxTokens) {
  int count = 0;
  command.trim();
  while (command.length() > 0 && count < maxTokens) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex < 0) {
      tokens[count++] = command;
      break;
    }
    String token = command.substring(0, spaceIndex);
    token.trim();
    if (token.length() > 0) {
      tokens[count++] = token;
    }
    command = command.substring(spaceIndex + 1);
    command.trim();
  }
  return count;
}

void printPos() {
  Serial.print("POS X=");
  Serial.print(X.positionSteps);
  Serial.print(" Y=");
  Serial.print(Y.positionSteps);
  Serial.print(" Z=");
  Serial.print(Z.positionSteps);
  Serial.print(" XMM=");
  Serial.print(X.positionSteps / PULSE_PER_MM, 4);
  Serial.print(" YMM=");
  Serial.print(Y.positionSteps / PULSE_PER_MM, 4);
  Serial.print(" ZMM=");
  Serial.println(Z.positionSteps / PULSE_PER_MM, 4);
}

void printLimits() {
  Serial.print("LIMIT X=");
  Serial.print(digitalRead(X.pins.limitPin) == LOW ? 1 : 0);
  Serial.print(" Y=");
  Serial.print(digitalRead(Y.pins.limitPin) == LOW ? 1 : 0);
  Serial.print(" Z=");
  Serial.println(digitalRead(Z.pins.limitPin) == LOW ? 1 : 0);
}

void printState() {
  Serial.print("STATE mode=");
  Serial.print(controlMode == MODE_PC ? "PC" : "MANUAL");
  Serial.print(" locked=");
  Serial.print(systemLocked ? 1 : 0);
  Serial.print(" enabled=");
  Serial.print(driversEnabled ? 1 : 0);
  Serial.print(" busy=");
  Serial.print(anyAxisBusy() ? 1 : 0);
  Serial.print(" sx=");
  Serial.print(X.targetSpeed, 2);
  Serial.print(" sy=");
  Serial.print(Y.targetSpeed, 2);
  Serial.print(" sz=");
  Serial.println(Z.targetSpeed, 2);
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  command.toUpperCase();

  String tokens[5];
  int n = tokenize(command, tokens, 5);
  if (n == 0) return;

  if (tokens[0] == "HELLO") {
    Serial.println("OK UMLP42_3AXIS_SERIAL version=1 pulse_per_mm=200");
    return;
  }
  if (tokens[0] == "POS?") {
    printPos();
    return;
  }
  if (tokens[0] == "STATE?") {
    printState();
    return;
  }
  if (tokens[0] == "LIMIT?") {
    printLimits();
    return;
  }
  if (tokens[0] == "MODE" && n >= 2) {
    if (tokens[1] == "MANUAL") {
      stopAllAxes();
      controlMode = MODE_MANUAL;
      Serial.println("OK MODE MANUAL");
      return;
    }
    if (tokens[1] == "PC") {
      stopAllAxes();
      controlMode = MODE_PC;
      Serial.println("OK MODE PC");
      return;
    }
  }
  if (tokens[0] == "ENABLE" && n >= 2) {
    bool enableState = tokens[1].toInt() != 0;
    if (!enableState) {
      stopAllAxes();
    }
    systemLocked = false;
    enableAllAxes(enableState);
    Serial.println(enableState ? "OK ENABLED" : "OK DISABLED");
    return;
  }
  if (tokens[0] == "STOP") {
    if (n >= 2 && tokens[1] != "ALL") {
      AxisState *axis = axisByName(tokens[1]);
      if (!axis) {
        Serial.println("ERR AXIS");
        return;
      }
      stopAxis(*axis);
      Serial.print("OK STOP axis=");
      Serial.println(axis->name);
    } else {
      stopAllAxes();
      Serial.println("OK STOP ALL");
    }
    return;
  }
  if (tokens[0] == "HOME" && n >= 2) {
    if (tokens[1] == "ALL") {
      homeAllAxes();
      return;
    }
    AxisState *axis = axisByName(tokens[1]);
    if (!axis) {
      Serial.println("ERR AXIS");
      return;
    }
    homeOneAxis(*axis);
    return;
  }

  if (controlMode != MODE_PC) {
    Serial.println("ERR MODE set MODE PC first");
    return;
  }
  if (systemLocked || !driversEnabled) {
    Serial.println("ERR LOCKED_OR_DISABLED");
    return;
  }

  if (tokens[0] == "JOG" && n >= 3) {
    AxisState *axis = axisByName(tokens[1]);
    if (!axis) {
      Serial.println("ERR AXIS");
      return;
    }
    setAxisSpeed(*axis, tokens[2].toFloat());
    Serial.print("OK JOG axis=");
    Serial.println(axis->name);
    return;
  }
  if (tokens[0] == "MOVE_STEPS" && n >= 4) {
    AxisState *axis = axisByName(tokens[1]);
    if (!axis) {
      Serial.println("ERR AXIS");
      return;
    }
    long steps = tokens[2].toInt();
    float speed = fabs(tokens[3].toFloat());
    startFixedMove(*axis, steps, speed);
    Serial.print("OK MOVE_STEPS axis=");
    Serial.println(axis->name);
    return;
  }
  if (tokens[0] == "MOVE_MM" && n >= 4) {
    AxisState *axis = axisByName(tokens[1]);
    if (!axis) {
      Serial.println("ERR AXIS");
      return;
    }
    float mm = tokens[2].toFloat();
    float speedMmS = fabs(tokens[3].toFloat());
    long steps = lround(mm * PULSE_PER_MM);
    float speedSteps = speedMmS * PULSE_PER_MM;
    startFixedMove(*axis, steps, speedSteps);
    Serial.print("OK MOVE_MM axis=");
    Serial.println(axis->name);
    return;
  }

  Serial.println("ERR COMMAND");
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (serialBuffer.length() < 96) {
      serialBuffer += c;
    }
  }
}

void serviceButtons() {
  if (checkButtonPressed(SK2, lastSK2Reading, stableSK2State, lastSK2DebounceTime)) {
    toggleLockState();
  }
  if (checkButtonPressed(SK1, lastSK1Reading, stableSK1State, lastSK1DebounceTime)) {
    homeAllAxes();
  }
}

void updateManualControl() {
  unsigned long nowMs = millis();
  if (nowMs - lastJoystickUpdateMs >= JOYSTICK_UPDATE_MS) {
    lastJoystickUpdateMs = nowMs;
    cachedCmdX = readAxisCommandX();
    cachedCmdY = readAxisCommandY();
    cachedCmdZ = readAxisCommandZ();
  }

  float cmdX = cachedCmdX;
  float cmdY = cachedCmdY;
  float cmdZ = cachedCmdZ;

  bool swaPressed = (digitalRead(SWA) == LOW);
  bool swbPressed = (digitalRead(SWB) == LOW);
  if (swaPressed && !swbPressed) {
    cmdY = 0.0f;
  } else if (!swaPressed && swbPressed) {
    cmdX = 0.0f;
  } else if (swaPressed && swbPressed) {
    cmdX = 0.0f;
    cmdY = 0.0f;
  }

  setAxisSpeed(X, normalizedToSpeed(cmdX));
  setAxisSpeed(Y, normalizedToSpeed(cmdY));
  setAxisSpeed(Z, normalizedToSpeed(cmdZ));
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  initAxis(X, axisXPins, 'X');
  initAxis(Y, axisYPins, 'Y');
  initAxis(Z, axisZPins, 'Z');

  pinMode(SK1, INPUT_PULLUP);
  pinMode(SK2, INPUT_PULLUP);
  pinMode(SWA, INPUT_PULLUP);
  pinMode(SWB, INPUT_PULLUP);

  pinMode(JOY_X1, INPUT);
  pinMode(JOY_Y1, INPUT);
  pinMode(JOY_X2, INPUT);
  pinMode(JOY_Y2, INPUT);

  lastJoystickUpdateMs = millis();
  Serial.println("OK UMLP42_3AXIS_SERIAL ready");
}

void loop() {
  serviceSerial();
  serviceButtons();

  if (systemLocked) {
    stopAllAxes();
    return;
  }

  if (controlMode == MODE_MANUAL) {
    updateManualControl();
  }

  updateAxis(X);
  updateAxis(Y);
  updateAxis(Z);
}
