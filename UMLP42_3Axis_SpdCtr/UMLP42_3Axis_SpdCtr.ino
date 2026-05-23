// ==========================================
// Arduino Mega 三轴平台双摇杆控制程序
// 功能：
// 1. 摇杆1的 X1/Y1 控制 X/Y 轴速度与方向
// 2. 摇杆2的 Y2 控制 Z 轴速度与方向
// 3. SWA/SWB 实现摇杆1的 XY 解耦
// 4. SK1 触发三轴回零
// 5. SK2 实现锁定切换（急停 + 使能/失能）
// 6. 驱动器采用低电平使能
// 7. 脉冲当量按 400 步/转配置
// 8. 引入正常控制与回零速度修正系数
// 9. 默认关闭串口打印
// ==========================================

#include <math.h>

// -------------------- 调试输出开关 --------------------
const bool ENABLE_SERIAL_DEBUG = false;

// -------------------- 三轴引脚定义 --------------------
struct AxisPins {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enPin;
  uint8_t limitPin;
};

AxisPins axisX = {2, 5, 8, 22};
AxisPins axisY = {3, 6, 9, 23};
AxisPins axisZ = {4, 7, 10, 24};

// -------------------- 摇杆与按键引脚 --------------------
const int JOY_X1 = A0;
const int JOY_Y1 = A1;
const int JOY_X2 = A2;   // 当前版本中空置
const int JOY_Y2 = A3;

const int SK1 = 30;
const int SK2 = 31;
const int SWA = 32;
const int SWB = 33;

// -------------------- 机械参数 --------------------
const float lead_mm = 2.0f;               // 丝杆导程 mm/rev
const long pulse_per_rev = 400;           // 当前驱动器拨码：400 pulses/rev
const float pulse_per_mm = pulse_per_rev / lead_mm;

// -------------------- 速度参数 --------------------
// 正常控制理论上限
const float MAX_SPEED_STEPS = 100000.0f;

// 摇杆速度修正系数
// 数值大于 1 时，中间段速度响应增强
const float JOYSTICK_SPEED_SCALE = 1.8f;

// 摇杆死区比例
const float DEADZONE_RATIO = 0.08f;

// STEP 脉冲高电平保持时间
const unsigned int STEP_HIGH_US = 5;

// 方向建立时间
const unsigned int DIR_SETUP_US = 100;

// 摇杆采样更新周期
const unsigned long JOYSTICK_UPDATE_MS = 2;

// 回零速度基准与缩放系数
const float HOMING_BASE_SPEED_STEPS = 16000.0f;
const float HOMING_SPEED_SCALE = 0.5f;
const float HOMING_SPEED_STEPS = HOMING_BASE_SPEED_STEPS * HOMING_SPEED_SCALE;

// 回零退回距离
const float HOMING_BACKOFF_MM = 2.0f;

// 按键消抖时间
const unsigned long DEBOUNCE_MS = 30;

// 串口打印周期
const unsigned long SERIAL_PRINT_MS = 200;

// -------------------- 轴运行状态 --------------------
struct AxisState {
  AxisPins pins;
  float targetSpeed;                  // 目标速度，单位 steps/s，带符号
  bool dirPositive;                   // 当前方向状态
  bool stepHigh;                      // 当前 STEP 状态
  unsigned long lastMicros;           // 最近一次 STEP 翻转时间
  unsigned long stepIntervalUs;       // 步进周期
  unsigned long dirChangeMicros;      // 最近一次方向变化时间
};

AxisState X, Y, Z;

// -------------------- 系统状态 --------------------
bool systemLocked = false;

// -------------------- 摇杆缓存 --------------------
float cachedCmdX = 0.0f;
float cachedCmdY = 0.0f;
float cachedCmdZ = 0.0f;
unsigned long lastJoystickUpdateMs = 0;

// -------------------- 按键边沿检测状态 --------------------
bool lastSK1Reading = HIGH;
bool lastSK2Reading = HIGH;
bool stableSK1State = HIGH;
bool stableSK2State = HIGH;
unsigned long lastSK1DebounceTime = 0;
unsigned long lastSK2DebounceTime = 0;

// -------------------- 工具函数 --------------------
void debugPrintln(const char *msg)
{
  if (ENABLE_SERIAL_DEBUG) {
    Serial.println(msg);
  }
}

void debugPrint(const char *msg)
{
  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(msg);
  }
}

void debugPrintValue(long value)
{
  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(value);
  }
}

void debugPrintFloat(float value)
{
  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(value);
  }
}

float joystickToNormalized(int adcValue)
{
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

// 摇杆模块中，右/下端点输出低电压。
// 为使右推和下推动作对应正速度，相关通道取反。
float readAxisCommandX()
{
  return -joystickToNormalized(analogRead(JOY_X1));
}

float readAxisCommandY()
{
  return -joystickToNormalized(analogRead(JOY_Y1));
}

float readAxisCommandZ()
{
  return -joystickToNormalized(analogRead(JOY_Y2));
}

float normalizedToSpeed(float norm)
{
  float speed = norm * MAX_SPEED_STEPS * JOYSTICK_SPEED_SCALE;

  if (speed > MAX_SPEED_STEPS) speed = MAX_SPEED_STEPS;
  if (speed < -MAX_SPEED_STEPS) speed = -MAX_SPEED_STEPS;

  return speed;
}

void initAxis(AxisState &axis, AxisPins pins)
{
  axis.pins = pins;
  axis.targetSpeed = 0.0f;
  axis.dirPositive = true;
  axis.stepHigh = false;
  axis.lastMicros = micros();
  axis.stepIntervalUs = 0;
  axis.dirChangeMicros = 0;

  pinMode(axis.pins.stepPin, OUTPUT);
  pinMode(axis.pins.dirPin, OUTPUT);
  pinMode(axis.pins.enPin, OUTPUT);
  pinMode(axis.pins.limitPin, INPUT_PULLUP);

  digitalWrite(axis.pins.stepPin, LOW);
  digitalWrite(axis.pins.dirPin, HIGH);

  // 低电平使能，高电平失能
  digitalWrite(axis.pins.enPin, LOW);
}

void enableAxis(AxisState &axis, bool enableState)
{
  digitalWrite(axis.pins.enPin, enableState ? LOW : HIGH);
}

void enableAllAxes(bool enableState)
{
  enableAxis(X, enableState);
  enableAxis(Y, enableState);
  enableAxis(Z, enableState);
}

void stopAxis(AxisState &axis)
{
  axis.targetSpeed = 0.0f;
  axis.stepIntervalUs = 0;
  axis.stepHigh = false;
  digitalWrite(axis.pins.stepPin, LOW);
}

void stopAllAxes()
{
  stopAxis(X);
  stopAxis(Y);
  stopAxis(Z);
}

void applyAxisDirection(AxisState &axis, bool newDirPositive)
{
  if (axis.dirPositive != newDirPositive) {
    axis.dirPositive = newDirPositive;
    digitalWrite(axis.pins.dirPin, axis.dirPositive ? HIGH : LOW);
    axis.dirChangeMicros = micros();
  }
}

bool axisDirSetupReady(const AxisState &axis)
{
  return ((unsigned long)(micros() - axis.dirChangeMicros) >= DIR_SETUP_US);
}

void setAxisSpeed(AxisState &axis, float speedStepsPerSec)
{
  axis.targetSpeed = speedStepsPerSec;

  if (speedStepsPerSec == 0.0f) {
    axis.stepIntervalUs = 0;
    return;
  }

  bool newDirPositive = (speedStepsPerSec > 0.0f);
  applyAxisDirection(axis, newDirPositive);

  float absSpeed = fabs(speedStepsPerSec);
  if (absSpeed > MAX_SPEED_STEPS) {
    absSpeed = MAX_SPEED_STEPS;
  }

  unsigned long periodUs = (unsigned long)(1000000.0f / absSpeed);

  if (periodUs <= STEP_HIGH_US + 1) {
    periodUs = STEP_HIGH_US + 1;
  }

  axis.stepIntervalUs = periodUs;
}

// 默认按“负方向靠近限位”处理。
// 若某一轴实际安装方向相反，可单独翻转该判断逻辑。
bool axisBlockedByLimit(const AxisState &axis)
{
  bool limitTriggered = (digitalRead(axis.pins.limitPin) == LOW);
  bool towardLimit = !axis.dirPositive;
  return (limitTriggered && towardLimit);
}

void updateAxis(AxisState &axis)
{
  if (axis.targetSpeed == 0.0f || axis.stepIntervalUs == 0) {
    if (axis.stepHigh) {
      digitalWrite(axis.pins.stepPin, LOW);
      axis.stepHigh = false;
    }
    return;
  }

  if (axisBlockedByLimit(axis)) {
    if (axis.stepHigh) {
      digitalWrite(axis.pins.stepPin, LOW);
      axis.stepHigh = false;
    }
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
    }
  } else {
    if ((unsigned long)(nowUs - axis.lastMicros) >= STEP_HIGH_US) {
      digitalWrite(axis.pins.stepPin, LOW);
      axis.stepHigh = false;
      axis.lastMicros = nowUs;
    }
  }
}

// 不带限位检测的固定步数输出，仅用于回零后的反向退出限位
void stepPulseNoLimit(AxisState &axis, long steps, bool dirPositive)
{
  digitalWrite(axis.pins.dirPin, dirPositive ? HIGH : LOW);
  axis.dirPositive = dirPositive;
  delayMicroseconds(DIR_SETUP_US);

  unsigned long periodUs = (unsigned long)(1000000.0f / HOMING_SPEED_STEPS);
  if (periodUs <= STEP_HIGH_US + 1) {
    periodUs = STEP_HIGH_US + 1;
  }

  for (long i = 0; i < steps; i++) {
    digitalWrite(axis.pins.stepPin, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(axis.pins.stepPin, LOW);
    delayMicroseconds(periodUs - STEP_HIGH_US);
  }
}

// 单轴回零流程：
// 1. 朝负方向运动，直至触发限位
// 2. 反向退出固定距离，避免机械持续压限位
void homeOneAxis(AxisState &axis, const char *axisName)
{
  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(axisName);
    Serial.println(" homing start...");
  }

  enableAxis(axis, true);

  digitalWrite(axis.pins.dirPin, LOW);
  axis.dirPositive = false;
  delayMicroseconds(DIR_SETUP_US);

  unsigned long homingPeriodUs = (unsigned long)(1000000.0f / HOMING_SPEED_STEPS);
  if (homingPeriodUs <= STEP_HIGH_US + 1) {
    homingPeriodUs = STEP_HIGH_US + 1;
  }

  while (digitalRead(axis.pins.limitPin) == HIGH) {
    if (systemLocked) {
      if (ENABLE_SERIAL_DEBUG) {
        Serial.print(axisName);
        Serial.println(" homing aborted by lock.");
      }
      stopAxis(axis);
      return;
    }

    digitalWrite(axis.pins.stepPin, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(axis.pins.stepPin, LOW);
    delayMicroseconds(homingPeriodUs - STEP_HIGH_US);
  }

  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(axisName);
    Serial.println(" limit found.");
  }

  long backoffPulses = (long)(HOMING_BACKOFF_MM * pulse_per_mm);
  stepPulseNoLimit(axis, backoffPulses, true);

  stopAxis(axis);

  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(axisName);
    Serial.println(" homing finished.");
  }
}

void homeAllAxes()
{
  if (systemLocked) {
    debugPrintln("Homing ignored: system locked.");
    return;
  }

  stopAllAxes();

  debugPrintln("=== All-axis homing start ===");
  homeOneAxis(X, "X");
  homeOneAxis(Y, "Y");
  homeOneAxis(Z, "Z");
  debugPrintln("=== All-axis homing finished ===");
}

void toggleLockState()
{
  systemLocked = !systemLocked;

  if (systemLocked) {
    stopAllAxes();
    enableAllAxes(false);
    debugPrintln("System locked: emergency stop + drivers disabled.");
  } else {
    enableAllAxes(true);
    debugPrintln("System unlocked: drivers enabled.");
  }
}

// 返回 true 表示检测到一次按下事件
bool checkButtonPressed(int pin,
                        bool &lastReading,
                        bool &stableState,
                        unsigned long &lastDebounceTime)
{
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

void printDebugState()
{
  if (!ENABLE_SERIAL_DEBUG) {
    return;
  }

  static unsigned long lastPrintMs = 0;
  unsigned long nowMs = millis();

  if (nowMs - lastPrintMs >= SERIAL_PRINT_MS) {
    lastPrintMs = nowMs;

    Serial.print("ADC X1=");
    Serial.print(analogRead(JOY_X1));
    Serial.print(" Y1=");
    Serial.print(analogRead(JOY_Y1));
    Serial.print(" Y2=");
    Serial.print(analogRead(JOY_Y2));

    Serial.print(" | SpeedX=");
    Serial.print(X.targetSpeed, 1);
    Serial.print(" SpeedY=");
    Serial.print(Y.targetSpeed, 1);
    Serial.print(" SpeedZ=");
    Serial.print(Z.targetSpeed, 1);
    Serial.print(" | Locked=");
    Serial.println(systemLocked ? "YES" : "NO");

    Serial.print("SK1=");
    Serial.print(digitalRead(SK1));
    Serial.print("  SK2=");
    Serial.print(digitalRead(SK2));
    Serial.print("  SWA=");
    Serial.print(digitalRead(SWA));
    Serial.print("  SWB=");
    Serial.println(digitalRead(SWB));
  }
}

void setup()
{
  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(115200);
    Serial.println("====================================");
    Serial.println("3-Axis Joystick Control Ready");
    Serial.println("Driver enable polarity: LOW = ENABLE");
    Serial.println("Pulse per rev = 400");
    Serial.println("Pulse per mm  = 200");
    Serial.println("Serial debug  = ON");
    Serial.println("====================================");
  }

  initAxis(X, axisX);
  initAxis(Y, axisY);
  initAxis(Z, axisZ);

  pinMode(SK1, INPUT_PULLUP);
  pinMode(SK2, INPUT_PULLUP);
  pinMode(SWA, INPUT_PULLUP);
  pinMode(SWB, INPUT_PULLUP);

  pinMode(JOY_X1, INPUT);
  pinMode(JOY_Y1, INPUT);
  pinMode(JOY_X2, INPUT);
  pinMode(JOY_Y2, INPUT);

  lastJoystickUpdateMs = millis();
}

void loop()
{
  // ---------- 按键事件处理 ----------
  if (checkButtonPressed(SK2, lastSK2Reading, stableSK2State, lastSK2DebounceTime)) {
    toggleLockState();
  }

  if (checkButtonPressed(SK1, lastSK1Reading, stableSK1State, lastSK1DebounceTime)) {
    homeAllAxes();
  }

  // ---------- 锁定状态处理 ----------
  if (systemLocked) {
    stopAllAxes();
    return;
  }

  // ---------- 摇杆缓存更新 ----------
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

  // ---------- SWA / SWB 解耦逻辑 ----------
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

  // ---------- 速度设定 ----------
  setAxisSpeed(X, normalizedToSpeed(cmdX));
  setAxisSpeed(Y, normalizedToSpeed(cmdY));
  setAxisSpeed(Z, normalizedToSpeed(cmdZ));

  // ---------- 三轴脉冲更新 ----------
  updateAxis(X);
  updateAxis(Y);
  updateAxis(Z);

  // ---------- 调试输出 ----------
  printDebugState();
}