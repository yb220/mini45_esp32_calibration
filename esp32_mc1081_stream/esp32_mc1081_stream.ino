#include <Wire.h>
#include <math.h>
#include "mc1081.h"

#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_FREQ_HZ 100000
#define FIRMWARE_VERSION "mini45_calibration_mc1081_v2_profiles"

MC1081_InitStructure init_struct;
CAP_AFE_DoubleEnded cap_data;

bool streaming = false;
uint32_t stream_period_ms = 20;
uint32_t last_stream_ms = 0;
uint32_t sequence_id = 0;
String active_profile = "TRAINING_FAST";
uint8_t active_cavg_count = 1;
float active_nominal_hz = 50.0f;

bool configure_profile(const String &name) {
  if (name == "STATIC_PRECISION") {
    init_struct.MC1081_CNT_CFG = 255;
    init_struct.MC1081_CAVG = CAVG_32;
    active_cavg_count = 32;
    active_nominal_hz = 2.262325f;
  } else if (name == "TRAINING_BALANCED") {
    init_struct.MC1081_CNT_CFG = 191;
    init_struct.MC1081_CAVG = CAVG_8;
    active_cavg_count = 8;
    active_nominal_hz = 11.363636f;
  } else if (name == "TRAINING_FAST") {
    init_struct.MC1081_CNT_CFG = 255;
    init_struct.MC1081_CAVG = CAVG_1;
    active_cavg_count = 1;
    active_nominal_hz = 50.0f;
  } else {
    return false;
  }
  active_profile = name;
  return true;
}

void print_profile() {
  Serial.printf("L:PROFILE,%s,cnt=%u,cavg=%u,nominal_hz=%.6f\n",
                active_profile.c_str(),
                init_struct.MC1081_CNT_CFG,
                active_cavg_count,
                active_nominal_hz);
}

void setup_mc1081() {
  init_struct.MC1081_OSC_MODE = OSC2;
  init_struct.MC1081_FINDIV = FIN_DIV_2;
  init_struct.MC1081_FREFDIV = FREF_DIV_1;
  init_struct.MC1081_OSC2_AMPLITUDE = OSC2_AMPLITUDE_2p4V;
  init_struct.MC1081_DRIVEI = DRIVE_I_8UA;
  init_struct.MC1081_OSC2_CHANNEL =
      OSC2_Channel_0 | OSC2_Channel_1 | OSC2_Channel_2 | OSC2_Channel_3 | OSC2_Channel_4 | OSC2_Ref_Channel;
  configure_profile(active_profile);
  init_struct.MC1081_SHLD_CFG = SHLD_DIS;

  if (Registers_Init(&init_struct)) {
    Serial.println("L:MC1081 initialized");
  } else {
    Serial.println("E:INIT,MC1081 initialization failed");
  }
}

bool apply_profile(const String &name) {
  bool was_streaming = streaming;
  streaming = false;
  if (!configure_profile(name)) return false;
  setup_mc1081();

  // 配置切换后的前五次转换仅用于稳定芯片内部状态，不输出到正式数据流。
  float discarded[5];
  for (int i = 0; i < 5; i++) {
    if (!read_capacitance(discarded)) return false;
  }
  streaming = was_streaming;
  last_stream_ms = 0;
  return true;
}

bool read_capacitance(float out[5]) {
  if (!MC1081_OSC2_Measure(&cap_data, &init_struct)) {
    return false;
  }
  for (int i = 0; i < 5; i++) {
    if (!isfinite(cap_data.cap_ch[i])) return false;
    out[i] = cap_data.cap_ch[i];
  }
  return true;
}

void print_data0() {
  float values[5];
  if (!read_capacitance(values)) {
    Serial.println("E:MEASURE,MC1081 measurement failed");
    return;
  }
  Serial.printf("DATA0,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                values[0], values[1], values[2], values[3], values[4]);
}

void print_cap_frame() {
  float values[5];
  if (!read_capacitance(values)) {
    Serial.println("E:MEASURE,MC1081 measurement failed");
    return;
  }
  sequence_id++;
  Serial.printf("CAP,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%u,%u,%.6f\n",
                (unsigned long)millis(),
                (unsigned long)sequence_id,
                values[0], values[1], values[2], values[3], values[4],
                active_profile.c_str(),
                init_struct.MC1081_CNT_CFG,
                active_cavg_count,
                active_nominal_hz);
}

void handle_command(String command) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;

  if (command == "INFO") {
    Serial.printf("L:%s,baud=%d,sda=%d,scl=%d\n", FIRMWARE_VERSION, SERIAL_BAUD, I2C_SDA_PIN, I2C_SCL_PIN);
    print_profile();
    return;
  }

  if (command == "GET_PROFILE") {
    print_profile();
    return;
  }

  if (command.startsWith("PROFILE,")) {
    String name = command.substring(8);
    if (!apply_profile(name)) {
      Serial.printf("E:PROFILE,failed:%s\n", name.c_str());
      return;
    }
    print_profile();
    return;
  }

  if (command == "CAPTURE") {
    print_data0();
    return;
  }

  if (command.startsWith("START")) {
    int comma = command.indexOf(',');
    int rate_hz = 50;
    if (comma >= 0) {
      rate_hz = command.substring(comma + 1).toInt();
    }
    if (rate_hz < 1) rate_hz = 1;
    if (rate_hz > 200) rate_hz = 200;
    stream_period_ms = max((uint32_t)1, (uint32_t)(1000 / rate_hz));
    streaming = true;
    last_stream_ms = 0;
    Serial.printf("L:STREAM_START,rate_hz=%d\n", rate_hz);
    return;
  }

  if (command == "STOP") {
    streaming = false;
    Serial.println("L:STREAM_STOP");
    return;
  }

  Serial.printf("E:COMMAND,unknown command:%s\n", command.c_str());
}

void read_commands() {
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handle_command(command);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(100);
  Serial.println("L:ESP32 ready");
  setup_mc1081();
}

void loop() {
  read_commands();
  if (streaming) {
    uint32_t now = millis();
    if (now - last_stream_ms >= stream_period_ms) {
      last_stream_ms = now;
      print_cap_frame();
    }
  }
  delay(1);
}
