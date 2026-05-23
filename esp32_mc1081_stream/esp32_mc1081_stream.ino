#include <Wire.h>
#include <math.h>
#include "mc1081.h"

#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_FREQ_HZ 100000
#define FIRMWARE_VERSION "mini45_calibration_mc1081_v1"

MC1081_InitStructure init_struct;
CAP_AFE_DoubleEnded cap_data;

bool streaming = false;
uint32_t stream_period_ms = 20;
uint32_t last_stream_ms = 0;
uint32_t sequence_id = 0;

void setup_mc1081() {
  init_struct.MC1081_OSC_MODE = OSC2;
  init_struct.MC1081_FINDIV = FIN_DIV_2;
  init_struct.MC1081_FREFDIV = FREF_DIV_1;
  init_struct.MC1081_OSC2_AMPLITUDE = OSC2_AMPLITUDE_2p4V;
  init_struct.MC1081_DRIVEI = DRIVE_I_8UA;
  init_struct.MC1081_OSC2_CHANNEL =
      OSC2_Channel_0 | OSC2_Channel_1 | OSC2_Channel_2 | OSC2_Channel_3 | OSC2_Channel_4 | OSC2_Ref_Channel;
  init_struct.MC1081_CNT_CFG = 0x7F;
  init_struct.MC1081_SHLD_CFG = SHLD_DIS;

  if (Registers_Init(&init_struct)) {
    Serial.println("L:MC1081 initialized");
  } else {
    Serial.println("E:INIT,MC1081 initialization failed");
  }
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
  Serial.printf("CAP,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                (unsigned long)millis(),
                (unsigned long)sequence_id,
                values[0], values[1], values[2], values[3], values[4]);
}

void handle_command(String command) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;

  if (command == "INFO") {
    Serial.printf("L:%s,baud=%d,sda=%d,scl=%d\n", FIRMWARE_VERSION, SERIAL_BAUD, I2C_SDA_PIN, I2C_SCL_PIN);
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
