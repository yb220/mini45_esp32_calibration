/*
  MC1081 独立诊断固件

  用途：
  - 不依赖现有标定上位机和采集固件，单独检查 MC1081 采集链路稳定性。
  - 同时输出原始计数、参考通道、overflow/status 和换算后的 pF。
  - 用固定电容、短接/开路、交换通道等方式定位漂移来自芯片、通道、线缆还是传感器。

  串口命令：
  - INFO              打印固件信息、CSV 表头和当前配置
  - CAPTURE           单次采样并输出一行 DIAG
  - START,<rate_hz>   开始连续输出，建议先用 START,10
  - STOP              停止连续输出
  - CAVG,1 或 CAVG,4  设置 MC1081 内部平均次数
  - DISCARD,<n>       每次正式读数前丢弃 n 次转换结果，默认 1
  - I2C,<hz>          设置 I2C 频率，例如 I2C,100000 或 I2C,400000
  - RESET             重新初始化 MC1081
  - HELP              打印命令说明
*/

#include <Wire.h>
#include <math.h>

#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define DEFAULT_I2C_FREQ_HZ 100000UL
#define FIRMWARE_VERSION "mc1081_diagnostic_v1"

#define MC1081_ADDR 0x70
#define CREF_PF 20.0f
#define IN_CLK_MHZ 19.2f

#define OSC2_CHANNEL_0 (1 << 0)
#define OSC2_CHANNEL_1 (1 << 1)
#define OSC2_CHANNEL_2 (1 << 2)
#define OSC2_CHANNEL_3 (1 << 3)
#define OSC2_CHANNEL_4 (1 << 4)
#define OSC2_REF_CHANNEL (1 << 5)

#define D0_MSB 0x02
#define DREF_MSB 0x16
#define OSC2_OF 0x1A
#define MC_STATUS 0x1B
#define C_CMD 0x1D
#define CNT_CFG 0x1E
#define DIV_CFG 0x1F
#define OSC2_DCHS 0x24
#define OSC2_CFG 0x25
#define SHLD_CFG 0x26
#define MC1081_RESET 0x69

#define MC1081_RESET_CMD 0x7A
#define OF_CLEAR 0x10
#define OSC1_LDO_H (0x01U << 7)
#define SHLD_DIS 0x00
#define OSC2_EN (0x01U << 7)
#define SLEEP_EN (0x01U << 6)
#define CAVG_1 (0x00U << 4)
#define CAVG_4 (0x01U << 4)
#define OS_SD_ONE 0x03

struct DiagConfig {
  uint8_t cnt_cfg = 0x7F;
  uint8_t fin_div_code = 0x01;   // FIN_DIV_2
  uint8_t fref_div_code = 0x00;  // FREF_DIV_1
  uint8_t osc2_amplitude = 0x05; // 2.4 V
  uint8_t drive_i = 0x01;        // 8 uA
  uint8_t cavg = CAVG_4;
  uint8_t discard_count = 1;
  uint32_t i2c_hz = DEFAULT_I2C_FREQ_HZ;
};

struct DiagFrame {
  bool valid = false;
  uint8_t error = 0;
  uint8_t status = 0;
  uint8_t overflow = 0;
  uint16_t dref = 0;
  uint16_t d[5] = {0, 0, 0, 0, 0};
  float c[5] = {NAN, NAN, NAN, NAN, NAN};
  uint32_t dt_us = 0;
};

DiagConfig cfg;
bool streaming = false;
uint32_t stream_period_ms = 100;
uint32_t last_stream_ms = 0;
uint32_t sequence_id = 0;

bool write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MC1081_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool read_reg(uint8_t reg, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(MC1081_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(MC1081_ADDR, size);
  if (received != size) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < size; i++) {
    data[i] = Wire.read();
  }
  return true;
}

bool read_u16(uint8_t reg, uint16_t &value) {
  uint8_t data[2] = {0, 0};
  if (!read_reg(reg, data, 2)) return false;
  value = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

bool init_mc1081() {
  bool ok = true;
  uint8_t div_value = (cfg.fin_div_code << 4) | cfg.fref_div_code;
  uint8_t osc2_cfg = cfg.drive_i | (cfg.osc2_amplitude << 4) | OSC1_LDO_H;
  uint8_t channels = OSC2_CHANNEL_0 | OSC2_CHANNEL_1 | OSC2_CHANNEL_2 |
                     OSC2_CHANNEL_3 | OSC2_CHANNEL_4 | OSC2_REF_CHANNEL;

  ok &= write_reg(MC1081_RESET, MC1081_RESET_CMD);
  delay(10);
  ok &= write_reg(DIV_CFG, div_value);
  ok &= write_reg(CNT_CFG, cfg.cnt_cfg);
  ok &= write_reg(SHLD_CFG, SHLD_DIS);
  ok &= write_reg(OSC2_DCHS, channels);
  ok &= write_reg(OSC2_CFG, osc2_cfg);
  return ok;
}

float cap_pf_from_counts(uint16_t d_ch, uint16_t d_ref) {
  if (d_ch == 0 || d_ref == 0) return NAN;
  return (float)d_ch / (float)d_ref * CREF_PF;
}

DiagFrame measure_once() {
  DiagFrame frame;
  uint32_t t0 = micros();

  if (!write_reg(MC_STATUS, OF_CLEAR)) {
    frame.error = 1;
    return frame;
  }
  if (!write_reg(C_CMD, OSC2_EN | SLEEP_EN | cfg.cavg | OS_SD_ONE)) {
    frame.error = 2;
    return frame;
  }

  uint8_t status = 0x01;
  int timeout = 2000;
  do {
    delayMicroseconds(20);
    if (!read_reg(MC_STATUS, &status, 1)) {
      frame.error = 3;
      return frame;
    }
    timeout--;
  } while ((status & 0x01) && timeout > 0);

  frame.status = status;
  if (timeout <= 0) {
    frame.error = 4;
    return frame;
  }

  if (!read_reg(OSC2_OF, &frame.overflow, 1)) {
    frame.error = 5;
    return frame;
  }
  if (!read_u16(DREF_MSB, frame.dref)) {
    frame.error = 6;
    return frame;
  }

  for (int i = 0; i < 5; i++) {
    if (!read_u16((uint8_t)(D0_MSB + 2 * i), frame.d[i])) {
      frame.error = 7;
      return frame;
    }
    frame.c[i] = cap_pf_from_counts(frame.d[i], frame.dref);
  }

  frame.dt_us = micros() - t0;
  frame.valid = true;
  if (frame.dref == 0 || frame.dref >= 65535) {
    frame.valid = false;
    frame.error = 8;
  }
  for (int i = 0; i < 5; i++) {
    if (frame.d[i] == 0 || frame.d[i] >= 65535 || !isfinite(frame.c[i])) {
      frame.valid = false;
      frame.error = 9;
    }
  }
  if (frame.overflow != 0) {
    frame.valid = false;
    frame.error = 10;
  }
  return frame;
}

DiagFrame measure_with_discard() {
  DiagFrame frame;
  for (uint8_t i = 0; i < cfg.discard_count; i++) {
    frame = measure_once();
    delay(2);
  }
  return measure_once();
}

void print_header() {
  Serial.println("L:HEADER,DIAG,esp_ms,seq,valid,error,status,overflow,dref,d0,d1,d2,d3,d4,c0,c1,c2,c3,c4,dt_us");
}

void print_config() {
  Serial.printf(
      "L:CONFIG,version=%s,baud=%d,sda=%d,scl=%d,i2c_hz=%lu,cavg=%s,discard=%u,cnt_cfg=%u,drive_i=%u,amp=%u\n",
      FIRMWARE_VERSION,
      SERIAL_BAUD,
      I2C_SDA_PIN,
      I2C_SCL_PIN,
      (unsigned long)cfg.i2c_hz,
      cfg.cavg == CAVG_4 ? "4" : "1",
      cfg.discard_count,
      cfg.cnt_cfg,
      cfg.drive_i,
      cfg.osc2_amplitude);
}

void print_diag_frame() {
  DiagFrame frame = measure_with_discard();
  sequence_id++;
  Serial.printf(
      "DIAG,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%lu\n",
      (unsigned long)millis(),
      (unsigned long)sequence_id,
      frame.valid ? 1 : 0,
      frame.error,
      frame.status,
      frame.overflow,
      frame.dref,
      frame.d[0],
      frame.d[1],
      frame.d[2],
      frame.d[3],
      frame.d[4],
      frame.c[0],
      frame.c[1],
      frame.c[2],
      frame.c[3],
      frame.c[4],
      (unsigned long)frame.dt_us);
}

void print_help() {
  Serial.println("L:HELP,INFO|CAPTURE|START,<rate_hz>|STOP|CAVG,1|CAVG,4|DISCARD,<n>|I2C,<hz>|RESET|HELP");
}

void handle_command(String command) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;

  if (command == "INFO") {
    print_config();
    print_header();
    return;
  }
  if (command == "HELP") {
    print_help();
    return;
  }
  if (command == "CAPTURE") {
    print_diag_frame();
    return;
  }
  if (command == "STOP") {
    streaming = false;
    Serial.println("L:STREAM_STOP");
    return;
  }
  if (command == "RESET") {
    bool ok = init_mc1081();
    Serial.println(ok ? "L:MC1081_RESET_OK" : "E:INIT,MC1081 reset failed");
    print_config();
    return;
  }
  if (command.startsWith("START")) {
    int comma = command.indexOf(',');
    int rate_hz = 10;
    if (comma >= 0) rate_hz = command.substring(comma + 1).toInt();
    if (rate_hz < 1) rate_hz = 1;
    if (rate_hz > 100) rate_hz = 100;
    stream_period_ms = max((uint32_t)1, (uint32_t)(1000 / rate_hz));
    last_stream_ms = 0;
    streaming = true;
    Serial.printf("L:STREAM_START,rate_hz=%d\n", rate_hz);
    print_header();
    return;
  }
  if (command.startsWith("CAVG,")) {
    int value = command.substring(5).toInt();
    if (value == 1) cfg.cavg = CAVG_1;
    else if (value == 4) cfg.cavg = CAVG_4;
    else {
      Serial.println("E:CAVG,only 1 or 4 is supported by this diagnostic firmware");
      return;
    }
    Serial.printf("L:CAVG_SET,%d\n", value);
    return;
  }
  if (command.startsWith("DISCARD,")) {
    int value = command.substring(8).toInt();
    if (value < 0) value = 0;
    if (value > 10) value = 10;
    cfg.discard_count = (uint8_t)value;
    Serial.printf("L:DISCARD_SET,%u\n", cfg.discard_count);
    return;
  }
  if (command.startsWith("I2C,")) {
    uint32_t value = (uint32_t)command.substring(4).toInt();
    if (value < 10000UL) value = DEFAULT_I2C_FREQ_HZ;
    cfg.i2c_hz = value;
    Wire.setClock(cfg.i2c_hz);
    Serial.printf("L:I2C_SET,%lu\n", (unsigned long)cfg.i2c_hz);
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
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, cfg.i2c_hz);
  delay(100);

  Serial.println("L:ESP32 ready");
  bool ok = init_mc1081();
  Serial.println(ok ? "L:MC1081 initialized" : "E:INIT,MC1081 initialization failed");
  print_config();
  print_header();
}

void loop() {
  read_commands();
  if (streaming) {
    uint32_t now = millis();
    if (now - last_stream_ms >= stream_period_ms) {
      last_stream_ms = now;
      print_diag_frame();
    }
  }
  delay(1);
}
