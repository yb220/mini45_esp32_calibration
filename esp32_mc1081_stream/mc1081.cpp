#include "mc1081.h"
#include <Wire.h>
#include <math.h>
#include <string.h>

bool CAP_AFE_Transmit(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.write(data);
  return Wire.endTransmission() == 0;
}

bool CAP_AFE_Receive(uint8_t deviceAddr, uint8_t regAddr, uint8_t *pData, uint8_t size) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(deviceAddr, size);
  if (received != size) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < size; i++) {
    pData[i] = Wire.read();
  }
  return true;
}

bool Registers_Init(MC1081_InitStructure *init) {
  uint8_t div_value = (init->MC1081_FINDIV << 4) | init->MC1081_FREFDIV;
  uint8_t cfg_value = init->MC1081_DRIVEI | (init->MC1081_OSC2_AMPLITUDE << 4) | OSC1_LDO_H;
  bool ok = true;
  ok &= CAP_AFE_Transmit(MC1081_ADDR, MC1081_RESET, MC1081_RESET_CMD);
  delay(5);
  ok &= CAP_AFE_Transmit(MC1081_ADDR, DIV_CFG, div_value);
  ok &= CAP_AFE_Transmit(MC1081_ADDR, CNT_CFG, init->MC1081_CNT_CFG);
  ok &= CAP_AFE_Transmit(MC1081_ADDR, SHLD_CFG, init->MC1081_SHLD_CFG);
  ok &= CAP_AFE_Transmit(MC1081_ADDR, OSC2_DCHS, init->MC1081_OSC2_CHANNEL);
  ok &= CAP_AFE_Transmit(MC1081_ADDR, OSC2_CFG, cfg_value);
  return ok;
}

bool MC1081_OSC2_Measure(CAP_AFE_DoubleEnded *cap, MC1081_InitStructure *init) {
  memset(cap, 0, sizeof(CAP_AFE_DoubleEnded));
  uint8_t status = 0x01;
  uint8_t dat[2] = {0};
  uint8_t overflow = 0;
  int timeout = 500;

  if (!CAP_AFE_Transmit(MC1081_ADDR, MC_STATUS, OF_CLEAR)) return false;
  if (!CAP_AFE_Transmit(MC1081_ADDR, C_CMD, OSC2_EN | SLEEP_EN | CAVG_1 | OS_SD_ONE)) return false;

  do {
    delayMicroseconds(20);
    if (!CAP_AFE_Receive(MC1081_ADDR, MC_STATUS, &status, 1)) return false;
    timeout--;
  } while ((status & 0x01) && timeout > 0);
  if (timeout <= 0) return false;

  if (!CAP_AFE_Receive(MC1081_ADDR, OSC2_OF, &overflow, 1)) return false;
  if (!CAP_AFE_Receive(MC1081_ADDR, DREF_MSB, dat, 2)) return false;
  cap->data_ref = ((uint16_t)dat[0] << 8) | dat[1];
  if (cap->data_ref == 0 || cap->data_ref >= 65535) return false;

  uint8_t fin_div = 1 << init->MC1081_FINDIV;
  uint8_t fref_div = 1 << init->MC1081_FREFDIV;
  cap->freq_ref = init->MC1081_CNT_CFG * IN_CLK * fin_div / (float)fref_div / (float)cap->data_ref;

  for (int i = 0; i < 5; i++) {
    cap->cap_ch[i] = NAN;
    if (((overflow >> i) & 0x01) != 0) return false;
    if (!CAP_AFE_Receive(MC1081_ADDR, (uint8_t)(D0_MSB + 2 * i), dat, 2)) return false;
    cap->data_ch[i] = ((uint16_t)dat[0] << 8) | dat[1];
    if (cap->data_ch[i] == 0 || cap->data_ch[i] >= 65535) return false;
    cap->freq_ch[i] = init->MC1081_CNT_CFG * IN_CLK * fin_div / (float)fref_div / (float)cap->data_ch[i];
    cap->cap_ch[i] = (float)cap->data_ch[i] / (float)cap->data_ref * Cref;
  }
  return true;
}
