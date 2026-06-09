#ifndef MC1081_REG_H
#define MC1081_REG_H

#include <Arduino.h>

#define MC1081_ADDR 0x70
#define Cref 20.0f
#define IN_CLK 19.2f

#define OSC2_Channel_0 (1 << 0)
#define OSC2_Channel_1 (1 << 1)
#define OSC2_Channel_2 (1 << 2)
#define OSC2_Channel_3 (1 << 3)
#define OSC2_Channel_4 (1 << 4)
#define OSC2_Ref_Channel (1 << 5)

typedef enum {
  FIN_DIV_1 = 0x00,
  FIN_DIV_2 = 0x01,
  FIN_DIV_4 = 0x02,
  FIN_DIV_8 = 0x03,
  FIN_DIV_16 = 0x04,
  FIN_DIV_32 = 0x05,
  FIN_DIV_64 = 0x06
} MC1081_findiv_config;

typedef enum {
  FREF_DIV_1 = 0x00,
  FREF_DIV_2 = 0x01,
  FREF_DIV_4 = 0x02,
  FREF_DIV_8 = 0x03
} MC1081_frefdiv_config;

typedef enum {
  DRIVE_I_4UA = 0x00,
  DRIVE_I_8UA = 0x01,
  DRIVE_I_16UA = 0x02,
  DRIVE_I_42UA = 0x03,
  DRIVE_I_100UA = 0x04,
  DRIVE_I_250UA = 0x05,
  DRIVE_I_500UA = 0x06,
  DRIVE_I_1000UA = 0x07,
  DRIVE_I_2000UA = 0x08
} MC1081_driver_i_config;

typedef enum {
  OSC2_AMPLITUDE_0p4V = 0x00,
  OSC2_AMPLITUDE_0p8V = 0x01,
  OSC2_AMPLITUDE_1p2V = 0x02,
  OSC2_AMPLITUDE_1p6V = 0x03,
  OSC2_AMPLITUDE_2p0V = 0x04,
  OSC2_AMPLITUDE_2p4V = 0x05
} MC1081_osc2_amplitude_config;

typedef enum {
  OSC1 = 0,
  OSC2 = 1
} MC1081_osc_mode;

typedef struct {
  uint8_t MC1081_CNT_CFG;
  MC1081_findiv_config MC1081_FINDIV;
  MC1081_frefdiv_config MC1081_FREFDIV;
  MC1081_osc_mode MC1081_OSC_MODE;
  MC1081_osc2_amplitude_config MC1081_OSC2_AMPLITUDE;
  MC1081_driver_i_config MC1081_DRIVEI;
  uint8_t MC1081_OSC2_CHANNEL;
  uint8_t MC1081_SHLD_CFG;
  uint8_t MC1081_CAVG;
} MC1081_InitStructure;

typedef struct {
  uint16_t data_ch[5];
  uint16_t data_ref;
  float freq_ch[5];
  float freq_ref;
  float cap_ch[5];
} CAP_AFE_DoubleEnded;

typedef enum {
  D0_MSB = 0x02,
  DREF_MSB = 0x16,
  OSC2_OF = 0x1A,
  MC_STATUS = 0x1B,
  C_CMD = 0x1D,
  CNT_CFG = 0x1E,
  DIV_CFG = 0x1F,
  OSC2_DCHS = 0x24,
  OSC2_CFG = 0x25,
  SHLD_CFG = 0x26,
  MC1081_RESET = 0x69
} MC1081_REG;

#define MC1081_RESET_CMD 0x7A
#define OF_CLEAR 0x10

#define OSC1_LDO_H (0x01U << 7)
#define SHLD_DIS 0x00

#define OSC_SEL_POS 7
#define OSC2_EN (0x01U << OSC_SEL_POS)
#define SLEEP_POS 6
#define SLEEP_EN (0x01U << SLEEP_POS)
#define CAVG_POS 4
#define CAVG_1 (0x00U << CAVG_POS)
#define CAVG_4 (0x01U << CAVG_POS)
#define CAVG_8 (0x02U << CAVG_POS)
#define CAVG_32 (0x03U << CAVG_POS)
#define OS_SD_ONE 0x03

#endif
