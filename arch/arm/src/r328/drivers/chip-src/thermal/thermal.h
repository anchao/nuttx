/*
 * Copyright 2019 AllWinnertech  Co., Ltd
 * frank@allwinnertech.com
 */

#ifndef __THERMAL_H__
#define __THERMAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define THS_BASE			0x05070400
#define THS_NUM				1

#define OFFSET				-2794
#define SCALE				-67

#define THS_CTL				THS_BASE + 0x0
#define THS_EN				THS_BASE + 0x4
#define THS_PCTL			THS_BASE + 0x8
#define THS_DICTL			THS_BASE + 0x10
#define THS_SICTL			THS_BASE + 0x14
#define THS_AICTL			THS_BASE + 0x18
#define THS_DIS				THS_BASE + 0x20
#define THS_SIS				THS_BASE + 0x24
#define THS_AOIS			THS_BASE + 0x28
#define THS_AIS				THS_BASE + 0x2C
#define THS_MFC				THS_BASE + 0x30
#define THS_ATC				THS_BASE + 0x40
#define THS_STC				THS_BASE + 0x80
#define THS_CALIB			THS_BASE + 0xA0
#define THS_DATA			THS_BASE + 0xC0

#define THS_CTRL_T_ACQ(x)			((0xffff & (x)) << 16)
#define THS_FILTER_EN				0x4
#define THS_FILTER_TYPE(x)			(0x3 & (x))
#define THS_PC_TEMP_PERIOD(x)			((0xfffff & (x)) << 12)
#define TEMP_CALIB_MASK				0xfff

#define FT_TEMP_MASK				0xfff
#define TEMP_TO_REG				672
#define CALIBRATE_DEFAULT			0x800

void ths_init(void);
int  ths_calibrate(short int *buf, unsigned int len);
int ths_reg2temp(int reg);
int ths_get_temp(unsigned int num, int *temp);

#ifdef __cplusplus
}
#endif

#endif /* __THERMAL_H__ */
