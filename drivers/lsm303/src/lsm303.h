/**
 ******************************************************************************
 * @file    LSM303.h
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    13-September-2024
 * @brief   Contains all the prototypes for the LSM303.C
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 mcu-dev
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef LSM303_H
#define LSM303_H

#include "../../zephyr/dev-utils/i2c.h"
#include <math.h>
#include <stdint.h>

/* SA0 pin connection status */
#define SA0 1

enum lsm303_acc_power_mode {
  ACC_POWER_DOWN = 0x00,
  ACC_LOW_POWER = 0x88,
  ACC_NORMAL = 0x0E
};

enum lsm303_acc_odr {
  ACC_ODR_1HZ = 0x01,
  ACC_ODR_10HZ = 0x02,
  ACC_ODR_25HZ = 0x03,
  ACC_ODR_50HZ = 0x04,
  ACC_ODR_100HZ = 0x05,
  ACC_ODR_200HZ = 0x06,
  ACC_ODR_400HZ = 0x07,
  ACC_ODR_1_620KHZ = 0x08,
  ACC_ODR_1_344KHZ = 0x09,
  ACC_ODR_5_376KHZ = 0x09
};

enum lsm303_acc_axes_enable {
  ACC_AXES_DISABLE_ALL = 0x00,
  ACC_AXES_ENABLE_X = 0x01,
  ACC_AXES_ENABLE_Y = 0x02,
  ACC_AXES_ENABLE_Z = 0x04,
  ACC_AXES_ENABLE_XY = 0x03,
  ACC_AXES_ENABLE_XZ = 0x05,
  ACC_AXES_ENABLE_YZ = 0x06,
  ACC_AXES_ENABLE_XYZ = 0x07
};

enum lsm303_acc_full_scale {
  ACC_SCALE_2G = 0x00,
  ACC_SCALE_4G = 0x10,
  ACC_SCALE_8G = 0x20,
  ACC_SCALE_16G = 0x30
};

enum lsm303_mag_power_mode {
  MAG_CONTINUOUS_CONVERSION = 0x00,
  MAG_SINGLE_CONVERSION = 0x01,
  MAG_SLEEP_MODE = 0x02
};

enum lsm303_mag_full_scale {
  MAG_SCALE_1_3G = 0x20,
  MAG_SCALE_1_9G = 0x40,
  MAG_SCALE_2_5G = 0x60,
  MAG_SCALE_4_0G = 0x80,
  MAG_SCALE_4_7G = 0xA0,
  MAG_SCALE_5_6G = 0xC0,
  MAG_SCALE_8_1G = 0xE0
};

enum lsm303_mag_odr {
  MAG_ODR_0_75HZ = 0x00,
  MAG_ODR_1_5HZ = 0x01,
  MAG_ODR_3_0HZ = 0x02,
  MAG_ODR_7_5HZ = 0x03,
  MAG_ODR_15HZ = 0x04,
  MAG_ODR_30HZ = 0x05,
  MAG_ODR_75HZ = 0x06,
  MAG_ODR_220HZ = 0x07
};

struct lsm303_dev {
  enum lsm303_acc_power_mode acc_power_mode;
  enum lsm303_mag_power_mode mag_power_mode;
  enum lsm303_acc_odr acc_odr;
  enum lsm303_mag_odr mag_odr;
  enum lsm303_acc_axes_enable acc_axes;
  enum lsm303_acc_full_scale acc_scale;
  enum lsm303_mag_full_scale mag_scale;
  bool temp_en;
};

struct lsm303_init_param {
  enum lsm303_acc_power_mode acc_power_mode;
  enum lsm303_mag_power_mode mag_power_mode;
  enum lsm303_acc_odr acc_odr;
  enum lsm303_mag_odr mag_odr;
  enum lsm303_acc_axes_enable acc_axes;
  enum lsm303_acc_full_scale acc_scale;
  enum lsm303_mag_full_scale mag_scale;
  bool temp_en;
};

/*****************************ID REGISTERS*************************************/
#if SA0
#define ACC_I2C_ADDRESS 0x19
#else
#define ACC_I2C_ADDRESS 0x18
#endif
#define MAG_I2C_ADDRESS 0x1E

/***********************ACCELEROMETER REGISTERS********************************/

typedef enum {
  CTRL_REG1_A = 0x20,
  CTRL_REG2_A = 0x21,
  CTRL_REG3_A = 0x22,
  CTRL_REG4_A = 0x23,
  CTRL_REG5_A = 0x24,
  CTRL_REG6_A = 0x25,
  REFERENCE_A = 0x26,
  STATUS_REG_A = 0x27,
  OUT_X_L_A = 0x28,
  OUT_X_H_A = 0x29,
  OUT_Y_L_A = 0x2A,
  OUT_Y_H_A = 0x2B,
  OUT_Z_L_A = 0x2C,
  OUT_Z_H_A = 0x2D,
  FIFO_CTRL_REG_A = 0x2E,
  FIFO_SRC_REG_A = 0x2F,
  INT1_CFG_A = 0x30,
  INT1_SRC_A = 0x31,
  INT1_THS_A = 0x32,
  INT1_DURATION_A = 0x33,
  INT2_CFG_A = 0x34,
  INT2_SRC_A = 0x35,
  INT2_THS_A = 0x36,
  INT2_DURATION_A = 0x37,
  CLICK_CFG_A = 0x38,
  CLICK_SRC_A = 0x39,
  CLICK_THS_A = 0x3A,
  TIME_LIMIT_A = 0x3B,
  TIME_LATENCY_A = 0x3C,
  TIME_WINDOW_A = 0x3D
} LSM303DLHC_ACC_REG;

/***********************MAGNETOMETER REGISTERS*********************************/
typedef enum {
  CRA_REG_M = 0x00,
  CRB_REG_M = 0x01,
  MR_REG_M = 0x02,
  OUT_X_H_M = 0x03,
  OUT_X_L_M = 0x04,
  OUT_Z_H_M = 0x05,
  OUT_Z_L_M = 0x06,
  OUT_Y_H_M = 0x07,
  OUT_Y_L_M = 0x08,
  SR_REG_M = 0x09,
  IRA_REG_M = 0x0A,
  IRB_REG_M = 0x0B,
  IRC_REG_M = 0x0C,
  TEMP_OUT_H_M = 0x31,
  TEMP_OUT_L_M = 0x32
} LSM303DLHC_MAG_REG;

/*********************************MASKS****************************************/

#define POWER_MODE_MASK 0x03

/*******************************PROTOTYPES*************************************/
uint8_t lsm303_setup(struct lsm303_dev **device,
                     struct lsm303_init_param lsm303_params);

uint8_t lsm303_set_power_mode(struct lsm303_dev *device,
                              enum lsm303_acc_power_mode mode);
#endif /* LSM303_H */