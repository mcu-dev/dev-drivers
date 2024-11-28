/**
 ******************************************************************************
 * @file    LSM303.c
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    13-September-2024
 * @brief   Contains all the functionalities to control the LSM303DLHC
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

#include "lsm303.h"

uint8_t lsm303_setup(struct lsm303_dev **device,
                     struct lsm303_init_param lsm303_params) {
  uint8_t ret;

  struct lsm303_dev *dev;

  dev->acc_power_mode = lsm303_params.acc_power_mode;
  dev->acc_odr = lsm303_params.acc_odr;
  dev->acc_axes = lsm303_params.acc_axes;
  dev->acc_scale = lsm303_params.acc_scale;

  ret |= lsm303_set_power_mode(dev, dev->acc_power_mode);
  ret |= lsm303_acc_enable_axes(dev, dev->acc_axes);
  ret |= lsm303_acc_set_odr(dev, dev->acc_odr);
  ret |= lsm303_acc_set_scale(dev, dev->acc_scale);

  *device = dev;

  return ret;
}

uint8_t lsm303_set_power_mode(struct lsm303_dev *device,
                              enum lsm303_acc_power_mode mode) {
  uint8_t val = 0x00;

  // Get the current value of the register
  val = lsm303_reg_read();

  // Update only the power mode
  val = val | mode << POWER_MODE_MASK;

  return lsm303_reg_write();
}

uint8_t lsm303_acc_enable_axes(struct lsm303_dev *device,
                               enum lsm303_acc_axes_enable) {
  uint8_t ret = 0;

  return lsm303_reg_write();
}

uint8_t lsm303_acc_set_odr(struct lsm303_dev *device, enum lsm303_acc_odr) {
  uint8_t ret = 0;

  return lsm303_reg_write();
}

uint8_t lsm303_acc_set_scale(struct lsm303_dev *device,
                             enum lsm303_acc_full_scale) {
  uint8_t ret = 0;

  return lsm303_reg_write();
}

uint8_t lsm303_reg_read() {
  uint8_t ret = 0;

  return ret;
}

uint8_t lsm303_reg_write() {
  uint8_t ret = 0;

  return ret;
}
