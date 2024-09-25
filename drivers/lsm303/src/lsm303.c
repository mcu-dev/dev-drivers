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
  uint8_t ret = 0;

  return ret;
}

uint8_t lsm303_set_power_mode(struct lsm303_dev *device,
                              enum lsm303_acc_power_mode mode) {
  uint8_t val = 0x00;

  val = val | mode << POWER_MODE_MASK;

  return i2c0_write_bytes();
}