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

/**
 * @brief Initializes and sets up the LSM303 device.
 *
 * Allocates and initializes the device structure and configures it with the
 * specified initialization parameters.
 *
 * @param device        Double pointer to the LSM303 device structure to be
 * allocated and initialized.
 * @param lsm303_params Initialization parameters for the LSM303 device.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_setup(struct lsm303_dev **device,
                     struct lsm303_init_param lsm303_params) {
  uint8_t ret;

  struct lsm303_dev *dev;

  dev->acc_power_mode = lsm303_params.acc_power_mode;
  dev->acc_odr        = lsm303_params.acc_odr;
  dev->acc_axes       = lsm303_params.acc_axes;
  dev->acc_scale      = lsm303_params.acc_scale;

  ret |= lsm303_set_power_mode(dev, dev->acc_power_mode);
  ret |= lsm303_acc_enable_axes(dev, dev->acc_axes);
  ret |= lsm303_acc_set_odr(dev, dev->acc_odr);
  ret |= lsm303_acc_set_scale(dev, dev->acc_scale);

  *device = dev;

  return ret;
}

/**
 * @brief Sets the accelerometer power mode.
 *
 * Configures the accelerometer's power mode to optimize power consumption
 * or performance based on the specified mode.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_set_power_mode(struct lsm303_dev *device,
                              enum lsm303_acc_power_mode mode) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (mode == ACC_POWER_DOWN) {
    val &= ~0xF0;
  } else {
    val = val | mode << ACC_POWER_MODE_MASK;
  }

  data_buffer[1] = CTRL_REG1_A;
  data_buffer[0] = val;

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Enables or disables specific accelerometer axes.
 *
 * Configures the accelerometer to enable or disable the specified axes
 * for data measurement.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param axes   Axes configuration to enable or disable specific accelerometer
 * axes.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_acc_enable_axes(struct lsm303_dev *device,
                               enum lsm303_acc_axes_enable axes) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x07;
  val = val | axes << ACC_AXES_MASK;

  data_buffer[1] = CTRL_REG1_A;
  data_buffer[0] = val;

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the Output Data Rate (ODR) for the accelerometer.
 *
 * Sets the output data rate of the accelerometer to control how frequently
 * the device outputs data.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param odr    Desired output data rate setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_acc_set_odr(struct lsm303_dev *device, enum lsm303_acc_odr odr) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0xF0;
  val = val | odr << ACC_ODR_MASK;

  data_buffer[1] = CTRL_REG1_A;
  data_buffer[0] = val;

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer scale for the LSM303 device.
 *
 * Sets the full-scale range of the accelerometer based on the specified scale
 * parameter.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param scale  Full-scale range setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_acc_set_scale(struct lsm303_dev *device,
                             enum lsm303_acc_full_scale scale) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x18;
  val = val | scale << ACC_SCALE_MASK;

  data_buffer[1] = CTRL_REG4_A;
  data_buffer[0] = val;

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Reads a register value from the LSM303 device via I2C.
 *
 * Reads a single register from the LSM303 device and stores the retrieved value
 * in the specified buffer.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param address     I2C address of the LSM303 device.
 * @param reg         Register address to read from.
 * @param read_data   Pointer to the buffer where the read value will be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_i2c_read(struct lsm303_dev *device, uint8_t address, uint8_t reg,
                        uint8_t *read_data) {
  uint8_t ret = 0;

  // TO DO: I2C communication
  return ret;
}

/**
 * @brief Writes data to a register of the LSM303 device over I2C.
 *
 * This function writes data to a specified register of the LSM303 device
 * using the provided data buffer.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param address     I2C address of the LSM303 device.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
uint8_t lsm303_i2c_write(struct lsm303_dev *device, uint8_t address,
                         uint8_t *data_buffer) {
  uint8_t ret = 0;

  // TO DO: I2C communication
  return ret;
}
