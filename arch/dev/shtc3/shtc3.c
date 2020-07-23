/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gpio-hal.h"
#include "ti-lib.h"
#include "board-i2c.h"
#include "shtc3.h"
#include "sensor-common.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE                    "shtc3"
#define LOG_LEVEL                     LOG_LEVEL_DBG
/*---------------------------------------------------------------------------*/
#ifndef SHTC3_I2C_CONTROLLER

#define SHTC3_I2C_CONTROLLER          0xFF /* No controller */

#define SHTC3_I2C_PIN_SCL             GPIO_HAL_PIN_UNKNOWN
#define SHTC3_I2C_PIN_SDA             GPIO_HAL_PIN_UNKNOWN

#define SHTC3_I2C_ADDRESS             0x70

#endif /* SHTC3_I2C_CONTROLLER */
/*---------------------------------------------------------------------------*/
// single shot commands
#if SHTC3_USE_CLOCKSTRETCH
#define SHTC3_CODE_SINGLE_HIGH        0x7CA2
#define SHTC3_CODE_SINGLE_LOW         0x6458
// #define SHTC3_CODE_SINGLE_LOW         0x5C24
#else
#define SHTC3_CODE_SINGLE_HIGH        0x7866
#define SHTC3_CODE_SINGLE_LOW         0x609C
// #define SHTC3_CODE_SINGLE_LOW         0x58E0
#endif
/*---------------------------------------------------------------------------*/
// general commands
#define SHTC3_CODE_SLEEP              0xB098
#define SHTC3_CODE_WAKEUP             0x3517
#define SHTC3_CODE_READ_ID            0xEFC8
#define SHTC3_CODE_SOFT_RESET         0x805D
//to be updated?
#define SHTC3_CODE_STATUS             0xF32D 
//to be updated?
#define SHTC3_CODE_CLEAR_STATUS       0x3041
/*---------------------------------------------------------------------------*/
#define SHTC3_READ_RETRY_ITERATIONS   50
/*---------------------------------------------------------------------------*/
int32_t
shtc3_convert_temperature(uint16_t raw)
{
#if SHTC3_USE_FAHRENHEIT
  return -4900 + (31500 * (int32_t)raw) / 65535;
#else
  return -4500 + (17500 * (int32_t)raw) / 65535;
#endif
}
/*---------------------------------------------------------------------------*/
int32_t
shtc3_convert_humidity(uint16_t raw)
{
  return (10000 * (int32_t)raw) / 65535;
}
/*---------------------------------------------------------------------------*/
static void
select_on_bus(void *conf)
{
  // select sensor slave
  LOG_DBG("I2C select (0x%02x)\n", SHTC3_I2C_ADDRESS);
  board_i2c_select(BOARD_I2C_INTERFACE_0, SHTC3_I2C_ADDRESS);
}
/*---------------------------------------------------------------------------*/
static void
deselect(void *conf)
{
  // deselect on I2C, reseting I2C interface
  LOG_DBG("I2C deselect\n");
  board_i2c_deselect();
}
/*---------------------------------------------------------------------------*/
bool
shtc3_read(void *conf, uint8_t *rbuf, uint8_t length)
{
  bool ret;

  // read the actual data
  LOG_DBG("I2C read\n");
  ret = board_i2c_read(rbuf, length);
  if (!ret) {
    LOG_DBG("I2C read FAILED\n");
    return ret;
  }

  return true;
}
/*---------------------------------------------------------------------------*/
bool
shtc3_write(void *conf, uint16_t command)
{
  uint8_t wbuffer[2];
  bool ret;
  wbuffer[0] = (uint8_t)(command >> 8);
  wbuffer[1] = (uint8_t)(command & 0xff);

  // read the actual data
  LOG_DBG("I2C write\n");
  ret = board_i2c_write(wbuffer, sizeof(wbuffer));
  if (!ret) {
    LOG_DBG("I2C write FAILED\n");
    return ret;
  }

  return true;
}
/*---------------------------------------------------------------------------*/
bool
shtc3_init(void *conf)
{
  // deselect to reset I2C to known state
  deselect(conf);

  // no sensor specific configuration needed
  return true;
}
/*---------------------------------------------------------------------------*/
bool shtc3_wakeup(void *conf) {
  // select device
  select_on_bus(conf);

  return shtc3_write(conf, SHTC3_CODE_WAKEUP);
}
/*---------------------------------------------------------------------------*/

bool shtc3_sleep(void *conf) {
  // select device
  select_on_bus(conf);

  return shtc3_write(conf, SHTC3_CODE_SLEEP);
}
/*---------------------------------------------------------------------------*/
bool
shtc3_read_values(void *conf, shtc3_repeatability_t repeatability,
                  uint16_t *temperature, uint16_t *humidity)
{
  uint8_t rbuf[6];
  uint16_t cmd;
  bool ret;

  // get command for chosen repeatability level
  switch(repeatability) {
  case SHTC3_REPEATABILITY_HIGH:
    cmd = SHTC3_CODE_SINGLE_HIGH;
    break;
  case SHTC3_REPEATABILITY_LOW:
  default:
    cmd = SHTC3_CODE_SINGLE_LOW;
    break;
  }

  // select device
  select_on_bus(conf);

  // request status
  ret = shtc3_write(conf, cmd);
  if (!ret) {
    return ret;
  }

  // read data (retry until data ready, up to 15 ms for high repeatability)
  uint8_t retry = SHTC3_READ_RETRY_ITERATIONS;
  while (retry > 0) {
    ret = shtc3_read(conf, rbuf, sizeof(rbuf));
    if (ret == true) {
      break;
    }
    retry = retry - 1;
  }
  if (!ret) {
    return ret;
  }

  // assemble temperature and humidity values
  *temperature = ((uint16_t)rbuf[0] << 8) | ((uint16_t)rbuf[1]);
  *humidity = ((uint16_t)rbuf[3] << 8) | ((uint16_t)rbuf[4]);

  return true;
}
/*---------------------------------------------------------------------------*/
bool
shtc3_read_status(void *conf, uint16_t *status)
{
  uint8_t rbuf[2];
  bool ret;

  // select device
  select_on_bus(conf);

  // request status
  ret = shtc3_write(conf, SHTC3_CODE_STATUS);
  if (!ret) {
    return ret;
  }

  // read register data
  ret = shtc3_read(conf, rbuf, sizeof(rbuf));
  if (!ret) {
    return ret;
  }

  // assemble status register value
  *status = ((uint16_t)rbuf[0] << 8) | ((uint16_t)rbuf[1]);

  return true;
}
/*---------------------------------------------------------------------------*/
bool
shtc3_clear_status(void *conf)
{
  // select device
  select_on_bus(conf);

  return shtc3_write(conf, SHTC3_CODE_CLEAR_STATUS);
}
/*---------------------------------------------------------------------------*/
bool
shtc3_soft_reset(void *conf)
{
  // select device
  select_on_bus(conf);

  return shtc3_write(conf, SHTC3_CODE_SOFT_RESET); 
}
/*---------------------------------------------------------------------------*/
