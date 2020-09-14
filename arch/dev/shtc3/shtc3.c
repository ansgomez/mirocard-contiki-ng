/*
 * 
 * Copyright (c) 2020, Andres Gomez, Miromico AG
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
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
#define LOG_LEVEL                     LOG_LEVEL_NONE
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
  board_i2c_select(SHTC3_INTERFACE, SHTC3_I2C_ADDRESS);
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
    printf("Unable to write\n");
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
