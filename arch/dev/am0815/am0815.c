/**
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
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
#include "dev/spi.h"
#include "gpio-hal.h"
#include "am0815.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE                    "am0815"
#define LOG_LEVEL                     LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/
/* Helper function macros */
#define BCD_TO_DEC(x)  (((x) & 0x0F) + 10 * ((x) >> 4))
#define DEC_TO_BCD(x)  ((((x) / 10) << 4) | ((x) % 10))
/*---------------------------------------------------------------------------*/
#ifndef AM0815_SPI_CONTROLLER

#define AM0815_SPI_CONTROLLER         0xFF /* No controller */

#define AM0815_SPI_PIN_SCK            GPIO_HAL_PIN_UNKNOWN
#define AM0815_SPI_PIN_MOSI           GPIO_HAL_PIN_UNKNOWN
#define AM0815_SPI_PIN_MISO           GPIO_HAL_PIN_UNKNOWN
#define AM0815_SPI_PIN_CS             GPIO_HAL_PIN_UNKNOWN

#endif /* AM0815_SPI_CONTROLLER */
/*---------------------------------------------------------------------------*/
/* RTC instruction codes */
#define AM0815_CODE_READ              0x00 /**< Read Register Command Flag */
#define AM0815_CODE_WRITE             0x80 /**< Write Register Command Flag */
/*---------------------------------------------------------------------------*/
/* RTC registers */
#define AM0815_HUNDREDTHS_ADDR        0x00 /**< hundredths address */
#define AM0815_SECONDS_ADDR           0x01 /**< seconds address */
#define AM0815_MINUTES_ADDR           0x02 /**< minutes address */
#define AM0815_HOURS_ADDR             0x03 /**< hours address */
#define AM0815_DATE_ADDR              0x04 /**< date address */
#define AM0815_MONTHS_ADDR            0x05 /**< months address */
#define AM0815_YEARS_ADDR             0x06 /**< years address */
#define AM0815_WEEKDAYS_ADDR          0x07 /**< weekdays address */

#define AM0815_HUNDREDTHS_ALARM_ADDR  0x08 /**< hundredths alarm address */
#define AM0815_SECONDS_ALARM_ADDR     0x09 /**< seconds alarm address */
#define AM0815_MINUTES_ALARM_ADDR     0x0A /**< minutes alarm address */
#define AM0815_HOURS_ALARM_ADDR       0x0B /**< hours alarm address */
#define AM0815_DATE_ALARM_ADDR        0x0C /**< date alarm address */
#define AM0815_MONTHS_ALARM_ADDR      0x0D /**< months alarm address */
#define AM0815_WEEKDAYS_ALARM_ADDR    0x0E /**< weekdays alarm address */

#define AM0815_STATUS_ADDR            0x0F /**< status register address */
#define AM0815_CONTROL1_ADDR          0x10 /**< control register 1 address */
#define AM0815_CONTROL2_ADDR          0x11 /**< control register 2 address */

#define AM0815_INTMASK_ADDR           0x12 /**< interrupt mask address */
#define AM0815_SWQ_ADDR               0x13 /**< register address */
#define AM0815_CAL_XT_ADDR            0x14 /**< register address */
#define AM0815_CAL_RC_HIGH_ADDR       0x15 /**< register address */
#define AM0815_CAL_RC_LOW_ADDR        0x16 /**< register address */
#define AM0815_INT_POL_ADDR           0x17 /**< interrupt polarity address */

#define AM0815_TIMER_CONTROL_ADDR     0x18 /**< register address */
#define AM0815_TIMER_ADDR             0x19 /**< register address */
#define AM0815_TIMER_INITIAL_ADDR     0x1A /**< register address */
#define AM0815_WDT_ADDR               0x1B /**< register address */

#define AM0815_OSC_CONTROL_ADDR       0x1C /**< oscillator control address */
#define AM0815_OSC_STATUS_ADDR        0x1D /**< oscillator status address */

#define AM0815_CONFIG_KEY_ADDR        0x1F /**< configuration key address */
#define AM0815_TRICKLE_ADDR           0x20 /**< register address */

#define AM0815_BREF_CONTROL_ADDR      0x21 /**< VBAT reference voltage address */
#define AM0815_AFCTRL_ADDR            0x26 /**< autocalibration filter address */
#define AM0815_BATMODE_ADDR           0x27 /**< battery mode IO config address */
#define AM0815_ID_ADDR                0x28 /**< ID address */
#define AM0815_ASTAT_ADDR             0x2F /**< analog status register address */
#define AM0815_OCTRL_ADDR             0x30 /**< output control register address */

#define AM0815_EXT_ADDR               0x3F /**< extension RAM address */
#define AM0815_SRAM_ADDR              0x40 /**< standard data address */
/*---------------------------------------------------------------------------*/
#define AM0815_KEY_OSC                0xA1 /**< oscillator control register key */
#define AM0815_KEY_RESET              0x3C /**< software reset key */
#define AM0815_KEY_CONFIG             0x9D /**< protected configuration registers key */
/*---------------------------------------------------------------------------*/
#define AM0815_TIME_SIZE              7
#define AM0815_RAM_SIZE               32
/*---------------------------------------------------------------------------*/
// #define VERIFY_PART_LOCKED           -2
// #define VERIFY_PART_ERROR            -1
// #define VERIFY_PART_POWERED_DOWN      0
// #define VERIFY_PART_OK                1
/*---------------------------------------------------------------------------*/
static spi_device_t am0815_spi_configuration_default = {
  .spi_controller = AM0815_SPI_CONTROLLER,
  .pin_spi_sck = AM0815_SPI_PIN_SCK,
  .pin_spi_miso = AM0815_SPI_PIN_MISO,
  .pin_spi_mosi = AM0815_SPI_PIN_MOSI,
  .pin_spi_cs = AM0815_SPI_PIN_CS,
  .spi_bit_rate = 2000000,
  .spi_pha = 0,
  .spi_pol = 0,
};
/*---------------------------------------------------------------------------*/
/**
 * Convert register values to time structure
 */
static void
am0815_reg_to_time(const uint8_t *reg, am0815_tm_t *timeptr)
{
  timeptr->split = BCD_TO_DEC(reg[0]);
  timeptr->sec = BCD_TO_DEC(reg[1] & 0x7F);
  timeptr->min = BCD_TO_DEC(reg[2] & 0x7F);
  timeptr->hour = BCD_TO_DEC(reg[3] & 0x3F);
  timeptr->day = BCD_TO_DEC(reg[4] & 0x3F);
  timeptr->month = BCD_TO_DEC(reg[5] & 0x1F);
  timeptr->year = BCD_TO_DEC(reg[6]);
}
/*---------------------------------------------------------------------------*/
/**
 * Convert time structure to register values
 */
static void
am0815_time_to_reg(const am0815_tm_t *timeptr, uint8_t *reg)
{
  reg[0] = DEC_TO_BCD(timeptr->split);
  reg[1] = DEC_TO_BCD(timeptr->sec);
  reg[2] = DEC_TO_BCD(timeptr->min);
  reg[3] = DEC_TO_BCD(timeptr->hour);
  reg[4] = DEC_TO_BCD(timeptr->day);
  reg[5] = DEC_TO_BCD(timeptr->month);
  reg[6] = DEC_TO_BCD(timeptr->year);
}
/*---------------------------------------------------------------------------*/
/**
 * Get SPI configuration, return default configuration if NULL
 */
static spi_device_t*
get_spi_conf(spi_device_t* conf)
{
  if(conf == NULL) {
    return &am0815_spi_configuration_default;
  }
  return conf;
}
/*---------------------------------------------------------------------------*/
/**
 * Pull CS line low to start communication
 */
static bool
select_on_bus(spi_device_t* spi_config)
{
  if(spi_select(spi_config) == SPI_DEV_STATUS_OK) {
    LOG_DBG("SPI selected\n");
    return true;
  }
  LOG_DBG("SPI select FAILED\n");
  return false;
}
/*---------------------------------------------------------------------------*/
/**
 * Pull CS high to stop communication
 */
static void
deselect(spi_device_t* spi_config)
{
  spi_deselect(spi_config);
  LOG_DBG("SPI deselected\n");
}
/*---------------------------------------------------------------------------*/
bool
am0815_open(spi_device_t *conf)
{
  spi_device_t *spi_config = get_spi_conf(conf);

  /* Check if platform has ext-fram */
  if(spi_config->pin_spi_sck == GPIO_HAL_PIN_UNKNOWN) {
    LOG_WARN("AM0815 RTC not present in platform!\n");
    return false;
  }

  if(spi_acquire(spi_config) != SPI_DEV_STATUS_OK) {
    LOG_DBG("SPI acquire FAILED\n");
    return false;
  }

  deselect(spi_config);

  return true;
}
/*---------------------------------------------------------------------------*/
bool
am0815_close(spi_device_t *conf)
{
  spi_device_t *spi_config = get_spi_conf(conf);

  if(spi_release(spi_config) != SPI_DEV_STATUS_OK) {
    return false;
  }

  return true;
}
/*---------------------------------------------------------------------------*/
bool
am0815_read_reg(spi_device_t *conf, uint8_t addr, uint8_t length, uint8_t *buf)
{
  bool ret;
  uint8_t cmd;

  spi_device_t *spi_config = get_spi_conf(conf);

  // assemble read command
  cmd = AM0815_CODE_READ | (addr & 0x7F);

  if(select_on_bus(spi_config) == false) {
    return false;
  }

  LOG_DBG("read register: address\n");
  if(spi_write(spi_config, &cmd, sizeof(cmd)) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(spi_config);
    return false;
  }

  LOG_DBG("read register: data\n");
  ret = (spi_read(spi_config, buf, length) == SPI_DEV_STATUS_OK);

  deselect(spi_config);

  return ret;
}
/*---------------------------------------------------------------------------*/
bool
am0815_write_reg(spi_device_t *conf, uint8_t addr, uint8_t length, const uint8_t *buf)
{
  bool ret;
  uint8_t cmd;

  spi_device_t *spi_config = get_spi_conf(conf);

  // assemble SRAM write command
  cmd = AM0815_CODE_WRITE | (addr & 0x7F);

  if(select_on_bus(spi_config) == false) {
    return false;
  }

  LOG_DBG("write register: command\n");
  if(spi_write(spi_config, &cmd, sizeof(cmd)) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(spi_config);
    return false;
  }

  LOG_DBG("write register: data\n");
  ret = (spi_write(spi_config, buf, length) == SPI_DEV_STATUS_OK);

  deselect(spi_config);

  return ret;
}
/*---------------------------------------------------------------------------*/
inline bool
am0815_set_reg(spi_device_t *conf, uint8_t addr, uint8_t value) {
  return am0815_write_reg(conf, addr, 1, &value);
}
/*---------------------------------------------------------------------------*/
bool
am0815_set_mode(spi_device_t *conf, am0815_mode_t mode)
{
  bool ret;
  if (mode == AM0815_MODE_STOP) {
    // halt the RTC
    return am0815_set_reg(conf, AM0815_CONTROL1_ADDR, 0x91);
  }
  
  // get mode specific osc register value
  uint8_t reg;
  switch (mode) {
  default:
    LOG_DBG("AM0815 invalid mode, fallback to default\n");
  case AM0815_MODE_CRYSTAL:
    reg = 0x00;
    break;
  case AM0815_MODE_RC_ONLY:
    reg = 0x80;
    break;
  case AM0815_MODE_RC_BACKUP:
    reg = 0x90;
    break;
  case AM0815_MODE_AUTOCALIBRATE_9MIN:
    reg = 0xC0;
    break;
  case AM0815_MODE_AUTOCALIBRATE_17MIN:
    reg = 0xE0;
    break;
  }
  // set oscillator register
  // 1. unlock register
  ret = am0815_set_reg(conf, AM0815_CONFIG_KEY_ADDR, AM0815_KEY_OSC);
  if (ret == false) {
    return false;
  }
  // 2. set register value
  ret = am0815_set_reg(conf, AM0815_OSC_CONTROL_ADDR, reg);
  if (ret == false) {
    return false;
  }
  // 3. unlock register (ignore errors)
  am0815_set_reg(conf, AM0815_CONFIG_KEY_ADDR, 0x00);

  // resume RTC in case it was disabled
  ret = am0815_set_reg(conf, AM0815_CONTROL1_ADDR, 0x11);

  return ret;
}
/*---------------------------------------------------------------------------*/
bool
am0815_get_time(spi_device_t *conf, am0815_tm_t *timeptr)
{
  uint8_t rbuf[AM0815_TIME_SIZE];
  bool ret;

  // read data
  ret = am0815_read_reg(conf, AM0815_HUNDREDTHS_ADDR, AM0815_TIME_SIZE, rbuf);
  if (ret == false) {
    *timeptr = AM0815_TIMESTAMP_INVALID;
    return false;
  }

  // perform time transformation
  am0815_reg_to_time(rbuf, timeptr);

  return true;
}
/*---------------------------------------------------------------------------*/
bool
am0815_set_time(spi_device_t *conf, const am0815_tm_t *timeptr)
{
  uint8_t wbuf[AM0815_TIME_SIZE];
  bool ret;

  // perform time to register values transformation
  am0815_time_to_reg(timeptr, wbuf);

  // write data
  ret = am0815_write_reg(conf, AM0815_HUNDREDTHS_ADDR, AM0815_TIME_SIZE, wbuf);

  return ret;
}
/*---------------------------------------------------------------------------*/
bool
am0815_read_sram(spi_device_t *conf, uint8_t offset, uint8_t length, uint8_t *buf)
{
  // calculate SRAM address
  uint8_t addr = AM0815_CODE_READ | (AM0815_SRAM_ADDR + (offset & 0x3F));

  // read data
  return am0815_read_reg(conf, addr, length, buf);
}
/*---------------------------------------------------------------------------*/
bool
am0815_write_sram(spi_device_t *conf, uint8_t offset, uint8_t length, const uint8_t *buf)
{
  // calculate SRAM address
  uint8_t addr = AM0815_CODE_READ | (AM0815_SRAM_ADDR + (offset & 0x3F));

  // read data
  return am0815_write_reg(conf, addr, length, buf);
}
/*---------------------------------------------------------------------------*/
bool
am0815_init(spi_device_t *conf)
{
  if(am0815_open(conf) == false) {
    return false;
  }

  // set configuration registers
  am0815_set_reg(conf, AM0815_STATUS_ADDR, 0x80);

  if(am0815_close(conf) == false) {
    return false;
  }

  LOG_INFO("AM0815 init successful\n");

  return true;
}
/*---------------------------------------------------------------------------*/
uint32_t
am0815_timestamp_to_seconds(const am0815_tm_t* timestamp)
{
  // return max value on invalid timestamp
  if (timestamp->sec == 0xFF) {
    return 0xFFFFFFFF;
  }

  // calculate days for month and years (incl. leap years and leap in 2000)
  uint32_t year_month_in_days = month_days_offset[timestamp->month - 1] +
    365 * (uint32_t)timestamp->year + (uint32_t)(timestamp->year / 4) + 1;
  
  // ignore additional day for Jan/Feb dates in leap years
  if (IS_LEAP_YEAR(timestamp->year) && timestamp->month <= 2) {
    year_month_in_days = year_month_in_days - 1;
  }

  // calculate seconds
  uint32_t seconds = (uint32_t)timestamp->sec + 60 * ((uint32_t)timestamp->min +
    60 * ((uint32_t)timestamp->hour + 
    24 * ((uint32_t)timestamp->day - 1 + year_month_in_days)));

  return seconds;
}
/*---------------------------------------------------------------------------*/
