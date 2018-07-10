/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ext-fram.h"
#include "dev/spi.h"
#include "gpio-hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE                    "ext-fram"
#define LOG_LEVEL                     LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/
#ifndef EXT_FRAM_SPI_CONTROLLER

#define EXT_FRAM_SPI_CONTROLLER       0xFF /* No controller */

#define EXT_FRAM_SPI_PIN_SCK          GPIO_HAL_PIN_UNKNOWN
#define EXT_FRAM_SPI_PIN_MOSI         GPIO_HAL_PIN_UNKNOWN
#define EXT_FRAM_SPI_PIN_MISO         GPIO_HAL_PIN_UNKNOWN
#define EXT_FRAM_SPI_PIN_CS           GPIO_HAL_PIN_UNKNOWN

#define EXT_FRAM_PRODUCT_ID_LOW       0xFF
#define EXT_FRAM_PRODUCT_ID_HIGH      0xFF

#define EXT_FRAM_PROGRAM_PAGE_SIZE    256

#endif /* EXT_FRAM_SPI_CONTROLLER */
/*---------------------------------------------------------------------------*/
/* FRAM instruction codes */
#define FRAM_CODE_WRITE_ENABLE        0x06 /**< Write Enable */
#define FRAM_CODE_WRITE_DISABLE       0x04 /**< Write Disable */
#define FRAM_CODE_READ_STATUS         0x05 /**< Read Status Register */
#define FRAM_CODE_WRITE_STATUS        0x01 /**< Write Status Register */
#define FRAM_CODE_READ                0x03 /**< Read Data */
#define FRAM_CODE_FAST_READ           0x0B /**< Fast Read Data */
#define FRAM_CODE_WRITE               0x02 /**< Write Data */
#define FRAM_CODE_SLEEP               0xB9 /**< Enter sleep mode */
#define FRAM_CODE_READ_ID             0x9F /**< Read Device ID */
#define FRAM_CODE_READ_SERIAL         0x9F /**< Read Device Serial Number */
/*---------------------------------------------------------------------------*/
// #define VERIFY_PART_LOCKED           -2
// #define VERIFY_PART_ERROR            -1
// #define VERIFY_PART_POWERED_DOWN      0
// #define VERIFY_PART_OK                1
/*---------------------------------------------------------------------------*/
static spi_device_t fram_spi_configuration_default = {
  .spi_controller = EXT_FRAM_SPI_CONTROLLER,
  .pin_spi_sck = EXT_FRAM_SPI_PIN_SCK,
  .pin_spi_miso = EXT_FRAM_SPI_PIN_MISO,
  .pin_spi_mosi = EXT_FRAM_SPI_PIN_MOSI,
  .pin_spi_cs = EXT_FRAM_SPI_PIN_CS,
  .spi_bit_rate = 4000000,
  .spi_pha = 0,
  .spi_pol = 0,
};
/*---------------------------------------------------------------------------*/
/**
 * Get spi configuration, return default configuration if NULL
 */
static spi_device_t *
get_spi_conf(spi_device_t *conf)
{
  if(conf == NULL) {
    return &fram_spi_configuration_default;
  }
  return conf;
}
/*---------------------------------------------------------------------------*/
/**
 * Clear external flash CSN line
 */
static bool
select_on_bus(spi_device_t *spi_config)
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
 * Set external flash CSN line
 */
static void
deselect(spi_device_t *spi_config)
{
  spi_deselect(spi_config);
  LOG_DBG("SPI deselected\n");
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Verify the flash part.
 * \retval VERIFY_PART_OK The part was identified successfully
 * \retval VERIFY_PART_ERROR There was an error communicating with the part
 * \retval VERIFY_PART_POWERED_DOWN Communication was successful, but the part
 *         was powered down
 */
// static uint8_t
// verify_part(spi_device_t *spi_config)
// {
//   const uint8_t wbuf[] = { FRAM_CODE_READ_ID };
//   uint8_t rbuf[8] = { 0 };
//   bool ret;

//   if(select_on_bus(spi_config) == false) {
//     return VERIFY_PART_LOCKED;
//   }

//   if(spi_write(spi_config, wbuf, sizeof(wbuf)) != SPI_DEV_STATUS_OK) {
//     deselect(spi_config);
//     return VERIFY_PART_ERROR;
//   }

//   ret = spi_read(spi_config, rbuf, sizeof(rbuf));
//   deselect(spi_config);
//   if(ret != SPI_DEV_STATUS_OK) {
//     return VERIFY_PART_ERROR;
//   }

//   LOG_DBG("Verify: %02x %02x\n", rbuf[0], rbuf[1]);

//   if(rbuf[0] != EXT_FRAM_PRODUCT_ID_LOW || 
//       rbuf[1] != EXT_FRAM_PRODUCT_ID_HIGH) {
//     return VERIFY_PART_POWERED_DOWN;
//   }
//   return VERIFY_PART_OK;
// }
/*---------------------------------------------------------------------------*/
/**
 * \brief Put the device in sleep mode.
 */
static bool
power_sleep(spi_device_t *spi_config)
{
  uint8_t cmd;
  bool success;

  cmd = FRAM_CODE_SLEEP;
  if(select_on_bus(spi_config) == false) {
    return false;
  }

  success = (spi_write_byte(spi_config, cmd) == SPI_DEV_STATUS_OK);

  deselect(spi_config);

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Enable write.
 * \return True when successful.
 */
static bool
write_enable(spi_device_t *spi_config)
{
  bool ret;
  const uint8_t wbuf[] = { FRAM_CODE_WRITE_ENABLE };

  if(select_on_bus(spi_config) == false) {
    return false;
  }

  ret = (spi_write(spi_config, wbuf, sizeof(wbuf)) == SPI_DEV_STATUS_OK);
  deselect(spi_config);

  if(ret == false) {
    LOG_DBG("write enable FAILED\n");
    return false;
  }
  LOG_DBG("write enabled\n");
  return true;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_open(spi_device_t *conf)
{
  spi_device_t *spi_config = get_spi_conf(conf);

  /* Check if platform has ext-fram */
  if(spi_config->pin_spi_sck == GPIO_HAL_PIN_UNKNOWN) {
    LOG_WARN("FRAM not present in platform!\n");
    return false;
  }

  if(spi_acquire(spi_config) != SPI_DEV_STATUS_OK) {
    LOG_DBG("SPI write acquire FAILED\n");
    return false;
  }

  /* Toggle CS to wakeup device */
  if(select_on_bus(spi_config) == false) {
    LOG_DBG("FRAM wakeup FAILED\n");
    return false;
  }
  deselect(spi_config);

  return true;

  // if(verify_part(spi_config) == VERIFY_PART_OK) {
  //   return true;
  // }

  // /* Failed to verify */
  // spi_release(spi_config);
  // return false;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_close(spi_device_t *conf)
{
  bool ret;
  spi_device_t *spi_config = get_spi_conf(conf);

  /* Put the part in low power mode */
  ret = power_sleep(spi_config);

  /* SPI is released no matter if power_down() succeeds or fails */
  if(spi_release(spi_config) != SPI_DEV_STATUS_OK) {
    return false;
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_read(spi_device_t *conf, uint32_t offset, uint32_t length, uint8_t *buf)
{
  uint8_t wbuf[4];
  bool ret;

  spi_device_t *spi_config;

  spi_config = get_spi_conf(conf);

  // /* Wait till previous erase/program operation completes */
  // if(wait_ready(spi_config) == false) {
  //   return false;
  // }

  wbuf[0] = FRAM_CODE_READ;
  wbuf[1] = (offset >> 16) & 0xff;
  wbuf[2] = (offset >> 8) & 0xff;
  wbuf[3] = offset & 0xff;

  if(select_on_bus(spi_config) == false) {
    return false;
  }

  if(spi_write(spi_config, wbuf, sizeof(wbuf)) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(spi_config);
    return false;
  }

  ret = (spi_read(spi_config, buf, length) == SPI_DEV_STATUS_OK);

  deselect(spi_config);

  return ret;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_write(spi_device_t *conf, uint32_t offset, uint32_t length, const uint8_t *buf)
{
  uint8_t wbuf[4];

  spi_device_t *spi_config = get_spi_conf(conf);

  // /* Wait till previous erase/program operation completes */
  // if(wait_ready(spi_config) == false) {
  //   return false;
  // }

  if(write_enable(spi_config) == false) {
    return false;
  }

  wbuf[0] = FRAM_CODE_WRITE;
  wbuf[1] = (offset >> 16) & 0xff;
  wbuf[2] = (offset >> 8) & 0xff;
  wbuf[3] = offset & 0xff;

  if(select_on_bus(spi_config) == false) {
    return false;
  }

  LOG_DBG("set write address\n");
  if(spi_write(spi_config, wbuf, sizeof(wbuf)) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(spi_config);
    return false;
  }

  LOG_DBG("write data\n");
  if(spi_write(spi_config, buf, length) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(spi_config);
    return false;
  }

  deselect(spi_config);

  return true;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_erase(spi_device_t *conf, uint32_t offset, uint32_t length)
{
  /* USE WRITE FUNCTION INSTEAD OF ERASE */
  return false;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_init(spi_device_t *conf)
{
  if(ext_fram_open(conf) == false) {
    return false;
  }

  if(ext_fram_close(conf) == false) {
    return false;
  }

  LOG_INFO("FRAM init successful\n");

  return true;
}
/*---------------------------------------------------------------------------*/
