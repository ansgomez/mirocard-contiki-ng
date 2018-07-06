/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ext-fram.h"
#include "dev/spi.h"
#include "gpio-hal.h"
#include "sys/log.h"

#include <stdint.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
#ifndef EXT_FRAM_SPI_CONTROLLER

#define EXT_FRAM_SPI_CONTROLLER      0xFF /* No controller */

#define EXT_FRAM_SPI_PIN_SCK         GPIO_HAL_PIN_UNKNOWN
#define EXT_FRAM_SPI_PIN_MOSI        GPIO_HAL_PIN_UNKNOWN
#define EXT_FRAM_SPI_PIN_MISO        GPIO_HAL_PIN_UNKNOWN
#define EXT_FRAM_SPI_PIN_CS          GPIO_HAL_PIN_UNKNOWN

#define EXT_FRAM_PRODUCT_ID_LOW      0xFF
#define EXT_FRAM_PRODUCT_ID_HIGH     0xFF

#define EXT_FRAM_PROGRAM_PAGE_SIZE   256

#endif /* EXT_FRAM_SPI_CONTROLLER */
/*---------------------------------------------------------------------------*/
/* Log configuration */
#define LOG_MODULE "ext-fram"
#define LOG_LEVEL LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/
/* FRAM instruction codes */
#define FRAM_CODE_READ             0x03 /**< Read Data */
#define FRAM_CODE_FAST_READ        0x0B /**< Fast Read Data */
#define FRAM_CODE_READ_STATUS      0x05 /**< Read Status Register */
#define FRAM_CODE_WRITE_STATUS     0x01 /**< Write Status Register */
#define FRAM_CODE_WRITE_ENABLE     0x06 /**< Write Enable */
#define FRAM_CODE_WRITE            0x02 /**< Write Data */
#define FRAM_CODE_READ_ID          0x9F /**< Read Device ID */

#define FRAM_CODE_SLEEP            0xB9 /**< Enter sleep mode */
/*---------------------------------------------------------------------------*/
#define VERIFY_PART_LOCKED          -2
#define VERIFY_PART_ERROR           -1
#define VERIFY_PART_POWERED_DOWN     0
#define VERIFY_PART_OK               1
/*---------------------------------------------------------------------------*/
static spi_device_t fram_spi_configuration_default = {
  .spi_controller = EXT_FRAM_SPI_CONTROLLER,
  .pin_spi_sck = EXT_FRAM_SPI_PIN_SCK,
  .pin_spi_miso = EXT_FRAM_SPI_PIN_MISO,
  .pin_spi_mosi = EXT_FRAM_SPI_PIN_MOSI,
  .pin_spi_cs = EXT_FRAM_SPI_PIN_CS,
  .spi_bit_rate = 4000000,
  .spi_pha = 0,
  .spi_pol = 0
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
select_on_bus(spi_device_t *fram_spi_configuration)
{
  if(spi_select(fram_spi_configuration) == SPI_DEV_STATUS_OK) {
    return true;
  }
  return false;
}
/*---------------------------------------------------------------------------*/
/**
 * Set external flash CSN line
 */
static void
deselect(spi_device_t *fram_spi_configuration)
{
  spi_deselect(fram_spi_configuration);
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
// verify_part(spi_device_t *fram_spi_configuration)
// {
//   const uint8_t wbuf[] = { FRAM_CODE_READ_ID };
//   uint8_t rbuf[8] = { 0 };
//   bool ret;

//   if(select_on_bus(fram_spi_configuration) == false) {
//     return VERIFY_PART_LOCKED;
//   }

//   if(spi_write(fram_spi_configuration, wbuf, sizeof(wbuf)) != SPI_DEV_STATUS_OK) {
//     deselect(fram_spi_configuration);
//     return VERIFY_PART_ERROR;
//   }

//   ret = spi_read(fram_spi_configuration, rbuf, sizeof(rbuf));
//   deselect(fram_spi_configuration);
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
power_sleep(spi_device_t *fram_spi_configuration)
{
  uint8_t cmd;
  bool success;

  cmd = FRAM_CODE_SLEEP;
  if(select_on_bus(fram_spi_configuration) == false) {
    return false;
  }

  success = (spi_write_byte(fram_spi_configuration, cmd) == SPI_DEV_STATUS_OK);

  deselect(fram_spi_configuration);

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Enable write.
 * \return True when successful.
 */
static bool
write_enable(spi_device_t *fram_spi_configuration)
{
  bool ret;
  const uint8_t wbuf[] = { FRAM_CODE_WRITE_ENABLE };

  if(select_on_bus(fram_spi_configuration) == false) {
    return false;
  }

  ret = (spi_write(fram_spi_configuration, wbuf, sizeof(wbuf)) == SPI_DEV_STATUS_OK);
  deselect(fram_spi_configuration);

  if(ret == false) {
    return false;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_open(spi_device_t *conf)
{
  spi_device_t *fram_spi_configuration;

  fram_spi_configuration = get_spi_conf(conf);

  /* Check if platform has ext-fram */
  if(fram_spi_configuration->pin_spi_sck == GPIO_HAL_PIN_UNKNOWN) {
    return false;
  }

  if(spi_acquire(fram_spi_configuration) != SPI_DEV_STATUS_OK) {
    return false;
  }
  /* Default output to clear chip select */
  deselect(fram_spi_configuration);

  /* Put the part is standby mode */
  power_sleep(fram_spi_configuration);

  return true;
  // if(verify_part(fram_spi_configuration) == VERIFY_PART_OK) {
  //   return true;
  // }

  // /* Failed to verify */
  // spi_release(fram_spi_configuration);
  // return false;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_close(spi_device_t *conf)
{
  bool ret;
  spi_device_t *fram_spi_configuration;

  fram_spi_configuration = get_spi_conf(conf);

  /* Put the part in low power mode */
  ret = power_sleep(fram_spi_configuration);

  /* SPI is released no matter if power_down() succeeds or fails */
  if(spi_release(fram_spi_configuration) != SPI_DEV_STATUS_OK) {
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

  spi_device_t *fram_spi_configuration;

  fram_spi_configuration = get_spi_conf(conf);

  // /* Wait till previous erase/program operation completes */
  // if(wait_ready(fram_spi_configuration) == false) {
  //   return false;
  // }

  /*
   * SPI is driven with very low frequency (1MHz < 33MHz fR spec)
   * in this implementation, hence it is not necessary to use fast read.
   */
  wbuf[0] = FRAM_CODE_READ;
  wbuf[1] = (offset >> 16) & 0xff;
  wbuf[2] = (offset >> 8) & 0xff;
  wbuf[3] = offset & 0xff;

  if(select_on_bus(fram_spi_configuration) == false) {
    return false;
  }

  if(spi_write(fram_spi_configuration, wbuf, sizeof(wbuf)) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(fram_spi_configuration);
    return false;
  }

  ret = (spi_read(fram_spi_configuration, buf, length) == SPI_DEV_STATUS_OK);

  deselect(fram_spi_configuration);

  return ret;
}
/*---------------------------------------------------------------------------*/
bool
ext_fram_write(spi_device_t *conf, uint32_t offset, uint32_t length, const uint8_t *buf)
{
  uint8_t wbuf[4];
  uint32_t ilen; /* interim length per instruction */

  spi_device_t *fram_spi_configuration;

  fram_spi_configuration = get_spi_conf(conf);

  // /* Wait till previous erase/program operation completes */
  // if(wait_ready(fram_spi_configuration) == false) {
  //   return false;
  // }

  if(write_enable(fram_spi_configuration) == false) {
    return false;
  }

  ilen = EXT_FRAM_PROGRAM_PAGE_SIZE - (offset % EXT_FRAM_PROGRAM_PAGE_SIZE);
  if(length < ilen) {
    ilen = length;
  }

  wbuf[0] = FRAM_CODE_WRITE;
  wbuf[1] = (offset >> 16) & 0xff;
  wbuf[2] = (offset >> 8) & 0xff;
  wbuf[3] = offset & 0xff;

  offset += ilen;
  length -= ilen;

  if(select_on_bus(fram_spi_configuration) == false) {
    return false;
  }

  if(spi_write(fram_spi_configuration, wbuf, sizeof(wbuf)) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(fram_spi_configuration);
    return false;
  }

  if(spi_write(fram_spi_configuration, buf, ilen) != SPI_DEV_STATUS_OK) {
    /* failure */
    deselect(fram_spi_configuration);
    return false;
  }
  buf += ilen;
  deselect(fram_spi_configuration);

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
