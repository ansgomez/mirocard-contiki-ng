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
#ifndef AM0815_H_
#define AM0815_H_
/*---------------------------------------------------------------------------*/
#include "dev/spi.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
/**
 * AM0815 operation mode definitions
 */
typedef enum {
  AM0815_MODE_STOP, /**< stop the RTC clocks */
  AM0815_MODE_CRYSTAL, /**< crystal oscillator (default mode after reset) */
  AM0815_MODE_AUTOCALIBRATE_9MIN, /**< RC oscillator with XT auto calibration every 512 sec */
  AM0815_MODE_AUTOCALIBRATE_17MIN, /**< RC oscillator with XT auto calibration every 1024 sec */
  AM0815_MODE_RC_ONLY,     /**< RC oscillator only */
  AM0815_MODE_RC_BACKUP,   /**< crystal oscillator with fallback to RC oscillator when on VBAT supply */
} am0815_mode_t;

/**
 * Date time structure to work with the RTC
 */
typedef struct am0815_tm {
  uint8_t split;   /**< hunderedths split seconds after seconds (0-99) */
  uint8_t sec;     /**< seconds after minutes (0-59)*/
  uint8_t min;     /**< minutes after hours (0-59) */
  uint8_t hour;    /**< hours after midnight (0-23) */
  uint8_t day;     /**< day of the month (1-31)*/
  uint8_t month;   /**< month of the year (1-12) */
  uint8_t year;    /**< year after 2000 (0-99) */
} am0815_tm_t;
/*---------------------------------------------------------------------------*/
/**
 * Invalid timestamp definition
 */
#define AM0815_TIMESTAMP_INVALID                                              \
  ((am0815_tm_t) { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF })
/**
 * Test wether year is a leap year
 */
#define IS_LEAP_YEAR(y)                                                       \
  ( (((y) % 4 == 0) && ((y) % 100 != 0)) || ((y) % 400 == 0) )
/*---------------------------------------------------------------------------*/
/**
 * Day offsets for months of the year (for no-leap years)
 */
static const uint16_t month_days_offset[] =
  {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize AM0815 RTC driver.
 * \param conf SPI bus configuration struct. NULL for default.
 * \return True when successful.
 */
bool am0815_open(spi_device_t *conf);

/**
 * \brief Close the AM0815 RTC driver
 * \param conf SPI bus configuration struct. NULL for default.
 * \return True when successful.
 *
 * This call will put the device in its lower power mode (power down).
 */
bool am0815_close(spi_device_t *conf);

/**
 * \brief Set operation mode of the AM0815 RTC
 * \param conf SPI bus configuration struct. NULL for default.
 * \param mode The operation mode to configure.
 * \return True when successful.
 */
bool am0815_set_mode(spi_device_t *conf, am0815_mode_t mode);

/**
 * \brief Get the current data/time value of AM0815 RTC
 * \param conf SPI bus configuration struct. NULL for default.
 * \param timeptr Pointer to date/time to to write the current RTC value to.
 * \return True when successful.
 * 
 * The output date/time structure has to be allocated by the caller.
 */
bool am0815_get_time(spi_device_t *conf, am0815_tm_t *timeptr);

/**
 * \brief Set the time of AM0815 RTC
 * \param conf SPI bus configuration struct. NULL for default.
 * \param timeptr Pointer to date/time to set the RTC value to. In case of an
 *                error it is set to `AM0815_TIMESTAMP_INVALID`.
 * \return True when successful.
 */
bool am0815_set_time(spi_device_t *conf, const am0815_tm_t *timeptr);

/**
 * \brief Read the integrated SRAM.
 * \param conf SPI bus configuration struct. NULL for default.
 * \param offset Address to read from
 * \param length Number of bytes to read
 * \param buf Buffer where to store the read bytes
 * \return True when successful.
 *
 * buf must be allocated by the caller
 */
bool am0815_read_sram(spi_device_t *conf, uint8_t offset, uint8_t length, uint8_t *buf);

/**
 * \brief Write to the integrated SRAM.
 * \param conf SPI bus configuration struct. NULL for default.
 * \param offset Address to write to
 * \param length Number of bytes to write
 * \param buf Buffer holding the bytes to be written
 *
 * \return True when successful.
 */
bool am0815_write_sram(spi_device_t *conf, uint8_t offset, uint8_t length, const uint8_t *buf);

/**
 * \brief Initialise the AM0815 RTC
 * \param conf SPI bus configuration struct. NULL for default.
 *
 */
bool am0815_init(spi_device_t *conf);

/**
 * \brief Convert the AM0815 date time structure to seconds.
 * \param timestamp AM0815 date time structure to convert.
 * \return Seconds since Jan 1st, 2000.
 */
uint32_t am0815_timestamp_to_seconds(const am0815_tm_t* timestamp);

/*---------------------------------------------------------------------------*/
#endif /* AM0815_H_ */
/*---------------------------------------------------------------------------*/
