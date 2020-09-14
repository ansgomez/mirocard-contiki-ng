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
#ifndef SHTC3_H_
#define SHTC3_H_
/*---------------------------------------------------------------------------*/
#include "dev/spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define SHTC3_USE_CLOCKSTRETCH  0
#define SHTC3_USE_FAHRENHEIT    0
/*---------------------------------------------------------------------------*/

#ifndef SHTC3_I2C_CONTROLLER

#define SHTC3_I2C_CONTROLLER          0xFF /* No controller */

#define SHTC3_I2C_PIN_SCL             GPIO_HAL_PIN_UNKNOWN
#define SHTC3_I2C_PIN_SDA             GPIO_HAL_PIN_UNKNOWN

#define SHTC3_I2C_ADDRESS             0x70

#define SHTC3_INTERFACE                BOARD_I2C_INTERFACE_0

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

/**
 * SHTC3 measurement repeatibility
 */
typedef enum {
  SHTC3_REPEATABILITY_LOW,    /**< low measurement repeatablity */
  SHTC3_REPEATABILITY_MEDIUM, /**< medium measurement repeatablity */
  SHTC3_REPEATABILITY_HIGH,   /**< hihg measurement repeatablity */
} shtc3_repeatability_t;
/*---------------------------------------------------------------------------*/

/**
 * Convert the raw reading into absolute temperature.
 * \param raw The raw temperature measurements reading
 * \return The absolute temperature in 100's of degrees Celsius/Fahrenheit
 * 
 * Units used are configured using the SHTC3_USE_FAHRENHEIT header define.
 */
int32_t shtc3_convert_temperature(uint16_t raw);

/**
 * Convert the raw reading into relative humidity.
 * \param raw The raw humidity measurements reading
 * \return The relative humidity in 100's of percents
 */
int32_t shtc3_convert_humidity(uint16_t raw);

/**
 * \brief Initialise the SHTC3 sensor
 * \param conf Unused pointer for future I2C device specification
 * \return Wheter the operation succeeded
 */
bool shtc3_init(void *conf);

/**
 * \brief Wakes up the SHT3x sensor
 * \param None
 * \return Whether the operation succeeded
 */
bool shtc3_wakeup(void *conf);

/**
 * \brief Puts the SHT3x sensor in sleep mode
 * \param None
 * \return Whether the operation succeeded
 */
bool shtc3_sleep(void *conf);

/**
 * \brief Read the sensor values of the SHTC3 sensor.
 * \param conf Unused pointer for future I2C device specification
 * \param repeatibility Sensing repeatability level to use for measuring
 * \param temperature The temperatature value output buffer
 * \param humidity The humidity value output buffer
 * \return Wheter the operation succeeded
 */
bool shtc3_read_values(void *conf, shtc3_repeatability_t repeatability,
                       uint16_t *temperature, uint16_t *humidity);

/**
 * \brief Get the status of the SHTC3 sensor.
 * \param conf Unused pointer for future I2C device specification
 * \param status The status register output buffer
 * \return Wheter the operation succeeded
 */
bool shtc3_read_status(void *conf, uint16_t *status);

/**
 * \brief Clear the status of the SHTC3 sensor.
 * \param conf Unused pointer for future I2C device specification
 * \return Wheter the operation succeeded
 */
bool shtc3_clear_status(void *conf);

/**
 * \brief Soft reset the SHTC3 sensor.
 * \param conf Unused pointer for future I2C device specification
 * \return Wheter the operation succeeded
 */
bool shtc3_soft_reset(void *conf);

/*---------------------------------------------------------------------------*/
#endif /* SHTC3_H_ */
/*---------------------------------------------------------------------------*/
