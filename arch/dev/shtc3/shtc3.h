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
