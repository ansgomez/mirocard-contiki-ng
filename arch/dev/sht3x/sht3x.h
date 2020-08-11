/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
 * Copyright (c) 2020, Miromico AG - http://www.miromico.ch/
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
#ifndef SHT3X_H_
#define SHT3X_H_
/*---------------------------------------------------------------------------*/
#include "dev/spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define SHT3X_USE_CLOCKSTRETCH  0
#define SHT3X_USE_FAHRENHEIT    0
/*---------------------------------------------------------------------------*/
/**
 * SHT3x measurement repeatibility
 */
typedef enum {
  SHT3X_REPEATABILITY_LOW,    /**< low measurement repeatablity */
  SHT3X_REPEATABILITY_MEDIUM, /**< medium measurement repeatablity */
  SHT3X_REPEATABILITY_HIGH,   /**< hihg measurement repeatablity */
} sht3x_repeatability_t;
/*---------------------------------------------------------------------------*/

/**
 * Convert the raw reading into absolute temperature.
 * \param raw The raw temperature measurements reading
 * \return The absolute temperature in 100's of degrees Celsius/Fahrenheit
 * 
 * Units used are configured using the SHT3X_USE_FAHRENHEIT header define.
 */
int32_t sht3x_convert_temperature(uint16_t raw);

/**
 * Convert the raw reading into relative humidity.
 * \param raw The raw humidity measurements reading
 * \return The relative humidity in 100's of percents
 */
int32_t sht3x_convert_humidity(uint16_t raw);

/**
 * \brief Initialise the SHT3x sensor
 * \param conf Unused pointer for future I2C device specification
 * \return Whether the operation succeeded
 */
bool sht3x_init(void *conf);

/**
 * \brief Read the sensor values of the SHT3x sensor.
 * \param conf Unused pointer for future I2C device specification
 * \param repeatibility Sensing repeatability level to use for measuring
 * \param temperature The temperatature value output buffer
 * \param humidity The humidity value output buffer
 * \return Wheter the operation succeeded
 */
bool sht3x_read_values(void *conf, sht3x_repeatability_t repeatability,
                       uint16_t *temperature, uint16_t *humidity);

/**
 * \brief Get the status of the SHT3x sensor.
 * \param conf Unused pointer for future I2C device specification
 * \param status The status register output buffer
 * \return Wheter the operation succeeded
 */
bool sht3x_read_status(void *conf, uint16_t *status);

/**
 * \brief Clear the status of the SHT3x sensor.
 * \param conf Unused pointer for future I2C device specification
 * \return Wheter the operation succeeded
 */
bool sht3x_clear_status(void *conf);

/**
 * \brief Soft reset the SHT3x sensor.
 * \param conf Unused pointer for future I2C device specification
 * \return Wheter the operation succeeded
 */
bool sht3x_soft_reset(void *conf);

/*---------------------------------------------------------------------------*/
#endif /* SHT3X_H_ */
/*---------------------------------------------------------------------------*/
