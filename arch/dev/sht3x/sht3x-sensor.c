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
#include "contiki.h"
#include "sht3x.h"
#include "sht3x-sensor.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define SHT3X_STATE_SLEEP     			0
#define SHT3X_STATE_ACTIVE       		1
#define SHT3X_STATE_DATA_READY   		2
/*---------------------------------------------------------------------------*/
static int sht3x_state = SHT3X_STATE_SLEEP;
static bool sht3x_values_updated[2] = {false, false};
static uint16_t sht3x_raw_values[2] = {0, 0};
/*---------------------------------------------------------------------------*/
static int value(int type)
{
  // return most negative interger on invalid type
  if (type != SHT3X_TYPE_TEMPERATURE && type != SHT3X_TYPE_HUMIDITY) {
    return INT_MIN;
  }
  
  // read new values if no update value is buffered
  if (!sht3x_values_updated[type]) {
    if (sht3x_read_values(NULL, SHT3X_REPEATABILITY_LOW,
                          &sht3x_raw_values[SHT3X_TYPE_TEMPERATURE],
                          &sht3x_raw_values[SHT3X_TYPE_HUMIDITY])) {
      // set updated flags on success
      sht3x_values_updated[SHT3X_TYPE_TEMPERATURE] = true;
      sht3x_values_updated[SHT3X_TYPE_HUMIDITY] = true;
    }
  }

  // check if updated value available
  if (type == SHT3X_TYPE_TEMPERATURE &&
      sht3x_values_updated[SHT3X_TYPE_TEMPERATURE]) {
    sht3x_values_updated[SHT3X_TYPE_TEMPERATURE] = false;
    return sht3x_convert_temperature(sht3x_raw_values[SHT3X_TYPE_TEMPERATURE]);
  } else if (type == SHT3X_TYPE_HUMIDITY && 
             sht3x_values_updated[SHT3X_TYPE_HUMIDITY]) {
    sht3x_values_updated[SHT3X_TYPE_HUMIDITY] = false;
    return sht3x_convert_humidity(sht3x_raw_values[SHT3X_TYPE_HUMIDITY]);
  } else {
    // on error updating values, return most negative interger
    return INT_MIN;
  }
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int enable)
{
  return sht3x_init(NULL);
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  return sht3x_state;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(sht3x_sensor, "SHT3x", value, configure, status);
/*---------------------------------------------------------------------------*/
