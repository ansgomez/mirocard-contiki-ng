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
#include "sht3x.h"
#include "sht3x-sensor.h"

#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define SHT3X_STATE_SLEEP     			0
#define SHT3X_STATE_ACTIVE       		1
#define SHT3X_STATE_DATA_READY   		2
#define SHT3X_TYPE_TEMPERATURE      0
#define SHT3X_TYPE_HUMIDITY         1
/*---------------------------------------------------------------------------*/
static int sht3x_state = SHT3X_STATE_SLEEP;
/*---------------------------------------------------------------------------*/
static int value(int type)
{
  uint16_t temperature_raw, humidity_raw;

  sht3x_read_values(NULL, SHT3X_REPEATABILITY_LOW,
                    &temperature_raw, &humidity_raw);

  if (type == SHT3X_TYPE_TEMPERATURE) {
    return sht3x_convert_temperature(temperature_raw);
  } else {
    return sht3x_convert_humidity(humidity_raw);
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
