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
#include "shtc3.h"
#include "shtc3-sensor.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define SHTC3_STATE_SLEEP     			0
#define SHTC3_STATE_ACTIVE       		1
#define SHTC3_STATE_DATA_READY   		2
/*---------------------------------------------------------------------------*/
static int shtc3_state = SHTC3_STATE_SLEEP;
static bool shtc3_values_updated[2] = {false, false};
static uint16_t shtc3_raw_values[2] = {0, 0};
/*---------------------------------------------------------------------------*/
static int value(int type)
{
  // return most negative interger on invalid type
  if (type != SHTC3_TYPE_TEMPERATURE && type != SHTC3_TYPE_HUMIDITY) {
    printf("SHTC3: asking wrong value type\n");
    return INT_MIN;
  }
  
  // read new values if no update value is buffered
  if (!shtc3_values_updated[type]) {
    if (shtc3_read_values(NULL, SHTC3_REPEATABILITY_LOW,
                          &shtc3_raw_values[SHTC3_TYPE_TEMPERATURE],
                          &shtc3_raw_values[SHTC3_TYPE_HUMIDITY])) {
      // set updated flags on success
      shtc3_values_updated[SHTC3_TYPE_TEMPERATURE] = true;
      shtc3_values_updated[SHTC3_TYPE_HUMIDITY] = true;
    }
    else {
      printf("SHTC3: Unable to read new values from I2C\n");
    }
  }

  // check if updated value available
  if (type == SHTC3_TYPE_TEMPERATURE &&
              shtc3_values_updated[SHTC3_TYPE_TEMPERATURE]) {
    shtc3_values_updated[SHTC3_TYPE_TEMPERATURE] = false;
    return shtc3_convert_temperature(shtc3_raw_values[SHTC3_TYPE_TEMPERATURE]);
  } else if (type == SHTC3_TYPE_HUMIDITY && 
              shtc3_values_updated[SHTC3_TYPE_HUMIDITY]) {
    shtc3_values_updated[SHTC3_TYPE_HUMIDITY] = false;
    return shtc3_convert_humidity(shtc3_raw_values[SHTC3_TYPE_HUMIDITY]);
  } else {
    // on error updating values, return most negative interger
    return INT_MIN;
  }
}
/*---------------------------------------------------------------------------*/
/** 
 * \brief Turn the sensor on/off
 * \param enable TRUE: on, FALSE: off
 */
static void
enable_sensor(bool enable)
{
  uint16_t had_data_ready = shtc3_state & SHTC3_STATE_DATA_READY;

  if(enable) {
    // shtc3_wakeup(NULL);
    shtc3_init(NULL);
    printf("Enabled SHT\n");

    /* Writing CONFIG_ENABLE_SINGLE_SHOT to M bits will clear CRF bits */
    shtc3_state = SHTC3_STATE_ACTIVE;
  } else {
    /* Writing CONFIG_DISABLE to M bits will not clear CRF bits */
    shtc3_state = SHTC3_STATE_SLEEP | had_data_ready;
    shtc3_sleep(NULL);
    printf("Disabled SHT\n");
  }

}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int enable)
{
  int rv = 0;

  switch(type) {
  case SENSORS_HW_INIT:
    /*
     * Device reset won't reset the sensor, so we put it to sleep here
     * explicitly
     */
    enable_sensor(0);
    rv = 0;
    break;
  case SENSORS_ACTIVE:
    if(enable) {
      enable_sensor(1);
      rv = 1;
    } else {
      enable_sensor(0);
      rv = 0;
    }
    break;
  default:
    break;
  }
  return rv;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  return shtc3_state;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(shtc3_sensor, "SHTC3", value, configure, status);
/*---------------------------------------------------------------------------*/