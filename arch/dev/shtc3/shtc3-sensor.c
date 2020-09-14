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
#include "contiki.h"
#include "shtc3.h"
#include "shtc3-sensor.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#if !(CC26XX_UART_CONF_ENABLE)
#warning "running in debug configuration while serial is NOT enabled!"
#endif
#else
#define PRINTF(...)
#endif

/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, SHTC3_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()

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
    PRINTF("SHTC3: asking wrong value type\n");
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
      PRINTF("SHTC3: Unable to read new values from I2C\n");
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
    PRINTF("Enabled SHT\n");

    /* Writing CONFIG_ENABLE_SINGLE_SHOT to M bits will clear CRF bits */
    shtc3_state = SHTC3_STATE_ACTIVE;
  } else {
    /* Writing CONFIG_DISABLE to M bits will not clear CRF bits */
    shtc3_state = SHTC3_STATE_SLEEP | had_data_ready;
    shtc3_sleep(NULL);
    PRINTF("Disabled SHT\n");
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
