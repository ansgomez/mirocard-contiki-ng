/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "dev/button-hal.h"
#include "random.h"
#include "board.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>


#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#if !(CC26XX_UART_CONF_ENABLE)
#warning "running in debug configuration while serial is NOT enabled!"
#endif
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
// static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)

/*---------------------------------------------------------------------------*/
static void init_opt_reading(void *not_used);
static void init_mpu_reading(void *not_used);
/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading)
{
  if(reading < 0) {
    PRINTF("-");
    reading = -reading;
  }

  PRINTF("%d.%02d", reading / 100, reading % 100);
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  int value;
  // clock_time_t next = SENSOR_READING_PERIOD +
    // (random_rand() % SENSOR_READING_RANDOM);

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
  } else {
    PRINTF("OPT: Light Read Error\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
get_sht_reading()
{
  // read ambient sensor values
  int temperature = sht3x_sensor.value(SHT3X_TYPE_TEMPERATURE);
  int humidity = sht3x_sensor.value(SHT3X_TYPE_HUMIDITY);

  // print read sensor values
  PRINTF("SHT3X:  TEMP = % 5d [degC x 100]\n", temperature);
  PRINTF("SHT3X:  RH   = % 5d [%% x 100]\n", humidity);
}
/*---------------------------------------------------------------------------*/
static void
get_mpu_reading()
{
  int value;
  // clock_time_t next = SENSOR_READING_PERIOD +
  //   (random_rand() % SENSOR_READING_RANDOM);

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("MPU Gyro: X=");
    print_mpu_reading(value);
    PRINTF(" deg/sec\n");
  } else {
    PRINTF("Error Reading MPU Gyro X\n");
  }

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("MPU Gyro: Y=");
    print_mpu_reading(value);
    PRINTF(" deg/sec\n");
  } else {
    PRINTF("Error Reading MPU Gyro Y\n");
  }

    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("MPU Gyro: Z=");
    print_mpu_reading(value);
    PRINTF(" deg/sec\n");
  } else {
    PRINTF("Error Reading MPU Gyro Z\n");
  }

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("MPU Acc: X=");
    print_mpu_reading(value);
    PRINTF(" G\n");
  } else {
    PRINTF("Error Reading MPU Acc X\n");
  }

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("MPU Acc: Y=");
    print_mpu_reading(value);
    PRINTF(" G\n");
  } else {
    PRINTF("Error Reading MPU Acc Y\n");
  }

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    PRINTF("MPU Acc: Z=");
    print_mpu_reading(value);
    PRINTF(" G\n");
  } else {
    PRINTF("Error Reading MPU Acc Z\n");
  }

  SENSORS_DEACTIVATE(mpu_9250_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_opt_reading(void *not_used)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}
/*---------------------------------------------------------------------------*/
static void
init_sht_reading(void *not_used)
{
  // configure SHT3x sensor
  sht3x_sensor.configure(0, 0);
  get_sht_reading();
}
/*---------------------------------------------------------------------------*/
static void
get_sync_sensor_readings(void)
{
  int value;

  PRINTF("-----------------------------------------\n");

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  PRINTF("Bat: Temp=%d C\n", value);

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  PRINTF("Bat: Volt=%d mV\n", (value * 125) >> 5);

  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{
  SENSORS_ACTIVATE(batmon_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{
  init_opt_reading(NULL);
  init_mpu_reading(NULL);
  init_sht_reading(NULL);
  
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
  static int count =0;
  static uint8_t state;
  PROCESS_BEGIN();

  // check reset source for power on reset and clear flags
  state = (uint8_t)ti_lib_sys_ctrl_reset_source_get();
  PRINTF("Reset source: 0x%x\n", state);

  /****
  * NOTE: This application is meant to test the board with a normal supply.
  * This application does "not" work in batteryless mode because it needs
  * too much energy to see an LED turn on. If running in batteryless mode, this
  * will have very short LED bursts.
  */ 
#ifdef MIROCARD_BATTERYLESS
  // if not triggered by GPIO or emulated, cold start init for sleep only
  if (state != RSTSRC_WAKEUP_FROM_SHUTDOWN) {
    /*-----------------------------------------------------------------------*/
    PRINTF("Going to sleep waiting for trigger\n");
    /*-----------------------------------------------------------------------*/
    /* cold start init for sleep only */
    batteryless_shutdown();
    /*-----------------------------------------------------------------------*/
  } else {
    /* wakeup from LPM on GPIO trigger, do initialize for execution */
    PRINTF("Woken up to perform a task\n");
  }
#endif

  PRINTF("Triggering new sensor reading\n");
  init_sensors();
  get_sync_sensor_readings();
  init_sensor_readings();

  count=0;
  while(count<2) {
    PROCESS_YIELD_UNTIL((ev == sensors_event));
    if(data == &opt_3001_sensor) {
      get_light_reading();
      count++;
    } else if(data == &mpu_9250_sensor) {
      get_mpu_reading();
      count++;
    }
  }
  /*---------------------------------------------------------------------------*/
#ifdef MIROCARD_BATTERYLESS  
  PRINTF("Finished reading all sensors\n");
  batteryless_shutdown();
#endif
  /*---------------------------------------------------------------------------*/
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
