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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC13xx/CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - cc26x0-cc13x0: SmartRF06EB + CC13xx/CC26xx EM
 *   - CC2650 and CC1350 SensorTag
 *   - CC1310, CC1350, CC2650 LaunchPads
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_TRIGGER_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_TRIGGER_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
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
#include "sense_tx.h"
#include "data.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 20)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_BLUE
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#if BOARD_BATTERYLESS || BOARD_TRANSIENT
#define CC26XX_DEMO_TRIGGER_1     BOARD_BUTTON_HAL_INDEX_KEY_USER
#define CC26XX_DEMO_TRIGGER_2     BOARD_BUTTON_HAL_INDEX_KEY_USER
#else
#define CC26XX_DEMO_TRIGGER_1     BOARD_BUTTON_HAL_INDEX_KEY_LEFT
#define CC26XX_DEMO_TRIGGER_2     BOARD_BUTTON_HAL_INDEX_KEY_RIGHT
#endif

#define MPU_SENSOR_TYPE MPU_9250_SENSOR_TYPE_ACC

#define DEBUG 1
#define PRINTF(...) printf(__VA_ARGS__)

/*---------------------------------------------------------------------------*/
static uint32_t timestamp=0;
static int32_t accel[3]={0};
static uint32_t light=0;
static int32_t temp=0;
static int32_t rh=0;

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
print_mpu_reading(int16_t reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%hd", reading );
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  light = (uint32_t) opt_3001_sensor.value(0);
  if(light != CC26XX_SENSOR_READING_ERROR) {
    printf("OPT: Light=%lu lux\n", light);
  } else {
    printf("OPT: Light Read Error\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
get_sht_reading()
{
  // read ambient sensor values
  temp = shtc3_sensor.value(SHTC3_TYPE_TEMPERATURE);
  rh = shtc3_sensor.value(SHTC3_TYPE_HUMIDITY);

  // print read sensor values
  printf("SHTC3:  TEMP = % 5ld [degC x 100]\n", temp);
  printf("SHTC3:  RH   = % 5ld [%% x 100]\n", rh);
}
/*---------------------------------------------------------------------------*/
static void
get_mpu_reading()
{
  int value;

  if( (MPU_SENSOR_TYPE & MPU_9250_SENSOR_TYPE_GYRO) == MPU_9250_SENSOR_TYPE_GYRO ) {
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      printf("MPU Gyro: X=");
      print_mpu_reading(value);
      printf(" deg/sec\n");
    } else {
      printf("Error Reading MPU Gyro X\n");
    }

    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      printf("MPU Gyro: Y=");
      print_mpu_reading(value);
      printf(" deg/sec\n");
    } else {
      printf("Error Reading MPU Gyro Y\n");
    }

      value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      printf("MPU Gyro: Z=");
      print_mpu_reading(value);
      printf(" deg/sec\n");
    } else {
      printf("Error Reading MPU Gyro Z\n");
    }
  }

  if( (MPU_SENSOR_TYPE & MPU_9250_SENSOR_TYPE_ACC) == MPU_9250_SENSOR_TYPE_ACC ) { 
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      accel[0] = (uint16_t) value;
      printf("MPU Acc: X=");
      print_mpu_reading(value);
      printf(" G\n");
    } else {
      printf("Error Reading MPU Acc X\n");
    }

    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      accel[1] = (uint16_t) value;
      printf("MPU Acc: Y=");
      print_mpu_reading(value);
      printf(" G\n");
    } else {
      printf("Error Reading MPU Acc Y\n");
    }

    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      accel[2] = (uint16_t) value;
      printf("MPU Acc: Z=");
      print_mpu_reading(value);
      printf(" G\n");
    } else {
      printf("Error Reading MPU Acc Z\n");
    }
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
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_SENSOR_TYPE);
}
/*---------------------------------------------------------------------------*/
static void
init_sht_reading(void *not_used)
{
  // configure SHT3x sensor
  SENSORS_ACTIVATE(shtc3_sensor);
  get_sht_reading();
}
/*---------------------------------------------------------------------------*/
static void
get_batmon_sensor_readings(void)
{
  int value;

  SENSORS_ACTIVATE(batmon_sensor);
  printf("-----------------------------------------\n");

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  printf("Bat: Temp=%d C\n", value);

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  printf("Bat: Volt=%d mV\n", (value * 125) >> 5);

  SENSORS_DEACTIVATE(batmon_sensor);
  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{
  //Initialize all sensors
  init_opt_reading(NULL);
  init_mpu_reading(NULL);
  init_sht_reading(NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
  static int count =0;
  static batteryless_data_unit_t data_buffer;
  static uint8_t ble_payload[BLE_ADV_MAX_SIZE];
  PROCESS_BEGIN();

  printf("Reset source: 0x%x\n", (uint8_t)ti_lib_sys_ctrl_reset_source_get());

  // // if not triggered by GPIO or emulated, cold start init for sleep only
  // if (system_state.reset_source != RSTSRC_WAKEUP_FROM_SHUTDOWN) {
  //   /*-----------------------------------------------------------------------*/
  //   PRINTF("Going to sleep waiting for trigger\n");
  //   // GPIO CONFIG 1-a
  //   ti_lib_gpio_set_dio(BOARD_IOID_GPIO_4);
  //   /*-----------------------------------------------------------------------*/
  //   /* cold start init for sleep only */
  //   batteryless_shutdown();
  //   /*-----------------------------------------------------------------------*/
  // } else {
  //   /* wakeup from LPM on GPIO trigger, do initialize for execution */
  //   PRINTF("Woken up to perform a task\n");
  //   // reset default system state and task id
  //   system_state.status = 0x00;
  //   system_state.task_id = 0;
  // }

  printf("Triggering new sensor reading\n");
  //Initialize sensors
  get_batmon_sensor_readings();
  init_sensor_readings();

  count=0;
  while(count<2) {
    //SHTC3 is read within the init function
    PROCESS_YIELD_UNTIL((ev == sensors_event));
    if(data == &opt_3001_sensor) {
      get_light_reading();
      count++;
    } else if(data == &mpu_9250_sensor) {
      get_mpu_reading();
      count++;
    }
  }

  /* assemble data packet */
  timestamp = 0xABABABAB;
  uint16_t humidity_raw = (uint16_t)(rh/ 10) & 0x03FF;
  uint16_t temperature_raw = (uint16_t)(temp+ 4000) & 0x3FFF;
  data_buffer.msg_type = 0xFF; //temp for all value
  data_buffer.time = timestamp;
  data_buffer.rh_temp_data[0] = (uint8_t)(humidity_raw & 0xFF);
  data_buffer.rh_temp_data[1] = (uint8_t)(((humidity_raw >> 8) & 0x03) | ((temperature_raw & 0x3F) << 2));
  data_buffer.rh_temp_data[2] = (uint8_t)((temperature_raw >> 6) & 0xFF);
  data_buffer.light = 1000;//(uint16_t) (light/10);
  uint16_t accX_raw = 0x03FF; // (uint16_t) (accel[0] + 200);// & 0x03FF;
  uint16_t accY_raw = 0x0000;//(uint16_t) (accel[1] + 200);// & 0x03FF;
  uint16_t accZ_raw = 0x0000;//(uint16_t) (accel[2] + 200);// & 0x03FF;
  data_buffer.acc_data[0] = (uint8_t) (accX_raw & 0x00FF);
  data_buffer.acc_data[1] = (uint8_t) ( ((accX_raw & 0x0300) >> 8) | ((accY_raw & 0x003F) << 2) );
  data_buffer.acc_data[2] = (uint8_t) ( ((accY_raw & 0x03C0) >> 2) | ((accZ_raw & 0x003F) >> 2) );
  data_buffer.acc_data[3] = (uint8_t) (( (accZ_raw & 0x00FF) >> 4) );

  printf("Raw Light: %u\n",data_buffer.light);
  printf("Raw: X:%u,Y:%u,Z:%u\n",accX_raw,accY_raw,accZ_raw);

#if DEBUG
  PRINTF("sample:  ");
  for (uint8_t i = 0; i < BATTERYLESS_DATA_UNIT_SIZE; i++) {
    PRINTF("%02x ", data_buffer.bytes[i]);
  }
  PRINTF("\n");
#endif
  /*
   * Assemble manufacturer specific BLE beacon payload, see README.md for
   * detailed definition of the BLE packet structure.
   */
  uint8_t ble_payload_size = 1; // setting payload size field at the end
  ble_payload[ble_payload_size++] = BLE_ADV_TYPE_MANUFACTURER;
  ble_payload[ble_payload_size++] = BLE_ADV_TYPE_MANUFACTURER; //system_state.status;
  // TODO: add data units up to full packet size
  for (uint8_t i = 0; i < BATTERYLESS_DATA_UNIT_SIZE; i++) {
    ble_payload[ble_payload_size++] =  data_buffer.bytes[i];
  }

  // update payload size field (data length byte not counted)
  ble_payload[0] = ble_payload_size - 1;

#if DEBUG
  PRINTF("beacon payload (%u bytes):  [ ", ble_payload_size);
  for (uint8_t i = 0; i < ble_payload_size; i++) {
    if (i == 3) {
      PRINTF("|| ");
    } else if ((i - 3) % BATTERYLESS_DATA_UNIT_SIZE == 0) {
      PRINTF("| ");
    }
    PRINTF("%02x ", ble_payload[i]);
  }
  PRINTF("]\n");
#endif
  /*-------------------------------------------------------------------------*/
  /* BLE packet transmission */
  // init RF core APIs and configure TX power
  rf_core_set_modesel();
  rf_ble_set_tx_power(0);

  // while(1) 
  {
  // transmit BLE beacon
  rf_ble_beacon_single(BLE_ADV_CHANNEL_ALL, ble_payload, 31);
  }
  /*---------------------------------------------------------------------------*/
  // printf("Finished reading all sensors\n");
  // batteryless_shutdown();
  /*---------------------------------------------------------------------------*/
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
