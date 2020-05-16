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

#include "ti-lib.h"

#include "sense-transmit.h"
#include "data.h"

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
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
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(batteryless_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&batteryless_process);
/*---------------------------------------------------------------------------*/
static void init_opt_reading(void *not_used);
static void init_mpu_reading(void *not_used);
/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  int value;

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
  } else {
    printf("OPT: Light Read Error\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
get_mpu_reading()
{
  int value;

  printf("MPU Gyro: X=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  print_mpu_reading(value);
  printf(" deg/sec\n");

  printf("MPU Gyro: Y=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  print_mpu_reading(value);
  printf(" deg/sec\n");

  printf("MPU Gyro: Z=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  print_mpu_reading(value);
  printf(" deg/sec\n");

  printf("MPU Acc: X=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Acc: Y=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Acc: Z=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  print_mpu_reading(value);
  printf(" G\n");

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
get_sync_sensor_readings(void)
{
  int value;

  printf("-----------------------------------------\n");

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  printf("Bat: Temp=%d C\n", value);

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  printf("Bat: Volt=%d mV\n", (value * 125) >> 5);

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
  SENSORS_ACTIVATE(opt_3001_sensor);
  init_mpu_reading(NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(batteryless_process, ev, data)
{
  static int count =0;
  static batteryless_system_state_t system_state_old;
  static batteryless_system_state_t system_state;
  static batteryless_data_unit_t data_buffer;
  static uint8_t ble_payload[BLE_ADV_MAX_SIZE];
  static uint32_t timestamp;
  static int accel[3];
  static uint16_t light;
  // static int temperature;
  // static int humidity;
  // static bool ret;
  PROCESS_BEGIN();
  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1
  // ti_lib_gpio_set_dio(BOARD_IOID_GPIO_1);
  /*-------------------------------------------------------------------------*/
 
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

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+2
  // ti_lib_gpio_set_dio(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/
  printf("Triggering new sensor reading\n");
  // get_sync_sensor_readings();
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
  printf("Finished reading all sensors\n");
  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 3
  // ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_1);
  /*-------------------------------------------------------------------------*/
  /* assemble data packet */
  data_buffer.time = timestamp;
  data_buffer.accel[0] = accel[0];
  data_buffer.accel[1] = accel[1];
  data_buffer.accel[2] = accel[2];
  data_buffer.light = light;
#if DEBUG
  PRINTF("sample:  ");
  for (uint8_t i = 0; i < BATTERYLESS_DATA_UNIT_SIZE; i++) {
    PRINTF("%5u ", data_buffer.bytes[i]);
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
  // add data units up to full packet size
  while (ble_payload_size + BATTERYLESS_DATA_UNIT_SIZE <= BLE_ADV_MAX_SIZE ) {
    uint8_t* data_bytes = (uint8_t*)&data_buffer;
    for (uint8_t i = 0; i < BATTERYLESS_DATA_UNIT_SIZE; i++) {
      ble_payload[ble_payload_size++] = data_bytes[i];
    }
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
  rf_ble_set_tx_power(BLE_RF_TX_POWER);

  // transmit BLE beacon
  rf_ble_beacon_single(BLE_ADV_CHANNEL_ALL, ble_payload, ble_payload_size);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
