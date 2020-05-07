/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich)
 * Copyright (c) 2020, Miromico AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
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
/* ------------------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "contiki.h"
#include "lpm.h"
#include "ti-lib.h"
#include "rf-core/rf-ble.h"
#include "board-peripherals.h"

#include "data.h"

#include "batteryless.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#if !(CC26XX_UART_CONF_ENABLE)
#warning "running in debug configuration while serial is NOT enabled!"
#endif
#else
#define PRINTF(...)
#endif
/* ------------------------------------------------------------------------- */
/**
 * Set peripherals to sleep, configure EMU and enter deep sleep.
 * @note This function enters deep sleep and does NOT return.
 * @param emu_energy EMU energy burst size for wakeup
 * @param emu_voltage Voltage level for next EMU energy burst
 */
static inline void batteryless_shutdown() {

  // LPM with GPIO triggered wakeup
  lpm_shutdown(WAKEUP_TRIGGER_IOID, IOC_NO_IOPULL, WAKEUP_TRIGGER_EDGE);
}
/* ------------------------------------------------------------------------- */
PROCESS(transient_app_process, "Transient application process");
AUTOSTART_PROCESSES(&transient_app_process);
/* ------------------------------------------------------------------------- */
PROCESS_THREAD(transient_app_process, ev, data) {
  static transient_system_state_t system_state_old;
  static transient_system_state_t system_state;
  static batteryless_data_unit_t data_buffer;
  static uint8_t ble_payload[BLE_ADV_MAX_SIZE];
  static uint32_t timestamp;
  // static int temperature;
  // static int humidity;
  // static bool ret;
  /*-------------------------------------------------------------------------*/
  PROCESS_BEGIN();
  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_1);
  /*-------------------------------------------------------------------------*/
  // check reset source for power on reset and clear flags
  system_state.reset_source = ti_lib_sys_ctrl_reset_source_get();
  PRINTF("Reset source: 0x%x\n", (uint8_t)system_state.reset_source);

  // if not triggered by GPIO or emulated, cold start init for sleep only
  if (system_state.reset_source != RSTSRC_WAKEUP_FROM_SHUTDOWN) {
    /*-----------------------------------------------------------------------*/
    // GPIO CONFIG 1-a
    ti_lib_gpio_set_dio(BOARD_IOID_GPIO_4);
    /*-----------------------------------------------------------------------*/
    /* cold start init for sleep only */
    batteryless_shutdown();
    /*-----------------------------------------------------------------------*/
  } else {
    /* wakeup from LPM on GPIO trigger, do initialize for execution */

    // reset default system state and task id
    system_state.status = 0x00;
    system_state.task_id = 0;
  }

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+2
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/

  // set task_id counter
  system_state.task_id = system_state_old.task_id + 1;

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+2+3
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_3);
  /*-------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------*/
  /* Time based system state updates */

  // store (potentially invalid) timestamp in system state
  system_state.activation_time = clock_time()/CLOCK_SECOND;

  // check wether timer was reset since last system activation
  if (system_state.activation_time < system_state_old.activation_time) {
    system_state.status |= SYSTEM_STATUS_TIMER_RESET;
  }

  // estimate the power level from the timer value
  if (system_state.status & SYSTEM_STATUS_TIMER_RESET) {
    // no valid time for last activation (clock reset): estimate lowest power
    system_state.power_level = 0;
  } else {
    uint32_t activation_interval = system_state.activation_time - 
                                   system_state_old.activation_time;
    //TODO: policy_estimate_power_level(activation_interval);
    system_state.power_level =  activation_interval;
  }

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+3
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/
  /* Sensor readout */


  //TODO: Update sensor readings
  // //configure SHT3x sensor
  // sht3x_sensor.configure(0, 0);
  // //read ambient sensor values
  // temperature = sht3x_sensor.value(SHT3X_TYPE_TEMPERATURE);
  // humidity = sht3x_sensor.value(SHT3X_TYPE_HUMIDITY);
  timestamp = clock_time();

  // print read sensor values
  // PRINTF("SHT31:  TEMP = % 5d [degC x 100]\n", temperature);
  // PRINTF("SHT31:  RH   = % 5d [%% x 100]\n", humidity);

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 3
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_1);
  /*-------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2+3
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/
  /* assemble data packet */
  data_buffer.time = timestamp;
  data_buffer.data[0] = 0xFF;
  data_buffer.data[1] = 0xFF;
  data_buffer.data[2] = 0xFF;
   
#if DEBUG
  PRINTF("sample:  ");
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
    PRINTF("%5u ", data_buffer.bytes[i]);
  }
  PRINTF("\n");
#endif

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_3);
  /*-------------------------------------------------------------------------*/

  /*
   * Assemble manufacturer specific BLE beacon payload, see README.md for
   * detailed definition of the BLE packet structure.
   */
  uint8_t ble_payload_size = 1; // setting payload size field at the end
  ble_payload[ble_payload_size++] = BLE_ADV_TYPE_MANUFACTURER;
  ble_payload[ble_payload_size++] = system_state.status;
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
  // GPIO CONFIG 2+4
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_4);
  /*-------------------------------------------------------------------------*/
  /* BLE packet transmission */

  // init RF core APIs and configure TX power
  rf_core_set_modesel();
  rf_ble_set_tx_power(BLE_RF_TX_POWER);

  // transmit BLE beacon
  rf_ble_beacon_single(BLE_ADV_CHANNEL_ALL, ble_payload, ble_payload_size);

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2+3+4
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_3);
  /*-------------------------------------------------------------------------*/
  /* cleanup and prepare shutdown */

  /* shutdown system for sleep */
  batteryless_shutdown();
  /*-------------------------------------------------------------------------*/
  PROCESS_END();
}
/* ------------------------------------------------------------------------- */
