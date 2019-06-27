/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich)
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
#include "rf-core/rf-ble.h"
#include "board-peripherals.h"

#include "data.h"
#include "policy.h"

#include "transient.h"
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
static inline void transient_shutdown(emu_energy_burst_t emu_energy,
                                      emu_output_voltage_t emu_voltage) {
  // close memory and send to sleep
  ext_fram_close(NULL);

  /* configure EMU, just before going into shutdown */
  emu_configure(emu_energy, emu_voltage);

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
  // static uint16_t delta_factor;
  static const uint16_t* selection_indexes;
  static transient_data_unit_t data_buffer[POLICY_SELECTION_SIZE];
  // static transient_data_unit_t aggregates_buffer[POLICY_BUFFER_AGGREGATES];
  static uint8_t ble_payload[BLE_ADV_MAX_SIZE];
  static am0815_tm_t timestamp;
  static int temperature;
  static int humidity;
  static bool ret;
  /*-------------------------------------------------------------------------*/
  PROCESS_BEGIN();
  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1
  gpio_hal_arch_set_pin(BOARD_IOID_GPIO_1);
  /*-------------------------------------------------------------------------*/
  // check reset source for power on reset and clear flags
  system_state.reset_source = ti_lib_sys_ctrl_reset_source_get();
  PRINTF("Reset source: 0x%x\n", (uint8_t)system_state.reset_source);

  // if not triggered by GPIO or emulated, cold start init for sleep only
  if (system_state.reset_source != RSTSRC_WAKEUP_FROM_SHUTDOWN) {
    /*-----------------------------------------------------------------------*/
    // GPIO CONFIG 1-a
    gpio_hal_arch_set_pin(BOARD_IOID_GPIO_4);
    /*-----------------------------------------------------------------------*/
    /* cold start init for sleep only */
    transient_shutdown(EMU_DEFAULT_BURST, EMU_DEFAULT_VOLTAGE);
    /*-----------------------------------------------------------------------*/
  } else {
    /* wakeup from LPM on GPIO trigger, do initialize for execution */

    // set active voltage
    emu_set_voltage(EMU_VOLTAGE_3_3V);

    // reset default system state and task id
    system_state.status = 0x00;
    system_state.task_id = 0;
  }
  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+2
  gpio_hal_arch_set_pin(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/
  /* restore backed up system state from FRAM */

  // read previous system state
  ret = data_load_system_state(&system_state_old);
  if (ret == false) {
    PRINTF("FRAM read failed\n");

    // set FRAM restore system state error flag
    system_state.status |= SYSTEM_STATUS_FRAM_RESTORE_ERROR;
  } else {
    // check last system state for restore error
    if (system_state_old.status & SYSTEM_STATUS_FRAM_RESTORE_ERROR) {
      PRINTF("detected previous FRAM restore failure\n");
    }

    // set task_id counter
    system_state.task_id = system_state_old.task_id + 1;
  }

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+2+3
  gpio_hal_arch_set_pin(BOARD_IOID_GPIO_3);
  /*-------------------------------------------------------------------------*/
  /* External RTC readout */

  // enable recharge RTC backup buffer at high voltage
  gpio_hal_arch_set_pin(BOARD_IOID_AM0815_CHARGE);

  // check for RTC IO ready
  if (gpio_hal_arch_read_pin(BOARD_IOID_AM0815_IRQ) == 0) {
    // RTC not ready to be accessed
    PRINTF("AM0815 not ready\n");

    // set error flag and invalid timestamp
    system_state.status |= SYSTEM_STATUS_RTC_ACCESS_ERROR;
    timestamp = AM0815_TIMESTAMP_INVALID;
  } else {

    // setup external RTC for access
    ret = am0815_open(NULL);
    if (ret == false) {
      // TODO: handle RTC init error
      PRINTF("AM0815 init failed\n");
      system_state.status |= SYSTEM_STATUS_RTC_CONFIG_ERROR;
    }

    // set to RC only timer mode
    ret = am0815_set_mode(NULL, AM0815_MODE_RC_ONLY);
    if (ret == false) {
      PRINTF("AM0815 configure mode failed\n");
      system_state.status |= SYSTEM_STATUS_RTC_CONFIG_ERROR;
    }

    // read time from external RTC
    ret = am0815_get_time(NULL, &timestamp);
    if(ret == false) {
      PRINTF("AM0815 read time failed\n");
      system_state.status |= SYSTEM_STATUS_RTC_READ_ERROR;
    } else {
      PRINTF("AM0815 time:  20%02u-%02u-%02u %02u:%02u:%02u.%02u\n",
              timestamp.year, timestamp.month, timestamp.day,
              timestamp.hour, timestamp.min, timestamp.sec, timestamp.split);
    }

    // close access to external RTC
    am0815_close(NULL);
  }

  /*-------------------------------------------------------------------------*/
  /* Time based system state updates */

  // store (potentially invalid) timestamp in system state
  system_state.activation_time = am0815_timestamp_to_seconds(&timestamp);

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
    system_state.power_level = policy_estimate_power_level(activation_interval);
  }

  // disable RTC backup domain charging, ((revert to lower supply voltage))
  gpio_hal_arch_clear_pin(BOARD_IOID_AM0815_CHARGE);
  // emu_set_voltage(EMU_VOLTAGE_2_3V);

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1+3
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/
  /* Sensor readout */

  // configure SHT3x sensor
  sht3x_sensor.configure(0, 0);

  // read ambient sensor values
  temperature = sht3x_sensor.value(SHT3X_TYPE_TEMPERATURE);
  humidity = sht3x_sensor.value(SHT3X_TYPE_HUMIDITY);

  // print read sensor values
  PRINTF("SHT31:  TEMP = % 5d [degC x 100]\n", temperature);
  PRINTF("SHT31:  RH   = % 5d [%% x 100]\n", humidity);

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 3
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_1);
  /*-------------------------------------------------------------------------*/
  /* Sensor data management */

  // init buffer
  ret = data_init(system_state.activation_time, system_state.power_level);
  if (ret == false) {
    PRINTF("Data buffer initialization failed\n");
    system_state.status |= SYSTEM_STATUS_FRAM_RESTORE_ERROR;
  }

  // check and reset buffer if needed
  if ((system_state.status & SYSTEM_STATUS_RTC_ACCESS_ERROR) ||
      (system_state.status & SYSTEM_STATUS_RTC_READ_ERROR) ||
      (system_state.status & SYSTEM_STATUS_FRAM_RESTORE_ERROR) ||
      (system_state.status & SYSTEM_STATUS_TIMER_RESET)) {
    PRINTF("FRAM buffer reset due FRAM restore or RTC error/reset\n");
    data_reset_buffer(system_state.activation_time);
    system_state.status |= SYSTEM_STATUS_BUFFER_RESET;
  }

  // store the measurement to memory
  data_store_measurement(system_state.activation_time, temperature, humidity);

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2+3
  gpio_hal_arch_set_pin(BOARD_IOID_GPIO_2);
  /*-------------------------------------------------------------------------*/
  /* predict power and use policy to assemble optimal data packet */
  
  // initialize policy
  policy_init();

  // get sample indexes for estimated power_level
  selection_indexes = policy_generate_selection(system_state.power_level);

#if DEBUG
  PRINTF("selected samples:  ");
  for (uint16_t i = 0; i < POLICY_SELECTION_SIZE; i++) {
    PRINTF("%5u ", selection_indexes[i]);
  }
  PRINTF("\n");
#endif

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_3);
  /*-------------------------------------------------------------------------*/
  /* Assemble BLE packet for transmission */

  // load the data selected from data buffer
  ret = data_load_selection(data_buffer, selection_indexes, POLICY_SELECTION_SIZE);
  if (ret == false) {
    PRINTF("loading selected data failed\n");
    system_state.status |= SYSTEM_STATUS_FRAM_READ_ERROR;
  }

  /*
   * Assemble manufacturer specific BLE beacon payload, see README.md for
   * detailed definition of the BLE packet structure.
   */
  uint16_t selection_index = 0; // data buffer index counter
  uint8_t ble_payload_size = 1; // setting payload size field at the end
  ble_payload[ble_payload_size++] = BLE_ADV_TYPE_MANUFACTURER;
  ble_payload[ble_payload_size++] = system_state.status;
  // add data units up to full packet size
  while (ble_payload_size + TRANSIENT_DATA_UNIT_SIZE <= BLE_ADV_MAX_SIZE &&
         selection_index < POLICY_SELECTION_SIZE) {
    uint8_t* data_bytes = (uint8_t*)&data_buffer[selection_index];
    for (uint8_t i = 0; i < TRANSIENT_DATA_UNIT_SIZE; i++) {
      ble_payload[ble_payload_size++] = data_bytes[i];
    }
    selection_index++;
  }
  // update payload size field (data length byte not counted)
  ble_payload[0] = ble_payload_size - 1;

#if DEBUG
  PRINTF("beacon payload (%u bytes):  [ ", ble_payload_size);
  for (uint8_t i = 0; i < ble_payload_size; i++) {
    if (i == 3) {
      PRINTF("|| ");
    } else if ((i - 3) % TRANSIENT_DATA_UNIT_SIZE == 0) {
      PRINTF("| ");
    }
    PRINTF("%02x ", ble_payload[i]);
  }
  PRINTF("]\n");
#endif

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2+4
  gpio_hal_arch_set_pin(BOARD_IOID_GPIO_4);
  /*-------------------------------------------------------------------------*/
  /* BLE packet transmission */

  // revert to lower supply voltage
  emu_set_voltage(EMU_VOLTAGE_2_3V);

  // init RF core APIs and configure TX power
  rf_core_set_modesel();
  rf_ble_set_tx_power(BLE_RF_TX_POWER);

  // transmit BLE beacon
  rf_ble_beacon_single(BLE_ADV_CHANNEL_ALL, ble_payload, ble_payload_size);

  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2+3+4
  gpio_hal_arch_set_pin(BOARD_IOID_GPIO_3);
  /*-------------------------------------------------------------------------*/
  /* cleanup and prepare shutdown */

  // close data buffer to store data state
  ret = data_close();
  if (ret == false) {
    PRINTF("Data access close operation failed\n");
    // no system state update, as lost anyway beyond this point
  } else {
    PRINTF("Data access closed\n");
  }

  // store next EMU burst configuration
  system_state.emu_energy = EMU_BURST_SMALL;
  system_state.emu_voltage = EMU_VOLTAGE_2_3V;

  // backup current system state
  ret = data_store_system_state(&system_state);
  if (ret == false) {
    PRINTF("FRAM backup system state failed\n");
      // no system state update, as lost anyway beyond this point
  } else {
    PRINTF("FRAM system state saved\n");
  }
  PRINTF("\n");

  /* shutdown system for sleep */
  transient_shutdown(system_state.emu_energy, system_state.emu_voltage);
  /*-------------------------------------------------------------------------*/
  PROCESS_END();
}
/* ------------------------------------------------------------------------- */
