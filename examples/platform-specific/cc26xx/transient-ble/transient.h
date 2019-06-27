/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
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
/*---------------------------------------------------------------------------*/
#ifndef TRANSIENT_H_
#define TRANSIENT_H_
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include "emu.h"
#include "policy.h"
/*---------------------------------------------------------------------------*/
typedef struct transient_sys_state {
  uint8_t status;
  uint8_t task_id;
  uint8_t reset_source;
  uint8_t power_level;
  uint32_t activation_time;
  uint16_t data_history_head_idx;
  uint16_t data_history_tail_idx;
  emu_output_voltage_t emu_voltage;
  emu_energy_burst_t emu_energy;
} transient_sys_state_t;
/*---------------------------------------------------------------------------*/
typedef struct transient_data_unit {
  uint32_t time;
  uint8_t data[3];
} transient_data_unit_t;
/*---------------------------------------------------------------------------*/
/**
 * System status flags
 */
#define SYSTEM_STATUS_FRAM_RESTORE_ERROR    0x80
#define SYSTEM_STATUS_FRAM_READ_ERROR       0x40
#define SYSTEM_STATUS_FRAM_WRITE_ERROR      0x20
#define SYSTEM_STATUS_RTC_ACCESS_ERROR      0x10
#define SYSTEM_STATUS_RTC_CONFIG_ERROR      0x08
#define SYSTEM_STATUS_RTC_READ_ERROR        0x04
#define SYSTEM_STATUS_BUFFER_RESET          0x02
#define SYSTEM_STATUS_TIMER_RESET           0x01
/**
 * System status flags
 */
#define TIMESTAMP_INVALID                   0xFFFFFFFF
/**
 * Actual size of the unaligned transient data unit structure
 */
#define TRANSIENT_DATA_UNIT_SIZE            7
/**
 * FRAM memory address of the system state backup
 */
#define FRAM_SYSTEM_STATE_ADDR              0x00
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_DATA_BASE_ADDR                 0x20
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_AGGREGATE_BASE_ADDR            (FRAM_DATA_BASE_ADDR)
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_HISTORY_BASE_ADDR              (FRAM_DATA_BASE_ADDR +            \
                                             POLICY_BUFFER_AGGREGATES *       \
                                             sizeof(transient_data_unit_t))
/**
 * Maximum buffer size of the history data buffer
 */
#define FRAM_HISTORY_SIZE                   ((EXT_FRAM_MEMORY_SIZE -          \
                                              FRAM_HISTORY_BASE_ADDR) /       \
                                             sizeof(transient_data_unit_t))
/* ------------------------------------------------------------------------- */
/**
 * BLE advertisement header type for manufacturer specific packet format.
 */
#define BLE_ADV_TYPE_MANUFACTURER   0xFF
/**
 * BLE radio TX power in dBm (-21, -18, ..., 0, 1, 2, 3, 4, 5)
 */
#define BLE_RF_TX_POWER             -21
/*---------------------------------------------------------------------------*/
/**
 * GPIO used as as wakeup trigger (EMU_TRIG or LED_GPIO for testing)
 */
#define WAKEUP_TRIGGER_IOID                 BOARD_IOID_EMU_TRIG
// #define WAKEUP_TRIGGER_IOID                 BOARD_IOID_LED_1
/**
 * GPIO trigger edge (IOC_WAKE_ON_HIGH or IOC_WAKE_ON_LOW for parallel testing)
 */
#define WAKEUP_TRIGGER_EDGE                 IOC_WAKE_ON_HIGH
// #define WAKEUP_TRIGGER_EDGE                 IOC_WAKE_ON_LOW
/*---------------------------------------------------------------------------*/
/**
 * Split a compressed data unit into its components
 */
inline void transient_data_unit_join(transient_data_unit_t* const data_unit,
                                     const uint32_t timestamp,
                                     const uint16_t temperature_raw,
                                     const uint16_t humidity_raw) {
  data_unit->time = timestamp;
  data_unit->data[0] = (uint8_t)(humidity_raw & 0xFF);
  data_unit->data[1] = (uint8_t)(((humidity_raw >> 8) & 0x03) | ((temperature_raw & 0x3F) << 2));
  data_unit->data[2] = (uint8_t)((temperature_raw >> 6) & 0xFF);
}
/**
 * Split a compressed data unit into its components
 */
inline void transient_data_unit_split(const transient_data_unit_t* const data_unit,
                                      uint32_t* const timestamp,
                                      uint16_t* const temperature_raw,
                                      uint16_t* const humidity_raw) {
  *timestamp = data_unit->time;
  *humidity_raw = (((uint16_t)data_unit->data[1] & 0x03) << 8) | data_unit->data[0];
  *temperature_raw = ((uint16_t)data_unit->data[2] << 6) | ((data_unit->data[1] & 0xFC) >> 2);
}
/**
 * Encode mesured timestamp and sensor values to a data unit
 */
inline void transient_data_unit_encode(transient_data_unit_t* const data_unit,
                                       const uint32_t timestamp,
                                       const int32_t temperature,
                                       const int32_t humidity) {
  uint16_t humidity_data = (uint16_t)(humidity / 10) & 0x03FF;
  uint16_t temperature_data = (uint16_t)(temperature + 4000) & 0x3FFF;
  transient_data_unit_join(data_unit, timestamp, temperature_data, humidity_data);
}
/*---------------------------------------------------------------------------*/
#endif /* TRANSIENT_H_ */
/*---------------------------------------------------------------------------*/
