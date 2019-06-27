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
/*---------------------------------------------------------------------------*/
#ifndef DATA_H_
#define DATA_H_
/*---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "policy.h"
#include "transient.h"
/*---------------------------------------------------------------------------*/
typedef struct transient_data_state {
  uint16_t history_head_index;
  uint16_t history_tail_index;
  uint32_t aggregate_timestamp;
  uint32_t average_timestamp;
  int32_t average_sum[2];
  uint16_t average_count;
} transient_data_state_t;
/*---------------------------------------------------------------------------*/
typedef struct transient_data_unit {
  uint32_t time;
  uint8_t data[3];
} transient_data_unit_t;
/*---------------------------------------------------------------------------*/
/**
 * Actual size of the unaligned transient data unit structure
 */
#define TRANSIENT_DATA_UNIT_SIZE    7
/**
 * Buffer index of the temperature data
 */
#define DATA_TEMPERATURE_INDEX      0
/**
 * Buffer index of the humidity data
 */
#define DATA_HUMIDITY_INDEX         1
/**
 * Timeout for average building [in seconds]
 */
#define DATA_AVERAGE_TIMEOUT        360
/**
 * Data aggregate window for compression (a power of two)
 */
#define DATA_AGGREGATE_WINDOW       256
/**
 * Timeout for average building [in seconds]
 */
#define DATA_AGGREGATE_TIMEOUT      (DATA_AVERAGE_TIMEOUT * DATA_AGGREGATE_WINDOW)
/**
 * Buffer size of the FRAM scratch space
 */
#define DATA_AGGREGATE_SIZE         (2 * DATA_AGGREGATE_WINDOW * sizeof(int32_t))
/*---------------------------------------------------------------------------*/
/**
 * FRAM memory base address of the state buffer
 */
#define FRAM_STATE_BASE_ADDR        0x0000
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_DATA_BASE_ADDR         0x0030
/**
 * FRAM memory address of the system state backup
 */
#define FRAM_SYSTEM_STATE_ADDR      (FRAM_STATE_BASE_ADDR + 0x00)
/**
 * FRAM memory address of the data state backup
 */
#define FRAM_DATA_STATE_ADDR        (FRAM_STATE_BASE_ADDR + 0x10)
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_AVERAGE_BASE_ADDR      (FRAM_DATA_BASE_ADDR + 0x00)
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_AGGREGATE_BASE_ADDR    (FRAM_DATA_BASE_ADDR + DATA_AGGREGATE_SIZE)
/**
 * FRAM memory base address of the data buffer
 */
#define FRAM_HISTORY_BASE_ADDR      (FRAM_DATA_BASE_ADDR +                    \
                                     DATA_AGGREGATE_SIZE +                    \
                                     POLICY_BUFFER_AGGREGATES *               \
                                     sizeof(transient_data_unit_t))
/**
 * Maximum buffer size of the history data buffer
 */
#define DATA_HISTORY_SIZE           ((EXT_FRAM_MEMORY_SIZE -                  \
                                      DATA_AGGREGATE_SIZE) /                  \
                                     sizeof(transient_data_unit_t))
/*---------------------------------------------------------------------------*/
/**
 * Decode a compressed history value data unit into its components
 */
inline void data_decode_value(const transient_data_unit_t* const data_unit,
                              uint32_t* const timestamp,
                              uint16_t* const temperature_raw,
                              uint16_t* const humidity_raw) {
  *timestamp = data_unit->time;
  *humidity_raw = (((uint16_t)data_unit->data[1] & 0x03) << 8) | data_unit->data[0];
  *temperature_raw = ((uint16_t)data_unit->data[2] << 6) | ((data_unit->data[1] & 0xFC) >> 2);
}
/**
 * Encode mesured timestamp and sensor values to a history value data unit
 */
inline void data_encode_value(transient_data_unit_t* const data_unit,
                              const uint32_t timestamp,
                              const int32_t temperature,
                              const int32_t humidity) {
  uint16_t humidity_raw = (uint16_t)(humidity / 10) & 0x03FF;
  uint16_t temperature_raw = (uint16_t)(temperature + 4000) & 0x3FFF;
  data_unit->time = timestamp;
  data_unit->data[0] = (uint8_t)(humidity_raw & 0xFF);
  data_unit->data[1] = (uint8_t)(((humidity_raw >> 8) & 0x03) | ((temperature_raw & 0x3F) << 2));
  data_unit->data[2] = (uint8_t)((temperature_raw >> 6) & 0xFF);
}
/**
 * Encode compressed aggregate coefficient with indexes in an aggregate data unit
 */
inline void data_encode_aggregate(transient_data_unit_t* const data_unit,
                                  const uint32_t timestamp,
                                  const uint8_t index,
                                  const uint16_t temperature_index,
                                  const int32_t temperature_coeff,
                                  const uint16_t humidity_index,
                                  const int32_t humidity_coeff) {
  // generate 22 (8+14) bit of humidity, and 26 (8+18) bit of temperature data
  uint32_t humidity_data = (((uint32_t)humidity_index & 0xFF) << 14) | ((uint32_t)humidity_coeff & 0x3FFF);
  uint32_t temperature_data = (((uint32_t)temperature_index & 0xFF) << 18) | ((uint32_t)temperature_coeff & 0x3FFFF);

  data_unit->time = (((uint32_t)(0xFF - index) << 24) & 0xFF000000) | ((humidity_data << 2) & 0x00FFFFFC) | ((temperature_data >> 24) & 0x03);
  data_unit->data[0] = (uint8_t)(temperature_data & 0xFF);
  data_unit->data[1] = (uint8_t)((temperature_data >> 8) & 0xFF);
  data_unit->data[2] = (uint8_t)((temperature_data >> 16) & 0xFF);
}
/* ------------------------------------------------------------------------- */
bool data_load_system_state(transient_system_state_t* system_state);
bool data_store_system_state(const transient_system_state_t* system_state);

bool data_init(const uint32_t timestamp, uint8_t power_level);
bool data_close(void);
bool data_store_measurement(const uint32_t timestamp, const int32_t temperature,
                            const int32_t humidity);
bool data_reset_buffer(const uint32_t timestamp);

/**
 * Load the selected data based on the selection index
 * @param data Array of data units to store the selected data to
 * @param selection Array of selection indexes.
 * @param size Size of the selection of selection indexes.
 * @return True on success, false on error.
 */
bool data_load_selection(transient_data_unit_t* data, const uint16_t* selection, uint16_t size);
/*---------------------------------------------------------------------------*/
#endif /* DATA_H_ */
/*---------------------------------------------------------------------------*/
