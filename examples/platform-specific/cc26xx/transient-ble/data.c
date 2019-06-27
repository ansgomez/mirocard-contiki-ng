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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ext-fram.h"
#include "haar.h"

#include "data.h"
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
/**
 * To force data aggregation in every single activation set to 1.
 */
#define FORCE_AGGREGATE_GENERATION  0
#if FORCE_AGGREGATE_GENERATION
#undef DATA_AVERAGE_TIMEOUT
#undef DATA_AGGREGATE_TIMEOUT
#define DATA_AVERAGE_TIMEOUT        1
#define DATA_AGGREGATE_TIMEOUT      1
#endif
/**
 * To skip data aggregation in every single activation set to 1.
 */
#define SKIP_AGGREGATE_GENERATION  0
#if SKIP_AGGREGATE_GENERATION & FORCE_AGGREGATE_GENERATION
#error "force and skip aggregation generation is NOT allowed!"
#endif

/* ------------------------------------------------------------------------- */
/**
 * Global data state.
 */
static transient_data_state_t data_state;
/**
 * Global buffer scaling factor for power dependent memory operations.
 */
static uint8_t buffer_scaling_factor = 1;
/*---------------------------------------------------------------------------*/
/**
 * Append data unit to history buffer.
 * @param data_unit Transient data unit to append to buffer
 * @return False on error, true on success
 */
static bool data_append_history(const transient_data_unit_t* const data_unit) {
  bool ret;

  // write value multiple times to account for variable power
  for (uint16_t i = 0; i < buffer_scaling_factor; i++) {
    // calculate FRAM write address
    uint32_t addr_write = FRAM_HISTORY_BASE_ADDR + sizeof(transient_data_unit_t) *
                          (uint32_t)data_state.history_head_index;

    // write data unit to FRAM
    PRINTF("FRAM write new history element at 0x%05lx\n", addr_write);
    ret = ext_fram_write(NULL, addr_write, sizeof(transient_data_unit_t),
                         (uint8_t*)data_unit);
    if (ret == false) {
        PRINTF("FRAM saving to history buffer failed\n");
        return false;
    }
    
    // update buffer pointers on success write
    data_state.history_head_index++;
    if (data_state.history_head_index == DATA_HISTORY_SIZE) {
      data_state.history_head_index = 0;
    }

    // on full buffer (head maps to tail): drop oldest value
    if (data_state.history_head_index == data_state.history_tail_index) {
      data_state.history_tail_index++;
      if (data_state.history_tail_index == DATA_HISTORY_SIZE) {
        data_state.history_tail_index = 0;
      }
    }

    PRINTF("FRAM history buffer state:   head = %4u,   tail = %4u\n",
           data_state.history_head_index, data_state.history_tail_index);
  }

  return true;
}
/*---------------------------------------------------------------------------*/
/**
 * Process and update aggregate values.
 * @param temperature Temperature sensor reading
 * @param humidity Humidity sensor reading
 * @param value The current sensor value to use for updating the aggregates
 */
static bool data_process_aggregates(const uint32_t timestamp,
                                    const int32_t temperature,
                                    const int32_t humidity) {
  bool ret;
#if FORCE_AGGREGATE_GENERATION
  data_state.average_timestamp = timestamp - 1;
  data_state.aggregate_timestamp = timestamp - 1;
#endif

  // update aggregate sums
  data_state.average_sum[DATA_TEMPERATURE_INDEX] += temperature * buffer_scaling_factor;
  data_state.average_sum[DATA_HUMIDITY_INDEX] += humidity * buffer_scaling_factor;
  data_state.average_count += buffer_scaling_factor;

  /*-------------------------------------------------------------------------*/
  /* new average value (short timeout) */
  if (timestamp >= data_state.average_timestamp + DATA_AVERAGE_TIMEOUT) {
    // calculate average
    data_state.average_sum[DATA_TEMPERATURE_INDEX] = 
        data_state.average_sum[DATA_TEMPERATURE_INDEX] / data_state.average_count;
    data_state.average_sum[DATA_HUMIDITY_INDEX] = 
        data_state.average_sum[DATA_HUMIDITY_INDEX] / data_state.average_count / 10;

    // store temperature and humidity average (-1 as consequence of timeout check)
    uint32_t average_index = (timestamp - data_state.aggregate_timestamp) /
                              DATA_AVERAGE_TIMEOUT - 1;

    for (int i = 0; i < 2; i++) {
      uint32_t addr_write = FRAM_AVERAGE_BASE_ADDR + sizeof(data_state.average_sum[i]) *
                            (average_index + i * DATA_AGGREGATE_WINDOW);
      
      // write data unit to FRAM
      PRINTF("FRAM write new average value with index %lu at 0x%05lx\n", average_index, addr_write);
      ret = ext_fram_write(NULL, addr_write, sizeof(data_state.average_sum[i]),
                          (uint8_t*)&data_state.average_sum[i]);
      if (ret == false) {
        PRINTF("FRAM saving to average buffer failed\n");
        return false;
      }
    }

    // update aggregates timeout and reset aggregate states
    data_state.average_timestamp = timestamp - (timestamp % DATA_AVERAGE_TIMEOUT);
    data_state.average_sum[DATA_TEMPERATURE_INDEX] = 0;
    data_state.average_sum[DATA_HUMIDITY_INDEX] = 0;
    data_state.average_count = 0;
  }

  /*-------------------------------------------------------------------------*/
  /* new compressed aggregate value (long timeout) */
  if (timestamp >= data_state.aggregate_timestamp + DATA_AGGREGATE_TIMEOUT) {
#if !(SKIP_AGGREGATE_GENERATION)
    PRINTF("DATA generate new aggregate values\n");
    
    // extract the most significant coefficients with index
    static int32_t data[DATA_AGGREGATE_WINDOW];
    static int32_t coeff_value[2][POLICY_BUFFER_AGGREGATES] = {{0}};
    static uint16_t coeff_index[2][POLICY_BUFFER_AGGREGATES] = {{-1}};

    // repeat for both sensors
    for (int i = 0; i < 2; i++) {
      // load average data from FRAM and compress
      uint32_t addr_read = FRAM_AVERAGE_BASE_ADDR + i * sizeof(data);
      PRINTF("FRAM read aggregate buffer for compression at 0x%05lx\n", addr_read);
      ret = ext_fram_read(NULL, addr_read, sizeof(data),
                          (uint8_t*)data);
      if (ret == false) {
        PRINTF("FRAM loading aggregate buffer failed\n");
        return false;
      }

      // perform Haar Wavelet Transformation
      haar_compress_inplace(data, DATA_AGGREGATE_WINDOW);
      
      // extract most significant coefficient with index
      for (uint32_t j = 0; j < DATA_AGGREGATE_WINDOW; j++) {
        // insert element in list, if larger than lowest selected value
        if (abs(data[j]) > abs(coeff_value[i][POLICY_BUFFER_AGGREGATES - 1])) {
          // reorder selection list by swapping, until insertion position is found
          uint32_t k = POLICY_BUFFER_AGGREGATES;
          while (k > 0) {
            k--;
            if (k > 0 && abs(data[j]) > abs(coeff_value[i][k - 1])) {
              // swap values and indexes
              coeff_value[i][k] = coeff_value[i][k - 1];
              coeff_index[i][k] = coeff_index[i][k - 1];
            } else {
              // insert value and index, break
              coeff_value[i][k] = data[j];
              coeff_index[i][k] = j;
              break;
            }
          }
        }
      }
    }

    // build and store aggregate data units
    static transient_data_unit_t aggregate_data[POLICY_BUFFER_AGGREGATES];
    for (uint16_t i = 0; i < POLICY_BUFFER_AGGREGATES; i++) {
      data_encode_aggregate(&aggregate_data[i], data_state.aggregate_timestamp, i,
                            coeff_index[DATA_TEMPERATURE_INDEX][i],
                            coeff_value[DATA_TEMPERATURE_INDEX][i],
                            coeff_index[DATA_HUMIDITY_INDEX][i],
                            coeff_value[DATA_HUMIDITY_INDEX][i]);
    }

    // write aggregate data units to FRAM
    PRINTF("FRAM write new aggregate coeffs at 0x%05x\n", FRAM_AGGREGATE_BASE_ADDR);
    ret = ext_fram_write(NULL, FRAM_AGGREGATE_BASE_ADDR,
                         POLICY_BUFFER_AGGREGATES * sizeof(transient_data_unit_t),
                         (uint8_t*)aggregate_data);
    if (ret == false) {
      PRINTF("FRAM saving aggregate data units failed\n");
      return false;
    }
#endif

    // reset aggregate timestamp
    data_state.aggregate_timestamp = timestamp - (timestamp % DATA_AGGREGATE_TIMEOUT);
  }

  return true;
}
/*---------------------------------------------------------------------------*/
/**
 * Load the system state from memory without needing data module initialization.
 */
bool data_load_system_state(transient_system_state_t *system_state) {
  bool ret;
  PRINTF("FRAM read system state at 0x%05x\n", FRAM_SYSTEM_STATE_ADDR);
  ret = ext_fram_read(NULL, FRAM_SYSTEM_STATE_ADDR,
                      sizeof(transient_system_state_t),
                      (uint8_t*)system_state);
  return ret;
}
/*---------------------------------------------------------------------------*/
/**
 * Store the system state from memory without needing data module initialization.
 */
bool data_store_system_state(transient_system_state_t const *system_state) {
  bool ret;
  PRINTF("FRAM write system state at 0x%05x\n", FRAM_SYSTEM_STATE_ADDR);
  ret = ext_fram_write(NULL, FRAM_SYSTEM_STATE_ADDR,
                       sizeof(transient_system_state_t),
                       (uint8_t*)system_state);
  return ret;
}
/*---------------------------------------------------------------------------*/
bool data_init(const uint32_t timestamp, uint8_t power_level) {
  bool ret;

  PRINTF("DATA init using configuration:\n"
         " FRAM_STATE_BASE_ADDR     = 0x%05x\n"
         " FRAM_DATA_BASE_ADDR      = 0x%05x\n"
         " FRAM_SYSTEM_STATE_ADDR   = 0x%05x\n"
         " FRAM_DATA_STATE_ADDR     = 0x%05x\n"
         " FRAM_AVERAGE_BASE_ADDR   = 0x%05x\n"
         " FRAM_AGGREGATE_BASE_ADDR = 0x%05x\n"
         " FRAM_HISTORY_BASE_ADDR   = 0x%05x\n"
         " DATA_HISTORY_SIZE        = 0x%05x\n",
         FRAM_STATE_BASE_ADDR, FRAM_DATA_BASE_ADDR, FRAM_SYSTEM_STATE_ADDR,
         FRAM_DATA_STATE_ADDR, FRAM_AVERAGE_BASE_ADDR, FRAM_AGGREGATE_BASE_ADDR,
         FRAM_HISTORY_BASE_ADDR, DATA_HISTORY_SIZE);

  // get scaling factor to account for variable power
  buffer_scaling_factor = policy_get_delta_factor(power_level);

  // restore data state from FRAM
  PRINTF("FRAM read data state at 0x%05x\n", FRAM_DATA_STATE_ADDR);
  ret = ext_fram_read(NULL, FRAM_DATA_STATE_ADDR,
                      sizeof(transient_data_state_t),
                      (uint8_t*)&data_state);
  if (ret == false) {
    // reset buffer state on restore error
    data_reset_buffer(timestamp);
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
bool data_close(void) {
  bool ret;
  // store buffer state to FRAM
  PRINTF("FRAM read data state at 0x%05x\n", FRAM_DATA_STATE_ADDR);
  ret = ext_fram_write(NULL, FRAM_DATA_STATE_ADDR,
                       sizeof(transient_data_state_t),
                       (uint8_t*)&data_state);
  return ret;
}
/*---------------------------------------------------------------------------*/
bool data_reset_buffer(const uint32_t timestamp) {
  bool ret;

  PRINTF("DATA resetting buffers\n");

  // reset history ring buffer
  data_state.history_head_index = 0;
  data_state.history_tail_index = 0;

  // reset average and aggregate states
  data_state.average_timestamp = timestamp;
  data_state.average_sum[DATA_TEMPERATURE_INDEX] = 0;
  data_state.average_sum[DATA_HUMIDITY_INDEX] = 0;
  data_state.average_count = 0;
  data_state.aggregate_timestamp = timestamp;

  // store buffer state immediately to memory by calling close function
  ret = data_close();
  if (ret == false) {
    PRINTF("DATA resetting data buffer state failed\n");
    return false;
  }

  // clear aggregates buffer in FRAM
  transient_data_unit_t reset_buffer[POLICY_BUFFER_AGGREGATES];
  for (uint8_t i = 0; i < POLICY_BUFFER_AGGREGATES; i++) {
    memset(&reset_buffer[i], (0xFF - i), sizeof(reset_buffer[0]));
  }
  PRINTF("FRAM write reset aggregate values at 0x%05x\n", FRAM_AGGREGATE_BASE_ADDR);
  ret = ext_fram_write(NULL, FRAM_AGGREGATE_BASE_ADDR, sizeof(reset_buffer),
                       (uint8_t*)reset_buffer);
  if (ret == false) {
    PRINTF("DATA resetting aggregate buffer failed\n");
    return false;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
bool data_store_measurement(const uint32_t timestamp, const int32_t temperature,
                            const int32_t humidity) {
  bool ret;
  static transient_data_unit_t data_unit;

  // assemble data unit structure to store
  data_encode_value(&data_unit, timestamp, temperature, humidity);

  // add to history buffer
  ret = data_append_history(&data_unit);
  if (ret == false) {
    PRINTF("DATA append to history buffer failed\n");
    return false;
  }

  // handle aggregate updates
  ret = data_process_aggregates(timestamp, temperature, humidity);
  if (ret == false) {
    PRINTF("DATA processing data aggregates failed\n");
    return false;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
bool data_load_selection(transient_data_unit_t* data, const uint16_t* selection, uint16_t size) {
  bool ret;
  // history buffer size calculation for truncating selection
  uint16_t fram_buffer_size = (DATA_HISTORY_SIZE + data_state.history_head_index - 
                          data_state.history_tail_index) % DATA_HISTORY_SIZE;
  PRINTF("FRAM buffer size: %5u of %5u\n", fram_buffer_size, DATA_HISTORY_SIZE);

  // get packet data from the FRAM
  for (uint16_t i = 0; i < size; i++) {
    uint32_t addr_read;
    // truncate invalid samples
    if (selection[i] == POLICY_SELECTION_UNSET) {
      // unset value selections: use current sensor value
      addr_read = FRAM_HISTORY_BASE_ADDR + sizeof(transient_data_unit_t) *
                  ((DATA_HISTORY_SIZE + 
                    (uint32_t)data_state.history_head_index - 1) %
                   DATA_HISTORY_SIZE);
    } else if (selection[i] >= POLICY_AGGREGATE_OFFSET) {
      // aggregate values: use bufferd aggregates
      addr_read = FRAM_AGGREGATE_BASE_ADDR + sizeof(transient_data_unit_t) *
                  (selection[i] - POLICY_AGGREGATE_OFFSET);
    } else {
      // buffered sensor values
      if (selection[i] * buffer_scaling_factor >= fram_buffer_size) {
        // truncate selection to oldest element if scaled index beyond buffer size
        addr_read = FRAM_HISTORY_BASE_ADDR + sizeof(transient_data_unit_t) *
                    (uint32_t)data_state.history_tail_index;
      } else {
        // calculate data value address in buffer (scaled with delta factor)
        addr_read = FRAM_HISTORY_BASE_ADDR + sizeof(transient_data_unit_t) *
                    ((DATA_HISTORY_SIZE +
                      (uint32_t)data_state.history_head_index - 1 -
                      selection[i] * buffer_scaling_factor) %
                     DATA_HISTORY_SIZE);
      }
    }

    // read value from FRAM
    PRINTF("FRAM read data unit # %5u at 0x%05lx\n", selection[i], addr_read);
    ret = ext_fram_read(NULL, addr_read, sizeof(transient_data_unit_t), 
                        (uint8_t*)(&data[i]));
    if (ret == false) {
      PRINTF("FRAM read data unit failed\n");
      return false;
    }
  }
  return true;
}
/*---------------------------------------------------------------------------*/
