/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich)
 * Copyright (c) 2020, Miromico AG - http://www.miromico.ch/
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
#include "sys/cc.h"

#include "sense-transmit.h"
/*---------------------------------------------------------------------------*/
typedef struct batteryless_data_state {
  uint16_t history_head_index;
  uint16_t history_tail_index;
  uint32_t aggregate_timestamp;
  uint32_t average_timestamp;
  int32_t average_sum[2];
  uint16_t average_count;
} batteryless_data_state_t;
/*---------------------------------------------------------------------------*/
/**
 * Actual size of the unaligned transient data unit structure
 */
#define BATTERYLESS_DATA_UNIT_SIZE    18
/*---------------------------------------------------------------------------*/
typedef union batteryless_data_unit {
  struct {
    uint32_t time;
    int accel[3];
    uint16_t light;
  };
  uint8_t bytes[BATTERYLESS_DATA_UNIT_SIZE];
} batteryless_data_unit_t CC_ALIGN (4);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
#endif /* DATA_H_ */
/*---------------------------------------------------------------------------*/
