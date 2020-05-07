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
/*---------------------------------------------------------------------------*/
#ifndef BATTERYLESS_H_
#define BATTERYLESS_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "board.h"

#include <stdint.h>
/*---------------------------------------------------------------------------*/
/**
 * Transient system state structure
 */
typedef struct transient_system_state {
  uint8_t status;
  uint8_t task_id;
  uint8_t reset_source;
  uint8_t power_level;
  uint32_t activation_time;
} transient_system_state_t;
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
 * Invalid timestamp constant
 */
#define TIMESTAMP_INVALID                   0xFFFFFFFF
/* ------------------------------------------------------------------------- */
/**
 * BLE advertisement header type for manufacturer specific packet format.
 */
#define BLE_ADV_TYPE_MANUFACTURER           0xFF
/**
 * BLE radio TX power in dBm (-21, -18, ..., 0, 1, 2, 3, 4, 5)
 */
#define BLE_RF_TX_POWER                     -21
/*---------------------------------------------------------------------------*/
/**
 * GPIO used as as wakeup trigger (EMU_TRIG or LED_GPIO for testing)
 */
#define WAKEUP_TRIGGER_IOID                 BOARD_IOID_EMU_COMP
// #define WAKEUP_TRIGGER_IOID                 BOARD_IOID_LED_1
/**
 * GPIO trigger edge (IOC_WAKE_ON_HIGH or IOC_WAKE_ON_LOW for parallel testing)
 */
#define WAKEUP_TRIGGER_EDGE                 IOC_WAKE_ON_HIGH
// #define WAKEUP_TRIGGER_EDGE                 IOC_WAKE_ON_LOW
/*---------------------------------------------------------------------------*/
#endif /* BATTERYLESS_H_ */
/*---------------------------------------------------------------------------*/
