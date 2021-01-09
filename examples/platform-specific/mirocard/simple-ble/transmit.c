/*
 * 
 * Copyright (c) 2020, Andres Gomez, Miromico AG
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
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
#include "ti-lib.h"
#include "rf-core/rf-ble.h"
#include "board.h"
#include "board-peripherals.h"
#include "batmon-sensor.h"
#include "lib/sensors.h"

#include "transmit.h"
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
  // lpm_shutdown(WAKEUP_TRIGGER_IOID, IOC_NO_IOPULL, WAKEUP_TRIGGER_EDGE);
  lpm_shutdown(WAKEUP_TRIGGER_IOID, IOC_IOPULL_UP, WAKEUP_TRIGGER_EDGE);
}
/* ------------------------------------------------------------------------- */
PROCESS(transient_app_process, "Transient application process");
AUTOSTART_PROCESSES(&transient_app_process);

/*---------------------------------------------------------------------------*/
/* PROCESSES */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(transient_app_process, ev, data) {
  static uint8_t ble_payload[BLE_ADV_MAX_SIZE];
  static uint8_t state;
  /*-------------------------------------------------------------------------*/
  PROCESS_BEGIN();
  /*-------------------------------------------------------------------------*/

  // check reset source for power on reset and clear flags
  state = ti_lib_sys_ctrl_reset_source_get();
  PRINTF("Reset source: 0x%x\n", (uint8_t)state);

#ifdef MIROCARD_BATTERYLESS
  // if not triggered by GPIO or emulated, cold start init for sleep only
  if (state!= RSTSRC_WAKEUP_FROM_SHUTDOWN) {
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

  /*-------------------------------------------------------------------------*/
  /*
   * Assemble manufacturer specific BLE beacon payload, see README.md for
   * detailed definition of the BLE packet structure.
   */
  uint8_t ble_payload_size = 1; // setting payload size field at the end
  ble_payload[ble_payload_size++] = BLE_ADV_TYPE_MANUFACTURER;
  ble_payload[ble_payload_size++] = state;
  for(uint8_t i=1;i<0x0F;i++) {
    ble_payload[ble_payload_size++] = i;
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

 /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 2 - Start of Transmission
#ifdef MIROCARD_GPIO_TRACING  
  ti_lib_gpio_set_dio(BOARD_IOID_GPIO_2);
#endif
  /*-------------------------------------------------------------------------*/

  // transmit BLE beacon
  rf_ble_beacon_single(BLE_ADV_CHANNEL_ALL, ble_payload, ble_payload_size);
  /*-------------------------------------------------------------------------*/
  /* cleanup and prepare shutdown */


  /*-------------------------------------------------------------------------*/
  // GPIO CONFIG 1/2 - End of Activation and Transmission
#ifdef MIROCARD_GPIO_TRACING
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_1); //This GPIO was set in platform.c
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_2);
#endif
  /*-------------------------------------------------------------------------*/

#ifdef MIROCARD_BATTERYLESS
  PRINTF("Shutting down\n");
  /* shutdown system for sleep */
  batteryless_shutdown();
#endif
  /*-------------------------------------------------------------------------*/
  PROCESS_END();
}
/* ------------------------------------------------------------------------- */
