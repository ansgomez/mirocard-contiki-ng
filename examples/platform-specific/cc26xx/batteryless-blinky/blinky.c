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
#include "ti-lib.h"
#include "lpm.h"
#include "leds.h"
#include "board.h"
#include "dev/gpio-hal.h"

#include "blinky.h"
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
/* ------------------------------------------------------------------------- */
static inline void batteryless_shutdown() {

  // LPM with GPIO triggered wakeup
  lpm_shutdown(WAKEUP_TRIGGER_IOID, IOC_NO_IOPULL, WAKEUP_TRIGGER_EDGE);
}
/* ------------------------------------------------------------------------- */
PROCESS(transient_app_process, "Transient application process");
AUTOSTART_PROCESSES(&transient_app_process);
/* ------------------------------------------------------------------------- */
PROCESS_THREAD(transient_app_process, ev, data) {

  static struct etimer timer;
  static uint8_t aux = 1;
  static uint8_t state;

  /*-------------------------------------------------------------------------*/
  PROCESS_BEGIN();
  /*-------------------------------------------------------------------------*/

  // check reset source for power on reset and clear flags
  // state = (uint8_t)ti_lib_sys_ctrl_reset_source_get();
  // PRINTF("Reset source: 0x%x\n", state);

  // // if not triggered by GPIO or emulated, cold start init for sleep only
  // if (state != RSTSRC_WAKEUP_FROM_SHUTDOWN) {
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
  // }

  leds_single_on(LEDS_BLUE);
  etimer_set(&timer, CLOCK_SECOND/20);
  PROCESS_YIELD_UNTIL((ev == PROCESS_EVENT_TIMER));
  leds_single_off(LEDS_BLUE);

// set the etimer module to generate an event in one second.
    // etimer_set(&timer, CLOCK_SECOND );
    // while (1)
    // {
    //     // wait here for an event to happen
    //     PROCESS_WAIT_EVENT();
 
    //     // if the event is the timer event as expected...
    //     if(ev == PROCESS_EVENT_TIMER)
    //     {
    //       if(aux) 
    //       {
    //         PRINTF("Turning On\n");
    //         // TOGGLE LED
    //         // leds_single_on(LEDS_GREEN);
    //         // leds_single_on(LEDS_RED);
    //         leds_single_on(LEDS_BLUE);
    //       }
    //       else
    //       {
    //         PRINTF("Turning Off\n");
    //         // leds_single_off(LEDS_GREEN);
    //         // leds_single_off(LEDS_RED);
    //         leds_single_off(LEDS_BLUE);
    //       }
          
    //       aux = !aux;

    //       // reset the timer so it will generate an other event
    //       // the exact same time after it expired (periodicity guaranteed)
    //       etimer_reset(&timer);
    //     }
 
    //     // and loop
    // }
  /*-------------------------------------------------------------------------*/
  /* shutdown system for sleep */
  batteryless_shutdown();
  PROCESS_END();
}
/* ------------------------------------------------------------------------- */