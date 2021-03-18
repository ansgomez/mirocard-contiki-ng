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
#include "ti-lib.h"
#include "lpm.h"
#include "leds.h"
#include "board.h"
#include "dev/gpio-hal.h"
#include "button-hal.h"
#include "dev/button-hal.h"

#include "blinky-button.h"

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

#ifdef MIROCARD_BATTERYLESS
#pragma message "Running WITH EMU"
#else
#pragma message "Running WITHOUT EMU"
#endif

/* ------------------------------------------------------------------------- */
static inline void batteryless_shutdown()
{

#ifdef MIROCARD_BATTERYLESS
  // LPM with WAKEUP triggered activation
  lpm_shutdown(WAKEUP_TRIGGER_IOID, IOC_NO_IOPULL, WAKEUP_TRIGGER_EDGE);
#endif

  // LPM with USER SWITCH triggered activation
  // lpm_shutdown(BOARD_IOID_KEY_USER, IOC_NO_IOPULL, IOC_WAKE_ON_LOW);
}
/* ------------------------------------------------------------------------- */
PROCESS(transient_app_process, "Transient application process");
AUTOSTART_PROCESSES(&transient_app_process);
/* ------------------------------------------------------------------------- */
PROCESS_THREAD(transient_app_process, ev, data)
{

  static struct etimer timer;
  static uint8_t aux = 1;
  static uint8_t state;

  button_hal_button_t *btn;

  /*-------------------------------------------------------------------------*/
  PROCESS_BEGIN();
  /*-------------------------------------------------------------------------*/

  btn = button_hal_get_by_index(0);

  PRINTF("Button HAL example.\n");
  PRINTF("Device button count: %u.\n", button_hal_button_count);

  if (btn)
  {
    PRINTF("%s on pin %u with ID=0, Logic=%s, Pull=%s\n",
           BUTTON_HAL_GET_DESCRIPTION(btn), btn->pin,
           btn->negative_logic ? "Negative" : "Positive",
           btn->pull == GPIO_HAL_PIN_CFG_PULL_UP ? "Pull Up" : "Pull Down");
  }

  // check reset source for power on reset and clear flags
  state = (uint8_t)ti_lib_sys_ctrl_reset_source_get();
  // PRINTF("Reset source: 0x%x\n", state);

  /****
  * NOTE: This application is meant to test the board with a normal supply.
  * This application does "not" work in batteryless mode because it needs
  * too much energy to see an LED turn on. If running in batteryless mode, this
  * will have very short LED bursts.
  */
#ifdef MIROCARD_BATTERYLESS
  // if not triggered by GPIO or emulated, cold start init for sleep only
  if (state != RSTSRC_WAKEUP_FROM_SHUTDOWN)
  {
    /*-----------------------------------------------------------------------*/
    PRINTF("Going to sleep waiting for trigger\n");
    /*-----------------------------------------------------------------------*/
    /* cold start init for sleep only */
    batteryless_shutdown();
    /*-----------------------------------------------------------------------*/
  }
  else
  {
    /* wakeup from LPM on GPIO trigger, do initialize for execution */
    PRINTF("Woken up to perform a task\n");
  }

  leds_single_on(LEDS_BLUE);
  etimer_set(&timer, CLOCK_SECOND / 20);
  PROCESS_YIELD_UNTIL((ev == PROCESS_EVENT_TIMER));
  leds_single_off(LEDS_BLUE);

  /*-------------------------------------------------------------------------*/
  /* shutdown system for sleep */
  batteryless_shutdown();
#else

  // set the etimer module to generate an event in one second.
   etimer_set(&timer, 10*CLOCK_SECOND );
   PRINTF("Before while");
  while (1)
  {
     PRINTF("Before wait\n");
    PROCESS_YIELD();

    PRINTF("Woke up, processing event\n");

    if (ev == button_hal_press_event)
    {
      btn = (button_hal_button_t *)data;
      PRINTF("Press event (%s)\n", BUTTON_HAL_GET_DESCRIPTION(btn));
      aux = !aux;
      // if (btn == button_hal_get_by_id(BUTTON_HAL_ID_BUTTON_ZERO))
      // {
      //   PRINTF("This was button 0, on pin %u\n", btn->pin);
      //   aux = !aux;
      // }

      if (aux)
      {
        PRINTF("Turning Red on\n");
        leds_single_off(LEDS_GREEN);
        leds_single_on(LEDS_RED);
        leds_single_off(LEDS_BLUE);
      }
      else
      {
        PRINTF("Turning Green on\n");
        leds_single_on(LEDS_GREEN);
        leds_single_off(LEDS_RED);
        leds_single_off(LEDS_BLUE);
      }
    }
    else if (ev == button_hal_release_event)
    {
      btn = (button_hal_button_t *)data;
      PRINTF("Release event (%s)\n", BUTTON_HAL_GET_DESCRIPTION(btn));
    }
    else if (ev == button_hal_periodic_event)
    {
      btn = (button_hal_button_t *)data;
      PRINTF("Periodic event, %u seconds (%s)\n", btn->press_duration_seconds,
             BUTTON_HAL_GET_DESCRIPTION(btn));

      if (btn->press_duration_seconds > 5)
      {
        PRINTF("%s pressed for more than 5 secs. Do custom action\n",
               BUTTON_HAL_GET_DESCRIPTION(btn));
      }
    }
    if(ev == PROCESS_EVENT_TIMER) {
      PRINTF("processing a timer event\n");
      etimer_reset(&timer);
    }
  }
#endif
  PROCESS_END();
}
/* ------------------------------------------------------------------------- */