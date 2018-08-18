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
#include "contiki.h"
#include "gpio-hal.h"
#include "emu.h"

#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE                      "emu"
#define LOG_LEVEL                       LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/
#ifndef EMU_DEFAULT_BURST
#define EMU_DEFAULT_BURST               EMU_BURST_SMALL
#endif
#ifndef EMU_DEFAULT_VOLTAGE
#define EMU_DEFAULT_VOLTAGE             EMU_VOLTAGE_2_0V
#endif
/*---------------------------------------------------------------------------*/
#define EMU_VOLTAGE_SELECT1_MASK        0x01
#define EMU_VOLTAGE_SELECT2_MASK        0x02
#define EMU_VOLTAGE_SELECT3_MASK        0x04
#define EMU_VOLTAGE_SELECT4_MASK        0x08
#define EMU_BURST_SELECT1_MASK          0x01
#define EMU_BURST_SELECT2_MASK          0x02
/*---------------------------------------------------------------------------*/
static emu_energy_burst_t emu_energy_burst;
static emu_output_voltage_t emu_output_voltage;
/*---------------------------------------------------------------------------*/
void
emu_init(void)
{
  LOG_DBG("init\n");

  // set default configuration
  emu_energy_burst = EMU_DEFAULT_BURST;
  emu_output_voltage = EMU_DEFAULT_VOLTAGE;

  // energy threshold configuration GPIOs
  gpio_hal_arch_pin_set_output(BOARD_IOID_EMU_OV_SEL_1);
  gpio_hal_arch_pin_set_output(BOARD_IOID_EMU_OV_SEL_2);

  // voltage configuration GPIOs
  gpio_hal_arch_pin_set_output(BOARD_IOID_EMU_VOUT_SEL_1);
  gpio_hal_arch_pin_set_output(BOARD_IOID_EMU_VOUT_SEL_2);
  gpio_hal_arch_pin_set_output(BOARD_IOID_EMU_VOUT_SEL_3);
  gpio_hal_arch_pin_set_output(BOARD_IOID_EMU_VOUT_SEL_4);

  // apply default configuration
  emu_configure(EMU_DEFAULT_BURST, EMU_DEFAULT_VOLTAGE);

  // energy burst wakeup trigger GPIO
  gpio_hal_arch_pin_set_input(BOARD_IOID_EMU_TRIG);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_EMU_TRIG, GPIO_HAL_PIN_CFG_PULL_DOWN);

  // custom comparator output
  gpio_hal_arch_pin_set_input(BOARD_IOID_EMU_COMP);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_EMU_COMP, GPIO_HAL_PIN_CFG_PULL_DOWN);

  // boffer voltage analog in
  // TODO: currently deactivated
  gpio_hal_arch_pin_set_input(BOARD_IOID_EMU_VBUF);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_EMU_VBUF, GPIO_HAL_PIN_CFG_PULL_DOWN);

  // configure wakeup interrupt
  // TODO 
}
/*---------------------------------------------------------------------------*/
void
emu_configure(emu_energy_burst_t energy, emu_output_voltage_t voltage)
{
  LOG_DBG("configure\n");

  // configure burst level
  if (energy != EMU_BURST_UNCHANGED) {
    emu_energy_burst = energy;

    // apply stored setting
    if (emu_energy_burst & EMU_BURST_SELECT1_MASK) {
      gpio_hal_arch_set_pin(BOARD_IOID_EMU_OV_SEL_1);
    } else {
      gpio_hal_arch_clear_pin(BOARD_IOID_EMU_OV_SEL_1);
    }
    if (emu_energy_burst & EMU_BURST_SELECT2_MASK) {
      gpio_hal_arch_set_pin(BOARD_IOID_EMU_OV_SEL_2);
    } else {
      gpio_hal_arch_clear_pin(BOARD_IOID_EMU_OV_SEL_2);
    }
  }

  // configure output voltage
  if (voltage != EMU_VOLTAGE_UNCHANGED) {
    emu_output_voltage = voltage;
    
    // apply stored setting
    if (emu_output_voltage & EMU_VOLTAGE_SELECT1_MASK) {
      gpio_hal_arch_set_pin(BOARD_IOID_EMU_VOUT_SEL_1);
    } else {
      gpio_hal_arch_clear_pin(BOARD_IOID_EMU_VOUT_SEL_1);
    }
    if (emu_output_voltage & EMU_VOLTAGE_SELECT2_MASK) {
      gpio_hal_arch_set_pin(BOARD_IOID_EMU_VOUT_SEL_2);
    } else {
      gpio_hal_arch_clear_pin(BOARD_IOID_EMU_VOUT_SEL_2);
    }
    if (emu_output_voltage & EMU_VOLTAGE_SELECT3_MASK) {
      gpio_hal_arch_set_pin(BOARD_IOID_EMU_VOUT_SEL_3);
    } else {
      gpio_hal_arch_clear_pin(BOARD_IOID_EMU_VOUT_SEL_3);
    }
    if (emu_output_voltage & EMU_VOLTAGE_SELECT4_MASK) {
      gpio_hal_arch_set_pin(BOARD_IOID_EMU_VOUT_SEL_4);
    } else {
      gpio_hal_arch_clear_pin(BOARD_IOID_EMU_VOUT_SEL_4);
    }
  }
}
