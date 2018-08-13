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
#ifndef EMU_H_
#define EMU_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"

#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define emu_set_voltage(voltage)  emu_configure(EMU_BURST_UNCHANGED, voltage)
#define emu_set_burst(energy)     emu_configure(energy, EMU_VOLTAGE_UNCHANGED)
/*---------------------------------------------------------------------------*/
/**
 * EMU energy burst size definitions.
 */
typedef enum {
  EMU_BURST_SMALL = 0x00,
  EMU_BURST_MEDIUM = 0x01,
  EMU_BURST_LARGE = 0x02,
  EMU_BURST_MAX = 0x03,
  EMU_BURST_UNCHANGED = 0xFF,
} emu_energy_burst_t;

/**
 * EMU output voltage burst size definitions.
 */
typedef enum {
  EMU_VOLTAGE_1_8V = 0x00,
  EMU_VOLTAGE_1_9V = 0x01,
  EMU_VOLTAGE_2_0V = 0x02,
  EMU_VOLTAGE_2_1V = 0x03,
  EMU_VOLTAGE_2_2V = 0x04,
  EMU_VOLTAGE_2_3V = 0x05,
  EMU_VOLTAGE_2_4V = 0x06,
  EMU_VOLTAGE_2_5V = 0x07,
  EMU_VOLTAGE_2_6V = 0x08,
  EMU_VOLTAGE_2_7V = 0x09,
  EMU_VOLTAGE_2_8V = 0x0A,
  EMU_VOLTAGE_2_9V = 0x0B,
  EMU_VOLTAGE_3_0V = 0x0C,
  EMU_VOLTAGE_3_1V = 0x0D,
  EMU_VOLTAGE_3_2V = 0x0E,
  EMU_VOLTAGE_3_3V = 0x0F,
  EMU_VOLTAGE_UNCHANGED = 0xFF,
} emu_output_voltage_t;
/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize the Energy Managment Unit interface.
 */
void emu_init(void);

/**
 * \brief Configure the Energy Managment Unit in one command.
 * \param energy Sets the energy level at which the EMU triggers a wakeup
 * \param voltage Set the output voltage for the application circuit
 */
void emu_configure(emu_energy_burst_t energy, emu_output_voltage_t voltage);
/*---------------------------------------------------------------------------*/
#endif /* EMU_H_ */
/*---------------------------------------------------------------------------*/
