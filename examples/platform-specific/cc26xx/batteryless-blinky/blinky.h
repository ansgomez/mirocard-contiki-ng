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
/*---------------------------------------------------------------------------*/
#ifndef BATTERYLESS_H_
#define BATTERYLESS_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "board.h"

#include <stdint.h>

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
 * GPIO used as as wakeup trigger (EMU_TRIG or USER_BUTTON for testing)
 */
/****  EMU COMPARATOR IS ACTIVE LOW  *****/
#define WAKEUP_TRIGGER_IOID                 BOARD_IOID_EMU_COMP

/****  LED PIN IS ACTIVE LOW  *****/
// #define WAKEUP_TRIGGER_IOID               BOARD_IOID_LED_1

/****  USER SWITCH IS ACTIVE LOW  *****/
// #define WAKEUP_TRIGGER_IOID               BOARD_IOID_KEY_USER 
/**
 * GPIO trigger edge (IOC_WAKE_ON_HIGH or IOC_WAKE_ON_LOW for parallel testing)
 */
/****  USE THIS DEFINE FOR ACTIVE HIGH WAKEUP  *****/
#define WAKEUP_TRIGGER_EDGE                 IOC_WAKE_ON_HIGH

/****  USE THIS DEFINE FOR ACTIVE LOW WAKEUP  *****/
// #define WAKEUP_TRIGGER_EDGE                 IOC_WAKE_ON_LOW
/*---------------------------------------------------------------------------*/
#endif /* BATTERYLESS_H_ */
/*---------------------------------------------------------------------------*/
