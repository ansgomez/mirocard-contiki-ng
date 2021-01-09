/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
 * Copyright (c) 2020, Miromico AG - http://www.miromico.ch/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup batteryless-peripherals
 * @{
 *
 * \defgroup batteryless-cc26xx-specific CC2650 Batteryless Node Peripherals
 *
 * Defines related to the CC2650 Batteryless Node
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * CC2650 Batteryless Node
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED HAL configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define LEDS_CONF_COUNT           3
#define LEDS_CONF_GREEN           0
#define LEDS_CONF_RED             1
#define LEDS_CONF_BLUE            2
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_11
#define BOARD_IOID_LED_2          IOID_18
#define BOARD_IOID_LED_3          IOID_19
#define BOARD_LED_ALL             ((1 << LEDS_CONF_COUNT) - 1)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_KEY_USER       IOID_10
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name State indicator pin IOID mappings
 *
 * PIN configuration for the use of external state indicator pins
 * @{
 */
#define BOARD_IOID_GPIO_1         IOID_7
#define BOARD_IOID_GPIO_2         IOID_13
#define BOARD_IOID_GPIO_3         IOID_14
#define BOARD_IOID_GPIO_4         IOID_15
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_2
#define BOARD_IOID_UART_TX        IOID_3
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_IOID_UART_CTS       IOID_UNUSED
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief EMU interface IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_EMU_STATUS     IOID_24
#define BOARD_IOID_EMU_COMP       IOID_25
#define BOARD_IOID_EMU_VBUF       IOID_26
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief MPU IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */

#define BOARD_IOID_I2C2_SDA_HP         IOID_8 /**< Interface 1 SDA: MPU */
#define BOARD_IOID_I2C2_SCL_HP         IOID_9 /**< Interface 1 SCL: MPU */
#define BOARD_IOID_MPU_INT        IOID_23
#define BOARD_IOID_MPU_POWER      IOID_12
#define BOARD_MPU_INT             (1 << BOARD_IOID_MPU_INT)
#define BOARD_MPU_POWER           (1 << BOARD_IOID_MPU_POWER)

#define BOARD_IOID_SDA_HP BOARD_IOID_I2C2_SDA_HP
#define BOARD_IOID_SCL_HP BOARD_IOID_I2C2_SCL_HP

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_I2C_SCL        IOID_0
#define BOARD_IOID_I2C_SDA        IOID_1

#define BOARD_IOID_SDA BOARD_IOID_I2C_SDA
#define BOARD_IOID_SCL BOARD_IOID_I2C_SCL
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief ROM bootloader configuration
 *
 * Change CCXXWARE_CONF_BL_PIN_NUMBER to BOARD_IOID_KEY_xyz to select which
 * button triggers the bootloader on reset. Use CCXXWARE_CONF_BL_LEVEL to
 * control the pin level that enables the bootloader (0: low, 1: high). It is
 * also possible to use any other externally-controlled DIO.
 * @{
 */
#define CCXXWARE_CONF_BL_PIN_NUMBER   BOARD_IOID_KEY_USER
#define CCXXWARE_CONF_BL_LEVEL        0
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Remaining pins
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_TDO            IOID_16
#define BOARD_IOID_TDI            IOID_17

#define BOARD_UNUSED_PINS { \
    BOARD_IOID_TDO, BOARD_IOID_TDI, \
    IOID_4, IOID_5, IOID_6, IOID_20, IOID_22, \
    IOID_UNUSED \
  }
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Board indices for the button HAL
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_BUTTON_HAL_INDEX_KEY_USER 0x00
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "MiroCard Board"

/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
