/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
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
/** \addtogroup transient-peripherals
 * @{
 *
 * \defgroup transient-cc26xx-specific CC2650 Transient Node Peripherals
 *
 * Defines related to the CC2650 Transient Node
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * CC2650 Transient Node
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
#define LEDS_CONF_COUNT           1
#define LEDS_CONF_GREEN           1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_11
#define BOARD_LED_ALL             BOARD_IOID_LED_1
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
 * \name State FLAG IOID mappings
 *
 * PIN configuration for the use of external state indicator pins
 * @{
 */
#define BOARD_IOID_GPIO_1         IOID_12
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
#define BOARD_IOID_EMU_VSEL_1     IOID_18
#define BOARD_IOID_EMU_VSEL_2     IOID_19
#define BOARD_IOID_EMU_VSEL_3     IOID_20
#define BOARD_IOID_EMU_VSEL_4     IOID_21
#define BOARD_IOID_EMU_ESEL_1     IOID_22
#define BOARD_IOID_EMU_ESEL_2     IOID_23
#define BOARD_IOID_EMU_TRIG       IOID_24
#define BOARD_IOID_EMU_COMP       IOID_25
#define BOARD_IOID_EMU_VBUF       IOID_26
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI IOID mapping
 *
 * @{
 */
#define EXT_RTC_SPI_CONTROLLER    SPI_CONTROLLER_SPI0
#define BOARD_IOID_RTC_SCK        IOID_4
#define BOARD_IOID_RTC_MOSI       IOID_5
#define BOARD_IOID_RTC_MISO       IOID_6
#define BOARD_IOID_RTC_CS         IOID_7
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name RTC IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_RTC_CHARGE     IOID_8
#define BOARD_IOID_RTC_IRQ        IOID_9
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name External FRAM IOID mapping (SPI)
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define EXT_FRAM_SPI_CONTROLLER   SPI_CONTROLLER_SPI1
#define BOARD_IOID_FRAM_SCK       IOID_27
#define BOARD_IOID_FRAM_MOSI      IOID_28
#define BOARD_IOID_FRAM_MISO      IOID_29
#define BOARD_IOID_FRAM_CS        IOID_30
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
/** @} */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief ROM bootloader configuration
 *
 * Change SET_CCFG_BL_CONFIG_BL_PIN_NUMBER to BOARD_IOID_KEY_xyz to select
 * which button triggers the bootloader on reset.
 *
 * The remaining values are not meant to be modified by the user
 * @{
 */
#if ROM_BOOTLOADER_ENABLE
#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE            0xC5
#define SET_CCFG_BL_CONFIG_BL_LEVEL                     0x00
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER                BOARD_IOID_KEY_USER
#define SET_CCFG_BL_CONFIG_BL_ENABLE                    0xC5
#else
#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE            0x00
#define SET_CCFG_BL_CONFIG_BL_LEVEL                     0x01
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER                0xFF
#define SET_CCFG_BL_CONFIG_BL_ENABLE                    0xFF
#endif
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
#define BOARD_STRING "TI CC2650 Transient Core"

/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */