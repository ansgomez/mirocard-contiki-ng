/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
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
#define LEDS_CONF_COUNT                 1
#define LEDS_CONF_GREEN                 1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_6
#define BOARD_LED_ALL             BOARD_IOID_LED_1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_KEY_USER       IOID_24
#define BOARD_KEY_USER            (1 << BOARD_IOID_KEY_USER)
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
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief EMU interface IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_EMU_VSEL_1     IOID_UNUSED
#define BOARD_IOID_EMU_VSEL_2     IOID_UNUSED
#define BOARD_IOID_EMU_VSEL_3     IOID_UNUSED
#define BOARD_IOID_EMU_VSEL_4     IOID_UNUSED
#define BOARD_IOID_EMU_ESEL_1     IOID_UNUSED
#define BOARD_IOID_EMU_ESEL_2     IOID_UNUSED
#define BOARD_IOID_EMU_TRIG       IOID_UNUSED
#define BOARD_IOID_EMU_COMP       IOID_UNUSED
#define BOARD_IOID_EMU_VBUF       IOID_UNUSED
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI IOID mapping
 *
 * @{
 */
#define EXT_RTC_SPI_CONTROLLER    SPI_CONTROLLER_SPI0
#define BOARD_IOID_RTC_SCK        IOID_21
#define BOARD_IOID_RTC_MOSI       IOID_18
#define BOARD_IOID_RTC_MISO       IOID_22
#define BOARD_IOID_RTC_CS         IOID_20
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name RTC IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_RTC_IRQ        IOID_19
#define BOARD_IOID_RTC_CHARGE     IOID_14
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name External FRAM IOID mapping (SPI)
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define EXT_FRAM_SPI_CONTROLLER   SPI_CONTROLLER_SPI0
#define BOARD_IOID_FRAM_SCK       IOID_9
#define BOARD_IOID_FRAM_MOSI      IOID_8
#define BOARD_IOID_FRAM_MISO      IOID_11
#define BOARD_IOID_FRAM_CS        IOID_10
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_I2C_SCL        IOID_4
#define BOARD_IOID_I2C_SDA        IOID_5
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SHT31 sensor IOID mappings
 *
 * PIN configuration for the use of external pins, to be used with the RocketLogger
 * @{
 */
#define BOARD_IOID_SHT_RESET      IOID_6
#define BOARD_IOID_SHT_ALERT      IOID_7
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
#define BOARD_IOID_DIO0           IOID_0
#define BOARD_IOID_DIO1           IOID_1
// #define BOARD_IOID_DIO2          IOID_2  // UART
// #define BOARD_IOID_DIO3          IOID_3  // UART
#define BOARD_IOID_DIO4           IOID_4
#define BOARD_IOID_DIO5           IOID_5
//#define BOARD_IOID_DIO6           IOID_6  //SHT_RESET
//#define BOARD_IOID_DIO7           IOID_7  //SHT_ALERT
//#define BOARD_IOID_FRAM_MISO      IOID_8  //FRAM
//#define BOARD_IOID_FRAM_MOSI      IOID_9  //FRAM
//#define BOARD_IOID_FRAM_SCK       IOID_10 //FRAM
//#define BOARD_IOID_FRAM_CS        IOID_11 //FRAM
//#define BOARD_IOID_DIO12          IOID_12 //GPIO12
//#define BOARD_IOID_DIO13          IOID_13 //GPIO13
//#define BOARD_IOID_DIO14          IOID_14 //RTC Charging
#define BOARD_IOID_DIO15          IOID_15
#define BOARD_IOID_TDO            IOID_16
#define BOARD_IOID_TDI            IOID_17
//#define BOARD_IOID_DIO18          IOID_18 //RTC_CS
//#define BOARD_IOID_DIO19          IOID_19 //RTC_IRQ
//#define BOARD_IOID_DIO20          IOID_20 //RTC_SCK
//#define BOARD_IOID_DIO21          IOID_21 //RTC_MISO
//#define BOARD_IOID_DIO22          IOID_22 //RTC_MOSI
//#define BOARD_IOID_DIO23          IOID_23
//#define BOARD_IOID_DIO24          IOID_24
#define BOARD_IOID_DIO25          IOID_25
#define BOARD_IOID_DIO26          IOID_26
#define BOARD_IOID_DIO27          IOID_27
#define BOARD_IOID_DIO28          IOID_28
#define BOARD_IOID_DIO29          IOID_29
#define BOARD_IOID_DIO30          IOID_30

#define BOARD_UNUSED_PINS { \
    BOARD_IOID_DIO0, BOARD_IOID_DIO1, \
    BOARD_IOID_DIO15, \
    BOARD_IOID_TDO, BOARD_IOID_TDI, \
    BOARD_IOID_DIO25, BOARD_IOID_DIO26, BOARD_IOID_DIO27, \
    BOARD_IOID_DIO28, BOARD_IOID_DIO29, BOARD_IOID_DIO30, \
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
#define BOARD_STRING "TI CC2650 Transient"

/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
