/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich)
 * 
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
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
/* Disable button shutdown functionality */
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN  0
/*---------------------------------------------------------------------------*/
/* Enable/Disable UART and system LED I/O */
#define CC26XX_UART_CONF_ENABLE             0
#define CC26XX_LED_CONF_ENABLE              0
/*---------------------------------------------------------------------------*/
/* Internal RCOSC as LF clock sufficient and initialized much faster */
#define CC26XX_CLOCK_CONF_USE_LF_RCOSC      1
/*---------------------------------------------------------------------------*/
/* Enable the ROM bootloader */
#define ROM_BOOTLOADER_ENABLE               1
/*---------------------------------------------------------------------------*/
/* Radio configuration: only BLE on demand */
#define NETSTACK_CONF_RADIO                 nullradio_driver
#define RF_BLE_CONF_ENABLED                 1
/*---------------------------------------------------------------------------*/
/* Logging configuration of system components */
#define LOG_CONF_LEVEL_RPL                  LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_TCPIP                LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_IPV6                 LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_6LOWPAN              LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_NULLNET              LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAC                  LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_FRAMER               LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_6TOP                 LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_COAP                 LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_LWM2M                LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAIN                 LOG_LEVEL_DBG
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
