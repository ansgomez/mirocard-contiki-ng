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
/**
 * \addtogroup transient-peripherals
 * @{
 *
 * \file
 *  Transient Node specific board initialisation driver
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lpm.h"
#include "ti-lib.h"
#include "board-peripherals.h"
#include "board-i2c.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
static void
board_gpio_shutdown(void)
{
  /* configure communication interface pin states for shutdown */
  // I2C bus pin configuration for shutdown
  gpio_hal_arch_pin_set_input(BOARD_IOID_I2C_SCL);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_I2C_SCL, GPIO_HAL_PIN_CFG_PULL_UP);
  gpio_hal_arch_pin_set_input(BOARD_IOID_I2C_SDA);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_I2C_SDA, GPIO_HAL_PIN_CFG_PULL_UP);

  // UART pin configuration for shutdown
  gpio_hal_arch_pin_set_input(BOARD_IOID_UART_RX);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_UART_RX, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_UART_TX);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_UART_TX, GPIO_HAL_PIN_CFG_PULL_DOWN);

  // FRAM SPI pin configuration for shutdown
  gpio_hal_arch_pin_set_input(BOARD_IOID_FRAM_SCK);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_FRAM_SCK, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_FRAM_MOSI);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_FRAM_MOSI, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_FRAM_MISO);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_FRAM_MISO, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_FRAM_CS);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_FRAM_CS, GPIO_HAL_PIN_CFG_PULL_UP);

  // RTC SPI pin configuration for shutdown
  gpio_hal_arch_pin_set_input(BOARD_IOID_AM0815_SCK);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_AM0815_SCK, GPIO_HAL_PIN_CFG_PULL_UP);
  gpio_hal_arch_pin_set_input(BOARD_IOID_AM0815_MOSI);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_AM0815_MOSI, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_AM0815_MISO);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_AM0815_MISO, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_AM0815_CS);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_AM0815_CS, GPIO_HAL_PIN_CFG_PULL_UP);

  // SHT3x GPIOs
  gpio_hal_arch_pin_set_input(BOARD_IOID_SHT_RESET);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_SHT_RESET, GPIO_HAL_PIN_CFG_PULL_UP);
  gpio_hal_arch_pin_set_input(BOARD_IOID_SHT_ALERT);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_SHT_ALERT, GPIO_HAL_PIN_CFG_PULL_NONE);

  // RTC GPIOs
  gpio_hal_arch_pin_set_input(BOARD_IOID_AM0815_CHARGE);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_AM0815_CHARGE, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_AM0815_IRQ);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_AM0815_IRQ, GPIO_HAL_PIN_CFG_PULL_NONE);

  // General Purpose I/O
  gpio_hal_arch_pin_set_input(BOARD_IOID_GPIO_1);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_GPIO_1, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_GPIO_2);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_GPIO_2, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_GPIO_3);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_GPIO_3, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(BOARD_IOID_GPIO_4);
  gpio_hal_arch_pin_cfg_set(BOARD_IOID_GPIO_4, GPIO_HAL_PIN_CFG_PULL_DOWN);
}
/*---------------------------------------------------------------------------*/
static void
wakeup_handler(void)
{
  /* Turn on the PERIPH PD */
  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
  while(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH)
        != PRCM_DOMAIN_POWER_ON);

  /* init I2C controller */
  board_i2c_wakeup();
}
/*---------------------------------------------------------------------------*/
static void
shutdown_handler(uint8_t mode)
{
  // shutdown I2C controller
  board_i2c_shutdown();

  // shutdown GPIOs
  board_gpio_shutdown();
}
/*---------------------------------------------------------------------------*/
/*
 * Declare a data structure to register with LPM.
 * We don't care about what power mode we'll drop to, we don't care about
 * getting notified before deep sleep. All we need is to be notified when we
 * wake up so we can turn power domains back on
 */
LPM_MODULE(transient_module, NULL, shutdown_handler, wakeup_handler, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static void
configure_unused_pins(void)
{
  uint32_t pins[] = BOARD_UNUSED_PINS;

  uint32_t *pin;

  for(pin = pins; *pin != IOID_UNUSED; pin++) {
    ti_lib_ioc_pin_type_gpio_input(*pin);
    ti_lib_ioc_io_port_pull_set(*pin, IOC_IOPULL_DOWN);
  }
}
/*---------------------------------------------------------------------------*/
void
board_init()
{
  /* Disable global interrupts */
  bool int_disabled = ti_lib_int_master_disable();

  /* Turn on relevant PDs */
  wakeup_handler();

  /* Enable GPIO peripheral */
  ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

  /* Apply settings and wait for them to take effect */
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* Set GPIOs to default state */
  board_gpio_shutdown();

  /* initialize external FRAM memory */
  // ext_fram_open(NULL); /* init without sending to sleep */
  // ext_fram_init(NULL); /* init and send to sleep */

  /* initialize AM0815 RTC */
  //am0815_init(NULL);

  // configure active GPIO states different from shutdown (i.e. outputs) */
  // RTC backup buffer charge switch
  gpio_hal_arch_pin_set_output(BOARD_IOID_AM0815_CHARGE);
  gpio_hal_arch_clear_pin(BOARD_IOID_AM0815_CHARGE);

  // general purpose GPIO_x
  gpio_hal_arch_pin_set_output(BOARD_IOID_GPIO_1);
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_1);
  gpio_hal_arch_pin_set_output(BOARD_IOID_GPIO_2);
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_2);
  gpio_hal_arch_pin_set_output(BOARD_IOID_GPIO_3);
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_3);
  gpio_hal_arch_pin_set_output(BOARD_IOID_GPIO_4);
  gpio_hal_arch_clear_pin(BOARD_IOID_GPIO_4);

  /* register LPM module to restore peripheral power on wakeup */
  lpm_register_module(&transient_module);

  /* For unsupported peripherals, select a default pin configuration */
  configure_unused_pins();

  /* Re-enable interrupt if initially enabled. */
  if(!int_disabled) {
    ti_lib_int_master_enable();
  }
}
/*---------------------------------------------------------------------------*/
/** @} */
