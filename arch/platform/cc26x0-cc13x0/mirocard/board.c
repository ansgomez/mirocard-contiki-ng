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
/**
 * \addtogroup batteryless-peripherals
 * @{
 *
 * \file
 *  Batteryless Node specific board initialisation driver
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
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
  //SENSORS 
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
  ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_MPU_INT);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_MPU_INT, IOC_NO_IOPULL);
  //TODO: Hall Sensor
  ti_lib_ioc_pin_type_gpio_input(IOID_21);
  ti_lib_ioc_io_port_pull_set(IOID_21, IOC_NO_IOPULL);

  //EMU
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_EMU_STATUS);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_EMU_STATUS, IOC_NO_IOPULL);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_EMU_COMP);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_EMU_COMP, IOC_NO_IOPULL);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_EMU_VBUF);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_EMU_VBUF, IOC_NO_IOPULL);

  /* configure communication interface pin states for shutdown */
  // I2C bus pin configuration for shutdown
  //I2C1 (Temp/RH/Light sensor) -> Vdd is same as MCU (always high)
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_I2C_SCL);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_I2C_SCL, IOC_IOPULL_UP);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_I2C_SDA);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_I2C_SDA, IOC_IOPULL_UP);
  //I2C2 (MPU) -> Vdd is a GPIO. When off, pull to GND
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_I2C2_SDA_HP);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_I2C2_SDA_HP, IOC_IOPULL_DOWN);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_I2C2_SCL_HP);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_I2C2_SCL_HP, IOC_IOPULL_DOWN);

  // UART pin configuration for shutdown
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_UART_RX);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_UART_RX, IOC_IOPULL_DOWN);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_UART_TX);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_UART_TX, IOC_IOPULL_DOWN);

  // General Purpose I/O
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_GPIO_1);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_GPIO_1, IOC_IOPULL_DOWN);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_GPIO_2);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_GPIO_2, IOC_IOPULL_DOWN);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_GPIO_3);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_GPIO_3, IOC_IOPULL_DOWN);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_GPIO_4);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_GPIO_4, IOC_IOPULL_DOWN);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_KEY_USER);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_KEY_USER, IOC_IOPULL_UP);
  

  /* LEDS:  */
#ifdef MIROCARD_BATTERYLESS
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_LED_1);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_LED_1, IOC_IOPULL_UP);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_LED_2);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_LED_2, IOC_IOPULL_UP);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_LED_3);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_LED_3, IOC_IOPULL_UP);
#else 
#pragma message "ALLOWING LEDS TO BE ON DURING SLEEP"
#pragma message "NOT LOW POWER"
#endif
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
  if(mode == LPM_MODE_SHUTDOWN) {
    SENSORS_DEACTIVATE(opt_3001_sensor);
    SENSORS_DEACTIVATE(mpu_9250_sensor);
    SENSORS_DEACTIVATE(sht3x_sensor);
    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
    ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
  }

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
LPM_MODULE(batteryless_module, NULL, shutdown_handler, wakeup_handler, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static void
configure_unused_pins(void)
{
  uint32_t pins[] = BOARD_UNUSED_PINS;

  uint32_t *pin;

  for(pin = pins; *pin != IOID_UNUSED; pin++) {
    ti_lib_ioc_pin_type_gpio_input(*pin);
    ti_lib_ioc_io_port_pull_set(*pin, IOC_IOPULL_DOWN);
    // ti_lib_ioc_io_port_pull_set(*pin, IOC_NO_IOPULL);
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

  // general purpose GPIO_x
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_GPIO_1);
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_1);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_GPIO_2);
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_2);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_GPIO_3);
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_3);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_GPIO_4);
  ti_lib_gpio_clear_dio(BOARD_IOID_GPIO_4);

  /* register LPM module to restore peripheral power on wakeup */
  lpm_register_module(&batteryless_module);

  /* For unsupported peripherals, select a default pin configuration */
  configure_unused_pins();

  /* Re-enable interrupt if initially enabled. */
  if(!int_disabled) {
    ti_lib_int_master_enable();
  }
}
/*---------------------------------------------------------------------------*/
/** @} */
