/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef EXT_FRAM_H_
#define EXT_FRAM_H_
/*---------------------------------------------------------------------------*/
#include "dev/spi.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize storage driver.
 * \param conf SPI bus configuration struct. NULL for default.
 * \return True when successful.
 * 
 * \note A delay of 400us needs to be waited before reading/writing when the
 * device was in sleep mode (default after init). Manually pulling the CS line
 * low after this function call enables faster wakeup without a dummy read.
 */
bool ext_fram_open(spi_device_t *conf);

/**
 * \brief Close the storage driver
 * \param conf SPI bus configuration struct. NULL for default.
 * \return True when successful.
 *
 * This call will put the device in its lower power mode (power down).
 */
bool ext_fram_close(spi_device_t *conf);

/**
 * \brief Read storage content
 * \param conf SPI bus configuration struct. NULL for default.
 * \param offset Address to read from
 * \param length Number of bytes to read
 * \param buf Buffer where to store the read bytes
 * \return True when successful.
 *
 * buf must be allocated by the caller
 */
bool ext_fram_read(spi_device_t *conf, uint32_t offset, uint32_t length, uint8_t *buf);

/**
 * \brief Erase storage sectors corresponding to the range.
 * \param conf SPI bus configuration struct. NULL for default.
 * \param offset Address to start erasing
 * \param length Number of bytes to erase
 * \return True when successful.
 *
 * The erase operation will be sector-wise, therefore a call to this function
 * will generally start the erase procedure at an address lower than offset
 */
bool ext_fram_erase(spi_device_t *conf, uint32_t offset, uint32_t length);

/**
 * \brief Write to storage sectors.
 * \param conf SPI bus configuration struct. NULL for default.
 * \param offset Address to write to
 * \param length Number of bytes to write
 * \param buf Buffer holding the bytes to be written
 *
 * \return True when successful.
 */
bool ext_fram_write(spi_device_t *conf, uint32_t offset, uint32_t length, const uint8_t *buf);

/**
 * \brief Initialise the external flash
 * \param conf SPI bus configuration struct. NULL for default.
 *
 * This function will explicitly put the part in its lowest power mode
 * (power-down).
 *
 * In order to perform any operation after this routine, the caller has to wake
 * up the device calling ext_fram_open() first.
 */
bool ext_fram_init(spi_device_t *conf);
/*---------------------------------------------------------------------------*/
#endif /* EXT_FRAM_H_ */
/*---------------------------------------------------------------------------*/
