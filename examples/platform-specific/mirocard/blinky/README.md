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

# Batteryless BLE Node

Baseline contiki-ng application for Miromico's batteryless sensor node with BLE beacon broadcasting.


## Getting Started

To build the project, make sure the [required software](https://github.com/contiki-ng/contiki-ng/wiki/Platform-srf06-cc26xx#requirements) is installed before you continue.


The command to compile the project is:
```bash
make TARGET=cc26x0-cc13x0 BOARD=mirocard/cc2650
```


For flashing the binary there are multiple options, see the [contiki-ng wiki](https://github.com/contiki-ng/contiki-ng/wiki/Platform-srf06-cc26xx#how-to-program-your-device) for detailed explanation. Initial flashing needs to be done via JTAG to flash the bootloader code. Afterwards the bootloader option is available: to compile and flash the binary in one command, you may use the following do-all-in-one command (replace `/dev/ttyUSB0` with your device):
```bash
make TARGET=cc26x0-cc13x0 BOARD=mirocard/cc2650 PORT=/dev/ttyUSB0 batteryless.upload
```

Sometimes the build system hangs up. In that case a complete cleanup and rebuild (also in the contiki tree) usually helps.


## Component Characteristics

All numbers are the typical values at room temperature taken from the respective datasheet, if not noted otherwise.


### Supply Voltage and Currents

| Component     | VCC (min) | VCC (max) | ICC (standby) | ICC (sleep)   |
|:--------------|----------:|----------:|--------------:|--------------:|
| CC2650        |     1.8 V |     3.8 V |          1 uA |        150 nA |
| SHTC3         |    1.62 V |     3.6 V |        0.3 uA |           N/A |
| BME280        |     1.8 V |     3.6 V |        0.2 uA |        0.1 uA |
| MPU-9250      |     2.4 V |     3.6 V |          8 uA |            0* |

> * The MPU is supplied by a GPIO and can be power gated
> Common supported supply range for *all* peripherals: 2.3-3.3 V

> Expected power down current: <10 uA
> Measured power down current:  2.47uA
> Notes on measured power: setup described on: A Gomez. "On-demand communication with the batteryless MiroCard: demo abstract." Proc. SenSys Conf. 2020.


### Start-Up Timing

| Component/State Transition            |    Time (typ) |    Time (max) |
|:--------------------------------------|--------------:|--------------:|
| CC2650: Idle - Active                 |         14 us |           N/A |
| CC2650: Standby - Active              |        151 us |           N/A |
| CC2650: Shutdown - Active             |       1015 us |           N/A |
| CC2650: 24 MHz XOSC_HF startup        |        150 us |           N/A |
| CC2650: 48 MHz RCOSC_HF startup       |          5 us |           N/A |
| CC2650: 32 kHz XOSC_LF startup        |           N/A |           N/A |
|                                                                       |
|                                                                       |
| SHT3X: power up                       |        0.5 ms |          1 ms |
| SHT3X: soft reset                     |        0.5 ms |          1 ms |
| SHT3X: measure low repeatability      |        2.5 ms |          4    |
| SHT3X: measure med repeatability      |        4.5 ms |          6 ms |
| SHT3X: measure high repeatability     |       12.5 ms |         15 ms |
|                                                                       |
|                                                                       |
| BME280: power up                      |           N/A |          2 ms |
| BME280: measurement time calculation see Appendix B in datasheet      |

## System Configuration

### GPIO Assignment and Reset State (HW v1.0)

| I/O Pin   | Function Description  | Active State      | Shutdown State    |
|:----------|:----------------------|:------------------|:------------------|
| DIO_0     | I2C SCL               | Peripheral out    | IN  - Pull Up     |
| DIO_1     | I2C SDA               | Peripheral in/out | IN  - Pull Up     |
| DIO_2     | UART_RXD              | Peripheral in     | IN  - Pull Down   |
| DIO_3     | UART_TXD              | Peripheral out    | IN  - Pull Down   |
| DIO_4     | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_5     | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_6     | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_7     | GPIO_1                | OUT - High/Low    | IN  - Pull Up     |
| DIO_8     | MPU9250_SDA           | Peripheral in/out | IN  - Pull Down   |
| DIO_9     | MPU9250_SCL           | Peripheral out    | IN  - No Pull     |
| DIO_10    | User button           | IN  - Pull Up     | IN  - Pull Up     |
| DIO_11    | LED_GREEN             | OUT - High/Low    | IN  - Pull Down   |
| DIO_12    | MPU_VDD               | OUT - High/Low    | OUT - Low         |
| DIO_13    | GPIO_2                | OUT - High/Low    | IN  - Pull Down   |
| DIO_14    | GPIO_3                | OUT - High/Low    | IN  - Pull Down   |
| DIO_15    | GPIO_4                | OUT - High/Low    | IN  - Pull Down   |
| DIO_16    | JTAG TDO pin          | system            | system            |
| DIO_17    | JTAG TDI pin          | system            | system            |
| DIO_18    | LED_RED               | OUT - High/Low    | OUT - High/Low    |
| DIO_19    | LED_BLUE              | OUT - High/Low    | OUT - High/Low    |
| DIO_20    | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_21    | HALL_OUT              | IN  - No Pull     | IN  - No Pull     |
| DIO_22    | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_23    | MPU_INT (trigger)     | IN  - No Pull     | IN  - Pull Down   |
| DIO_24    | EMU STATUS            | IN  - No Pull     | IN  - No Pull     |
| DIO_25    | EMU wakeup comparator | IN  - No Pull     | IN  - No Pull     |
| DIO_26    | EMU buffer voltage    | IN  - No Pull     | IN  - No Pull     |
| DIO_27    | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_28    | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_29    | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_30    | unused                | IN  - Pull Down   | IN  - Pull Down   |

### BLE packet data

The bytes of a BLE packet are organized as follows
- Byte `0`: The length of the following data (excluding this byte)
- Byte `1`: BLE Manufacturer-Specific Data Flag `0xFF`
- Byte `2`: BLE Company ID LSB used for current system status flag
- Bytes `3-30`: 1 up to 4 data units using the same binary data format as for FRAM.
