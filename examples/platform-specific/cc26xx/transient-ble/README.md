# Transient BLE Node

Baseline contiki-ng application for the EMU powered transiently powered ambient sensor node with BLE beacon broadcasting.


## Getting Started

To build the project, make sure the [required software](https://github.com/contiki-ng/contiki-ng/wiki/Platform-srf06-cc26xx#requirements) is installed before you continue.


The command to compile the project is:
```bash
make TARGET=srf06-cc26xx BOARD=transient/cc2650
```

You can specify to compile with a specific policy using the `POLICY` parameters, e.g.
```bash
make TARGET=srf06-cc26xx BOARD=transient/cc2650 POLICY=policies/pool_stochastic_60.pkl
```

For flashing the binary there are multiple options, see the [contiki-ng wiki](https://github.com/contiki-ng/contiki-ng/wiki/Platform-srf06-cc26xx#how-to-program-your-device) for detailed explanation. Initial flashing needs to be done via JTAG to flash the bootloader code. Afterwards the bootloader option is available: to compile and flash the binary in one command, you may use the following do-all-in-one command (replace `/dev/ttyUSB0` with your device):
```bash
make TARGET=srf06-cc26xx BOARD=transient/cc2650 POLICY=policies/pool_stochastic_60.pkl PORT=/dev/ttyUSB0 transient.upload
```

Sometimes the build system hangs up. In that case a complete cleanup and rebuild (also in the contiki tree) usually helps.


## Component Characteristics

All numbers are the typical values at room temperature taken from the respective datasheet, if not noted otherwise.


### Supply Voltage and Currents

| Component     | VCC (min) | VCC (max) | ICC (standby) | ICC (sleep)   |
|:--------------|----------:|----------:|--------------:|--------------:|
| CC2650        |     1.8 V |     3.8 V |          1 uA |        150 nA |
| AM0815        |     1.6 V |     3.6 V |         55 nA |         14 nA |
| FM25V10       |     2.0 V |     3.6 V |         90 uA |          5 uA |
| SHT3x         |    2.15 V |     5.5 V |        0.2 uA |           N/A |
| TSL45315      |     2.3 V |     3.3 V |        2.2 uA |           N/A |
| BME280        |     1.8 V |     3.6 V |        0.2 uA |        0.1 uA |

> Common supported supply range for *all* peripherals: 2.3-3.3 V

> Expected power down current: <10 uA


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
| AM0815: power up                      |         0.5 s |           N/A |
| AM0815: XT OSC start                  |         0.4 s |           N/A |
|                                                                       |
| FM25V10: power up                     |        250 us |           N/A |
| FM25V10: sleep recovery               |           N/A |        400 us |
|                                                                       |
| SHT3x: power up                       |        0.5 ms |          1 ms |
| SHT3x: soft reset                     |        0.5 ms |          1 ms |
| SHT3x: measure low repeatability      |        2.5 ms |          4 ms |
| SHT3x: measure med. repeatability     |        4.5 ms |          6 ms |
| SHT3x: measure high repeatability     |       12.5 ms |         15 ms |
|                                                                       |
| TSL45315: power up                    |           N/A |           N/A |
| TSL45315: measurement (100ms int.)    |        115 ms |      119.6 ms |
| TSL45315: measurement (200ms int.)    |        230 ms |      239.2 ms |
| TSL45315: measurement (400ms int.)    |        460 ms |      478.4 ms |
|                                                                       |
| BME280: power up                      |           N/A |          2 ms |
| BME280: measurement time calculation see Appendix B in datasheet      |



## System Configuration

### GPIO Assignment and Reset State (HW bugfix v1.1)

| I/O Pin   | Function Description  | Active State      | Shutdown State    |
|:----------|:----------------------|:------------------|:------------------|
| DIO_0     | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_1     | unused                | IN  - Pull Down   | IN  - Pull Down   |
| DIO_2     | UART RXD              | Peripheral in     | IN  - Pull Down   |
| DIO_3     | UART TXD              | Peripheral out    | IN  - Pull Down   |
| DIO_4     | I2C SCL               | Peripheral out    | IN  - Pull Up     |
| DIO_5     | I2C SDA               | Peripheral in/out | IN  - Pull Up     |
| DIO_6     | SHT31 reset           | IN  - Pull Up     | IN  - Pull Up     |
| DIO_7     | SHT31 alert           | IN  - No Pull     | IN  - No Pull     |
| DIO_8     | FRAM SPI MOSI         | Peripheral out    | IN  - Pull Down   |
| DIO_9     | FRAM SPI SCK          | Peripheral out    | IN  - Pull Down   |
| DIO_10    | FRAM SPI CS           | OUT - High        | IN  - Pull Up     |
| DIO_11    | FRAM SPI MISO         | Peripheral in     | IN  - Pull Down   |
| DIO_12    | GPIO_1                | OUT - High/Low    | IN  - Pull Down   |
| DIO_13    | GPIO_2                | OUT - High/Low    | IN  - Pull Down   |
| DIO_14    | AM0815 RTC CHARGE     | OUT - Low         | IN  - Pull Down   |
| DIO_15    | unused (EMU wakeup)   | IN  - Pull Down   | IN  - Pull Down   |
| DIO_16    | JTAG TDO pin          | system            | system            |
| DIO_17    | JTAG TDI pin          | system            | system            |
| DIO_18    | AM0815 RTC SPI MOSI   | Peripheral out    | IN  - Pull Down   |
| DIO_19    | AM0815 RTC IRQ        | IN  - No Pull     | IN  - No Pull     |
| DIO_20    | AM0815 RTC SPI CS     | PUT - High        | IN  - Pull Up     |
| DIO_21    | AM0815 RTC SPI SCK    | Peripheral out    | IN  - Pull Up     |
| DIO_22    | AM0815 RTC SPI MISO   | Peripheral in     | IN  - Pull Down   |
| DIO_23    | LED_1 (trigger)       | IN  - Pull Down   | IN  - Pull Down   |
| DIO_24    | User button           | IN  - Pull Up     | IN  - Pull Up     |
| DIO_25    | unused (EMU config)   | IN  - Pull Down   | IN  - Pull Down   |
| DIO_26    | unused (EMU config)   | IN  - Pull Down   | IN  - Pull Down   |
| DIO_27    | unused (EMU config)   | IN  - Pull Down   | IN  - Pull Down   |
| DIO_28    | unused (EMU config)   | IN  - Pull Down   | IN  - Pull Down   |
| DIO_29    | unused (EMU config)   | IN  - Pull Down   | IN  - Pull Down   |
| DIO_30    | unused (EMU config)   | IN  - Pull Down   | IN  - Pull Down   |


### GPIO Assignment and Reset State (HW v2.0)

| I/O Pin   | Function Description  | Active State      | Shutdown State    |
|:----------|:----------------------|:------------------|:------------------|
| DIO_0     | I2C SCL               | Peripheral out    | IN  - Pull Up     |
| DIO_1     | I2C SDA               | Peripheral in/out | IN  - Pull Up     |
| DIO_2     | GPIO_3                | OUT - High/Low    | IN  - Pull Down   |
| DIO_3     | GPIO_4                | OUT - High/Low    | IN  - Pull Down   |
| DIO_4     | AM0815 RTC SPI SCK    | Peripheral out    | IN  - Pull Up     |
| DIO_5     | AM0815 RTC SPI MOSI   | Peripheral out    | IN  - Pull Down   |
| DIO_6     | AM0815 RTC SPI MISO   | Peripheral in     | IN  - Pull Down   |
| DIO_7     | AM0815 RTC SPI CS     | OUT - High        | IN  - Pull Up     |
| DIO_8     | AM0815 RTC CHARGE     | OUT - Low         | IN  - Pull Down   |
| DIO_9     | AM0815 RTC IRQ        | IN  - No Pull     | IN  - No Pull     |
| DIO_10    | User button           | IN  - Pull Up     | IN  - Pull Up     |
| DIO_11    | LED_1                 | OUT - Low         | IN  - Pull Down   |
| DIO_12    | GPIO_1                | OUT - High/Low    | IN  - Pull Down   |
| DIO_13    | GPIO_2                | OUT - High/Low    | IN  - Pull Down   |
| DIO_14    | UART RXD              | Peripheral in     | IN  - Pull Down   |
| DIO_15    | UART TXD              | Peripheral out    | IN  - Pull Down   |
| DIO_16    | JTAG TDO pin          | system            | system            |
| DIO_17    | JTAG TDI pin          | system            | system            |
| DIO_18    | EMU voltage config    | OUT - High/Low    | OUT - High/Low    |
| DIO_19    | EMU voltage config    | OUT - High/Low    | OUT - High/Low    |
| DIO_20    | EMU voltage config    | OUT - High/Low    | OUT - High/Low    |
| DIO_21    | EMU voltage config    | OUT - High/Low    | OUT - High/Low    |
| DIO_22    | EMU burst config      | OUT - High/Low    | OUT - High/Low    |
| DIO_23    | EMU burst config      | OUT - High/Low    | OUT - High/Low    |
| DIO_24    | EMU wakeup trigger    | IN  - No Pull     | IN  - No Pull     |
| DIO_25    | EMU wakeup comparator | IN  - No Pull     | IN  - No Pull     |
| DIO_26    | EMU buffer voltage    | IN  - No Pull     | IN  - No Pull     |
| DIO_27    | FRAM SPI SCK          | Peripheral out    | IN  - Pull Down   |
| DIO_28    | FRAM SPI MOSI         | Peripheral out    | IN  - Pull Down   |
| DIO_29    | FRAM SPI MISO         | Peripheral in     | IN  - Pull Down   |
| DIO_30    | FRAM SPI CS           | OUT - High        | IN  - Pull Up     |


## Data Formats

### Data storage in FRAM memory

The last system state is stored at address `0x00` in the FRAM (last successfully
saved system state after completing an full activation).

The sensor data history is stored in a ring buffer in the remaining FRAM memory,
starting at address `0x20`.
First, the fixed number of `N` aggregate values are stored.
In the remaining FRAM memory a wrapping around ring buffer structure is used for
continuous accumulation of new sensor data.

**TODO: FRAM data section layout needs update for aggregate buffers.**

A data unit consists 7 bytes (=54 bits) of data, organized as follows:
- 32 bit of timestamp in seconds since the last system cold start (little endian).
  This allows covering a continuous sensing interval of more than 68 years after
  a cold start of the system.
  The highest 32 bit value (i.e. `0xFFFF FFFF`) refers to an invalid timestamp.
  The remaining `N` highest values are used for `N` aggregate values, enumerated
  in decreasing order (i.e. timestamp  `0xFFFF FFFE` is used for the first,
  `0xFFFF FFFD` for the second aggregate value, etc.)
- 10 bit of relative humidity measurements in 10ths of percents relative humidity
  ranging from 0.0 up to 100.0 %RH
- 14 bit of temperature value in 100ths of centigrades, offset at -40.0 and
  ranging up to +120.0 centigrades.

This results in the following *sensor value* data format (see [data.h:data_encode_value()](data.h))
- Byte `0-3`: The unix timestamp of the time the value was sensed
- Byte `4`: Lower 8 Bits of humidity data
- Byte `5`: Higher 2 Bits of humidity data (LSB) and lower 6 Bits of temperature data (MSB)
- Byte `6`: Higher 8 Bits of temperature data


### Aggregate data unit format

A second data type encodes aggregated data, i.e. averaged and Haar wavelet compressed coefficients. Multi-byte data types are stored little endian byte order.
For the coefficient generation the raw sensor values (temperature in 100th centigrade, humidity in 10th percent) are averaged without offset conversion and 256 of the average values compressed using Haar wavelet transform.
Both, temperature (18bit signed) and humidity (14bit signed) coefficients contain additional addition 8 Bits for the index, which is added as MSB to the actual coefficient data. 

*The aggregate indexed coefficient value* (see [data.h:data_encode_aggregate()](data.h))

- Byte `0-3`: The 32bit unix timestamp is used for: index of the aggregate (8 MSB of timestamp), the humidity data (22 bit) and most significant two temperature value bits (2 LSB of timestamp)
- Byte `4-6`: 24 LSB Bits of temperature data (little endian)


### BLE packet data

The bytes of a BLE packet are organized as follows
- Byte `0`: The length of the following data (excluding this byte)
- Byte `1`: BLE Manufacturer-Specific Data Flag `0xFF`
- Byte `2`: BLE Company ID LSB used for current system status flag
- Bytes `3-30`: 1 up to 4 data units using the same binary data format as for FRAM.