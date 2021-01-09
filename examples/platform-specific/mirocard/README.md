# Contiki-NG MiroCard Examples

The MiroCard and MiroReader app are designed by Andres Gomez, inspired by the [Transient BLE Node] (https://gitlab.ethz.ch/tec/public/employees/sigristl/transient_ble_node) project developed at [ETH Zurich](https://ethz.ch/en.html). The Transient BLE Node project is an open-source project released under the Creative Commons Attribution 4.0 International License. The MiroCard project is released under an MIT license. 

The following projects are available:

### Blinky

This a sample project that blink's an LED. This project is for sample purposes, it is not intended for batteryless operation.

### Sensing

This is another sample project that shows the MiroCard's sensing capabilities. It is not intended for batteryless operation.

### Simple-BLE

This sample application emits a constant BLE advertisement. In it's simplest form, it can be used as an identifier for users or locations. This application can be batteryless.

### Ambient-BLE

This is the sample application reads an ambient sensor and broadcasts the last sensor reading. This application can be batteryless.

# Running Sample Projects

Before you compile one of these projects, be aware that certain projects are not compatible with batteryless operation. The `blinky` and `sensing` projects require too much energy to run from an SMD capacitor, so they are meant as practice projects. If you are interested in batteryless applications, take a look at the `simple-ble` and `ambient-ble` projects.

### Batteryless Operation

MiroCards are meant to be deployed for batteryless operation. To enable this, the following flag needs to be defined (typically in the project-conf.h): 

`MIROCARD_BATTERYLESS`

In this mode, there are additional settings to be configured for correct Energy Management Unit (EMU) operation. Specifically, the external wake-up pin and its edge needs to be defined in the following macros:

`WAKEUP_TRIGGER_IOID`

`WAKEUP_TRIGGER_EDGE`

### Debugging Mode

For development and debugging purposes, the MiroCard can be USB powered. To do this, please ensure the `MIROCARD_BATTERYLESS` flag is *not* defined. If this is the case, the Contiki-NG application will executed normally (without waiting for an EMU signal). Note that the applications do not have a main loop, so they run only once. They also require a steady power supply, typically from the flashing device.
# To Build this application

### Compile

To compile, use the following command:

```bash
make TARGET=cc26x0-cc13x0 BOARD=mirocard/cc2650
```

Optionally, you can add `WERROR=0` to avoid having warnings treated as errors.

### Upload

You can use the ROM Bootloader to load a new image via UART. Use the following command:

```bash
make <PROJECT_NAME>.upload PORT=<YOUR_PORT>
```
Where `<PROJECT_NAME>` is one of the above mentioned projects (blinky, sensing, simple-ble, ambient-ble).

Where `<YOUR_PORT>` is replaced with the MiroCard's UART port. In Linux/MacOS this will be "/dev/tty*", and in Windows "COM*".

Alternately, you can load the hex file via JTAG, using a TagConnect cable to connect the MiroCard to a LaunchPad/XDS debugger. This has been tested using the SmartRF Flasher 2 on Windows.

### Serial Logging

If you have enabled UART logging, you can view the device's output on the terminal. Use the following command:

```bash
make login PORT=<YOUR_PORT>
```

Where `<YOUR_PORT>` is replaced with the MiroCard's UART port. In Linux/MacOS this will be "/dev/tty*", and in Windows "COM*".
### Contributors

MiroCard Software Contributors:

* Andres Gomez, Miromico AG
* Simon Galli, Miromico AG
