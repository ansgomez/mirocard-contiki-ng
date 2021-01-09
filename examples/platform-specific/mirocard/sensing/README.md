# MiroCard Demo
===========
This is a sample project for the MiroCard. It can be used as a basis for development. Since blinking LEDs requires a signficant amount of energy, this demo does not have a batteryless mode by default. For a batteryless demo, please look at the "simple-ble" project.

## Batteryless Operation

MiroCard are meant to be deployed for batteryless operation. To enable this, the following flag needs to be defined (typically in the project-conf.h): 

`MIROCARD_BATTERYLESS`

In this mode, there are additional settings to be configured for correct Energy Management Unit (EMU) operation. Specifically, the external wake-up pin and its edge needs to be defined in the following macros:

`WAKEUP_TRIGGER_IOID`

`WAKEUP_TRIGGER_EDGE`

## Debugging Mode

For development and debugging purposes, the MiroCard can be USB powered. To do this, please ensure the `MIROCARD_BATTERYLESS` flag is *not* defined. If this is the case, the Contiki-NG application will executed normally (without waiting for an EMU signal). This mode requires a steady power supply, typically from the flashing device.
# To Build this application

## Compile

To compile, use the following command:

```bash
make TARGET=cc26x0-cc13x0 BOARD=mirocard/cc2650
```

Optionally, you can add `WERROR=0` to avoid having warnings treated as errors.

## Upload

You can use the ROM Bootloader to load a new image via UART. Use the following command:

```bash
make blinky.upload PORT="YOUR_DEVICE"
```

Where "YOUR_DEVICE" is replaced with the MiroCard's UART port. In Linux/MacOS this will be "/dev/tty*", and in Windows "COM*".

Alternately, you can load the hex file via JTAG, using a TagConnect cable to connect the MiroCard to a LaunchPad/XDS debugger. This has been tested using the SmartRF Flasher 2 on Windows.

## Serial Logging

If you have enabled UART logging, you can view the device's output on the terminal. Use the following command:

```bash
make login PORT="YOUR_DEVICE"
```

Where "YOUR_DEVICE" is replaced with the MiroCard's UART port. In Linux/MacOS this will be "/dev/tty*", and in Windows "COM*".