# MiroCard fork of Contiki-NG

This is a fork of [Contiki-NG](https://github.com/contiki-ng/contiki-ng) (release 4.4) with
support for the batteryless MiroCard. It builds on the ETH Zurich
[Transient BLE Node](https://gitlab.ethz.ch/tec/public/employees/sigristl/transient_ble_node)
Contiki-NG patches. MiroCard-specific code lives in:

* `arch/platform/cc26x0-cc13x0/mirocard/` – MiroCard board/platform (`BOARD=mirocard/cc2650`)
* `arch/dev/shtc3/`, `arch/dev/sht3x/`, `arch/dev/emu/`, `arch/dev/am0815/`, `arch/dev/ext-fram/` – sensor and energy-management drivers
* `examples/platform-specific/mirocard/` – example applications (`blinky`, `sensing`, `simple-ble`, `ambient-ble`); see its [README](examples/platform-specific/mirocard/README.md) for build, upload and batteryless-mode instructions

```bash
git clone --recursive https://github.com/ansgomez/mirocard-contiki-ng.git
cd mirocard-contiki-ng/examples/platform-specific/mirocard/simple-ble
make TARGET=cc26x0-cc13x0 BOARD=mirocard/cc2650
```

**Licensing.** Contiki-NG is distributed under the 3-clause BSD license ([LICENSE.md](LICENSE.md)).
Files from the Transient BLE Node project carry ETH Zurich BSD-3-Clause notices, and MiroCard
additions carry (c) 2020 Andres Gomez, Miromico AG BSD-3-Clause notices (the LIS3DH driver and
examples also (c) 2021 Lars Suter). Every
file's own header applies; keep all existing notices when reusing code.

### MiroCard project

The MiroCard is a batteryless, light-powered BLE smart card, designed by Andres Gomez
(Miromico AG) and inspired by the
[Transient BLE Node](https://gitlab.ethz.ch/tec/public/employees/sigristl/transient_ble_node)
project developed at ETH Zurich. It was presented in:

> Andres Gomez. 2020. *Demo Abstract: On-Demand Communication with the Batteryless MiroCard.*
> In The 18th ACM Conference on Embedded Networked Sensor Systems (SenSys '20).
> [doi:10.1145/3384419.3430440](https://doi.org/10.1145/3384419.3430440)

Related repositories:

| Repository | Contents |
| --- | --- |
| [mirocard-hardware](https://github.com/ansgomez/mirocard-hardware) | Hardware: datasheet, schematics and Altium PCB project (MiroCard V2.0) |
| **mirocard-contiki-ng** (this repository) | Firmware: Contiki-NG fork with the MiroCard platform and example applications |
| [miroreader-app](https://github.com/ansgomez/miroreader-app) | Android app to receive and display MiroCard beacons |
| [mirocard-scanner-python](https://github.com/ansgomez/mirocard-scanner-python) | Python scripts to scan for and decode MiroCard beacons (bluepy) and discover devices (gattlib) |
| [mirocard-scanner-mqtt](https://github.com/ansgomez/mirocard-scanner-mqtt) | Node.js bridge forwarding MiroCard beacons to an MQTT broker |
| [mirocard-scanner-influx](https://github.com/ansgomez/mirocard-scanner-influx) | Node.js bridge storing MiroCard beacons in InfluxDB |
| [mirocard-webid](https://github.com/ansgomez/mirocard-webid) | Web Bluetooth demo page for identification and sensor readout |
| [mirocard-postprocessing](https://github.com/ansgomez/mirocard-postprocessing) | Jupyter notebook to post-process RocketLogger power measurements |
| [mirocard-plotly](https://github.com/ansgomez/mirocard-plotly) | Plotly Dash web app visualizing a RocketLogger measurement |

---

<img src="https://github.com/contiki-ng/contiki-ng.github.io/blob/master/images/logo/Contiki_logo_2RGB.png" alt="Logo" width="256">

# Contiki-NG: The OS for Next Generation IoT Devices

[![Build Status](https://travis-ci.org/contiki-ng/contiki-ng.svg?branch=master)](https://travis-ci.org/contiki-ng/contiki-ng/branches)
[![Documentation Status](https://readthedocs.org/projects/contiki-ng/badge/?version=master)](https://contiki-ng.readthedocs.io/en/master/?badge=master)
[![license](https://img.shields.io/badge/license-3--clause%20bsd-brightgreen.svg)](https://github.com/contiki-ng/contiki-ng/blob/master/LICENSE.md)
[![Latest release](https://img.shields.io/github/release/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/releases/latest)
[![GitHub Release Date](https://img.shields.io/github/release-date/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/releases/latest)
[![Last commit](https://img.shields.io/github/last-commit/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/commit/HEAD)

Contiki-NG is an open-source, cross-platform operating system for Next-Generation IoT devices. It focuses on dependable (secure and reliable) low-power communication and standard protocols, such as IPv6/6LoWPAN, 6TiSCH, RPL, and CoAP. Contiki-NG comes with extensive documentation, tutorials, a roadmap, release cycle, and well-defined development flow for smooth integration of community contributions.

Unless explicitly stated otherwise, Contiki-NG sources are distributed under
the terms of the [3-clause BSD license](LICENSE.md). This license gives
everyone the right to use and distribute the code, either in binary or
source code format, as long as the copyright license is retained in
the source code.

Contiki-NG started as a fork of the Contiki OS and retains some of its original features.

Find out more:

* GitHub repository: https://github.com/contiki-ng/contiki-ng
* Documentation: https://github.com/contiki-ng/contiki-ng/wiki
* Web site: http://contiki-ng.org
* Nightly testbed runs: https://contiki-ng.github.io/testbed

Engage with the community:

* Gitter: https://gitter.im/contiki-ng
* Twitter: https://twitter.com/contiki_ng
