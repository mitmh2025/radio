# Two P.I. Noir Radio Firmware

This project contains the firmware for the 2025 MIT Mystery Hunt radio,
targeting our custom hardware based on the ESP32-S3-MINI-1-N4R2.

## Setup

### Toolchain installation

This firmware is built on Espressif's [ESP-IDF][] and [ESP-ADF][] frameworks.
You'll need to install both. Make sure the install the correct versions of each.

First, setup ESP-IDF using the [installation instructions for v5.3.1][ESP-IDF
install]. You should only need to complete steps 1-4. In step 3, make sure to
install the tools for `esp32s3` (which is different than just `esp32`).

Next, install ESP-ADF. Because we want a specific version, run:

```
cd ~/esp
git clone --recursive -b ff7f39dcf0a0da87671ff4ab422eb258be429049 https://github.com/espressif/esp-adf.git
. v5.3.1/esp-idf/export.sh
cd esp-adf && ./install.sh
```

Once both are installed, you can activate the IDF and ADF environment by
running:

```
. ~/src/esp/v5.3.1/esp-idf/export.sh && . ~/src/esp/esp-adf/export.sh
```

**Note**: You must activate ESP-IDF before activating ESP-ADF, otherwise ESP-ADF
will activate an embedded copy of ESP-IDF, which is the wrong version for this
project.

### Configuration and build

Make sure to clone this repository with `--recursive` or run `git submodule
update --init --recursive` to fetch all dependencies.

Copy the `config.template.h` file to `config.h`. Set `RADIO_WIFI_SSID` and
`RADIO_WIFI_PASSWORD` as appropriate for your local network. (Note: the ESP32-S3
can not connect to 5GHz networks and our firmware can not navigate any captive
portals). `RADIO_THINGSBOARD_SERVER` should be set to `"things.mitmh2025.com"`

Finally, run `idf.py build` to generate the firmware.

## Flashing

To flash compiled firmware onto a radio board, run `idf.py flash`.

On new boards or boards which are otherwise stuck in boot loops, you may need to
manually put the board into firmware download mode. To do this, perform the
following sequence:

* Press (and hold) RST
* Press (and hold) PROG
* Release RST
* Release PROG

and then attempt to flash the firmware. Once complete, you will need to press
and release RST to exit firmware download mode.

## Monitoring

To monitor the radio over its built-in USB serial port, run `idf.py monitor`.
This will reset the firmware when it connects. The radio has a simple command
REPL accessible over serial.

To exit the monitor, press `Ctrl + ]`. It also has a range of [additional
keyboard shortcuts][ESP-IDF monitor].

## Provisioning

A new board has a number of required provisioning steps in order to reach full
functionality. The radio will prompt for each of these in its logs, but the list
is enumerated here as well for convenience:

* ThingsBoard provisioning: Go to the [ThingsBoard devices page][] and add a new
  device. Use the MAC address as the device name and set Device profile to
  "radio". Copy the "Access token" from the credentials page, then use the
  `provision` command in the radio serial console to store the access token.
* WHEP (WebRTC) URL: In ThingsBoard, open the device, go to Attributes, choose
  Shared Attributes, and create a key called `whep_url`. You can use
  `https://radio.mitmh2025.com/music/whep` as the value for testing.

[ESP-IDF]: https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/index.html
[ESP-ADF]: https://docs.espressif.com/projects/esp-adf/en/latest/
[ESP-IDF install]: https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/get-started/linux-macos-setup.html
[ESP-IDF monitor]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-monitor.html
[ThingsBoard devices page]: https://things.mitmh2025.com/entities/devices