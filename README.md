# Two P.I. Noir Radio Firmware

This project contains the firmware for the 2025 MIT Mystery Hunt radio,
targeting our custom hardware based on the ESP32-S3-MINI-1-N4R2. Full
schematics, designs, partslists, and additional details are available in the
[Mystery Hunt archives][].

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
portals).

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

## Calibration and Provisioning

Before any main routines will run, a newly flashed radio board will go through a calibration and validation process. This involves checking the ranges of both the volume and frequency potentiometers and ensuring that all inputs and sensors are working. The radio will prompt for each step in its logs.

Once that is complete, it will prompt for ThingsBoard provisioning. As Mystery Hunt is over and our ThingsBoard server is no longer running, these steps can likely be skipped; see the archives for instructions on how to set the various parameters that would have previously been configured via ThingsBoard.

## Architecture

(Unless otherwise specified, all filenames are in the `main/` directory.)

The entrypoint for the radio firmware is `main.c`. However, once the radio has been calibrated (see `calibration.c`), the main function configures the hardware, starts an assortment of FreeRTOS tasks and event listeners, and then goes into a sleep loop. These various tasks and event listeners are responsible for the majority of the radio's functionality.

Several modules in the code form a light OS layer for managing hardware, including `accelerometer.{h,c}`, `adc.{h,c}`, `bluetooth.{h,c}`, `debounce.{h,c}`, `fm.{h,c}`, `led.{h,c}`, `magnet.{h,c}`, `storage.{h,c}`, `tas2505.{h,c}`, `touch.{h,c}`. This also includes the code in `components/boardconfig/` (which must be structured as a separate IDF component due to requirements of ESP-ADF). These modules only provide hardware abstractions, and generally do not attempt to enforce any sort of mutual exclusion to individual hardware components (although where it makes sense, they generally support shared usage of independent components, like separate ADC channels). The header files provide a good introduction into the interfaces they support. A few additional modules also primarily manage hardware, but operate as largely independent subroutines rather than interface layers (`battery.{h,c}`, `wifi.{h,c}`).

The firmware is designed to be orchestrated by [ThingsBoard][], which is managed in `things.{h,cpp}`. (This is the only code in the firmware written in C++, as the ThingsBoard SDK is in C++.) Functionality throughout the firmware is enabled or disabled by ThingsBoard shared attributes. The ThingsBoard client subscribes to changes in attributes and in turn allows other modules to subscribe to it. It additionally caches all attribute values in [NVS][], ensuring that configuration is still available if the radio is offline or when first powered on.

The core modality of the radio (i.e. tuning to a station) is handled by the tuner (`tuner.{h,c}`). It monitors the band switch and frequency potentiometer and determines the active station. FM stations are managed entirely within the tuner logic. PM station modules (e.g. `station_numbers.{h,c}`) register with the tuner and enable or disable their stations based on ThingsBoard shared attributes. Stations register callback functions to start ("entune") or stop ("detune") themselves, and must fully clean up after themselves when they are detuned (including releasing hardware resources). The sole exception to the tuner's control over the radio is during "activation" of Station π / Songs on the Radio (i.e. knocking on the radio), during which `station_pi_activation.c` takes control, suspending the tuner (and many other background routines) until the activation audio playback is complete.

Audio playback is primarily mediated by the mixer (`mixer.{h,c}`), which uses the [ESP-ADF downmix module][] to mix up to 8 separate tracks of audio together (in practice, we at most use 2-3) and output that audio via I2S to the TAS2505. Tracks are registered as callbacks which yield audio samples. The mixer is also responsible for generating static when there are no active tracks (generally indicating the radio is not tuned to a valid station) or "comfort noise" (lower volume static) when the radio is tuned to a valid station but no audio is currently playing. Additionally, the mixer can temporarily duck the volume of tracks when requested, for instance, during the post-solve celebration fanfare. Outside of Station π and activation of Station π, `audio_output.{h,c}` is responsible for switching the TAS2505 configuration between speaker and headphone output (based on the presence of a headphone plug). Amplification volume of the TAS2505 is managed by the ADC callback in `main/audio_volume.c`.

The radio has limited ability to generate audio (see `tone_generator.{h,c}`,used for Songs on the Radio), but local audio playback is primarily from Opus or WAV files stored locally on flash. The two flash chips are treated as combined concatenated storage and mounted as a single LittleFS filesystem (see `storage.{h,c}`). The file cache (`file_cache.c`) is responsible for populating local storage, using a JSON manifest file configured via ThingsBoard. The manifest file includes a list of named files and their expected hash values. These files are stored locally by hash (allowing for an atomic switch to a new version of the file) but looked up by name. Because they proved to be common operations, `playback.{h,c}` provides an interface for playing an audio file through the mixer while `playback_queue.{h,c}` provides an interface for playing a series of audio files in sequence.

Finally, for Station 2π, `webrtc.{h,c}` wraps our fork of the [Amazon Kinesis Video Streams WebRTC SDK][]. It is responsible for WebRTC signaling (via [WHEP][]) and provides a connection handle interface for the rest of the firmware. That is consumed by `webrtc_manager.c`, which detects connection failure and reconnects as needed.

[Mystery Hunt archives]: https://puzzles.mit.edu/2025/extras/radio
[ESP-IDF]: https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/index.html
[ESP-ADF]: https://docs.espressif.com/projects/esp-adf/en/latest/
[ESP-IDF install]: https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/get-started/linux-macos-setup.html
[ESP-IDF monitor]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-monitor.html
[ESP-ADF downmix module]: https://docs.espressif.com/projects/esp-adf/en/latest/api-reference/audio-processing/downmix.html
[ThingsBoard]: https://thingsboard.io/
[NVS]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/storage/nvs_flash.html
[Amazon Kinesis Video Streams WebRTC SDK]: https://github.com/mitmh2025/amazon-kinesis-video-streams-webrtc-sdk-c
[WHEP]: https://www.ietf.org/archive/id/draft-murillo-whep-03.html