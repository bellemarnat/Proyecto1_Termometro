#!/usr/bin/python3
#
# Python script that regenerates the README.md from the embedded template. Uses
# ./generate_table.awk to regenerate the ASCII tables from the various *.txt
# files.

from subprocess import check_output

nano_results = check_output(
    "./generate_table.awk < nano.txt", shell=True, text=True)
micro_results = check_output(
    "./generate_table.awk < micro.txt", shell=True, text=True)
samd21_results = check_output(
    "./generate_table.awk < samd21.txt", shell=True, text=True)
stm32_results = check_output(
    "./generate_table.awk < stm32.txt", shell=True, text=True)
samd51_results = check_output(
    "./generate_table.awk < samd51.txt", shell=True, text=True)
esp8266_results = check_output(
    "./generate_table.awk < esp8266.txt", shell=True, text=True)
esp32_results = check_output(
    "./generate_table.awk < esp32.txt", shell=True, text=True)

print(f"""\
# Memory Benchmark

The `MemoryBenchmark.ino` was compiled with each `FEATURE_*` and the flash
memory and static RAM sizes were recorded. The `FEATURE_BASELINE` selection is
the baseline, and its memory usage numbers are subtracted from the subsequent
`FEATURE_*` memory usage.

**Version**: AUnit v1.7.1

**DO NOT EDIT**: This file was auto-generated using `make README.md`.

## How to Regenerate

To regenerate this README.md:

```
$ make clean_benchmarks
$ make benchmarks
$ make README.md
```

The `make benchmarks` target uses `collect.sh` script which calls `auniter.sh`
(https://github.com/bxparks/AUniter) to invoke the Arduino IDE programmatically.
It produces a `*.txt` file with the flash and ram usage information (e.g.
`nano.txt`).

The `make README.md` command calls the `generated_readme.py` Python script which
generates this `README.md` file. The ASCII tables below are generated by the
`generate_table.awk` script, which takes each `*.txt` file and converts it to an
ASCII table.

## Library Size Changes

## Arduino Nano

* 16MHz ATmega328P
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* Arduino AVR Boards 1.8.6

```
{nano_results}
```

## Sparkfun Pro Micro

* 16 MHz ATmega32U4
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* SparkFun AVR Boards 1.1.13

```
{micro_results}
```

## SAMD21 Seeeduino XIAO M0

* SAMD51, 120 MHz ARM Cortex-M4
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* Seeeduino SAMD 1.8.4

```
{samd21_results}
```

## STM32 Blue Pill

* STM32F103C8, 72 MHz ARM Cortex-M3
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* STM32duino 2.5.0

```
{stm32_results}
```

## SAMD51 Adafruit ItsyBitsy M4

* SAMD51, 120 MHz ARM Cortex-M4
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* Adafruit SAMD 1.7.11

```
{samd51_results}
```

## ESP8266

* NodeMCU 1.0, 80MHz ESP8266
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* ESP8266 Boards 3.0.2

```
{esp8266_results}
```

## ESP32

* ESP32-01 Dev Board, 240 MHz Tensilica LX6
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* ESP32 Boards 2.0.9

```
{esp32_results}
```
""")
