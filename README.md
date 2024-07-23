# labjack_pic

Program PIC microcontrollers using LabJack U3

I have multiple PICkits 1, 2, and 3, but none of them can program the
part I selected for my latest project. Rather than shell out for yet
another PICkit, I wrote this script to program the part using a
[LabJack U3](https://labjack.com/products/u3) which I already had
lying around.

## Connections

Connect GND, MCLR, ICSPDAT, ICSPCLK to FIOs on the LabJack. Defaults are:

 - MCLR on FIO4
 - ICSPDAT on FIO5
 - ICSPCLK on FIO6

You can select different pins with the `--mclr`, `--icspdat`, and
`--icspclk` arguments.

If your circuit is OK with 5V power and doesn't draw too much current,
you can power it with VS (USB power).  Otherwise arrange some other
power supply. LabJack outputs are 3.3 V, so lower voltages are a bad
idea.

## Installation

`pip3 install labjack_pic`

## Usage

`labjack_pic [filename.hex]`
  
This programs the hex file:

 - If the hex file contains program code, it will bulk erase that area first
 - If the hex file contains config words, it will erase those first
 - If the hex file contains user id words, it will erase that area first
 - If the hex file contains EEPROM data, it will not erase that
   first. It will update any bytes specified.

## Devices supported

This script only supports low-voltage programming. This means MCLR is
not available as a general-purpose input in the circuit.  This also
means that this programmer cannot recover a part that has been
programmed to MCLR-as-input.

Currently supports devices covered by [Programming Specification 40002317](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/ProgrammingSpecifications/PIC16F180XX-Family-Programming-Specification-40002317.pdf), [Programming Specification 40002266](https://ww1.microchip.com/downloads/aemtest/MCU08/ProductDocuments/ProgrammingSpecifications/PIC16F171XX-Family-Programming-Specification-40002266.pdf), and [Programming Specification 40001683B](https://ww1.microchip.com/downloads/en/DeviceDoc/40001683B.pdf)

These are:

- PIC16F1703
- PIC16LF1703
- PIC16F1704
- PIC16LF1704
- PIC16F1705
- PIC16LF1705
- PIC16F1707
- PIC16LF1707
- PIC16F1708
- PIC16LF1708
- PIC16F1709
- PIC16LF1709
- PIC16F17114
- PIC16F17115
- PIC16F17124
- PIC16F17125
- PIC16F17126
- PIC16F17144
- PIC16F17145
- PIC16F17146
- PIC16F17154
- PIC16F17155
- PIC16F17156
- PIC16F17174
- PIC16F17175
- PIC16F17176
- PIC16F18013
- PIC16F18023
- PIC16F18014
- PIC16F18024
- PIC16F18044
- PIC16F18054
- PIC16F18074
- PIC16F18015
- PIC16F18025
- PIC16F18045
- PIC16F18055
- PIC16F18075
- PIC16F18026
- PIC16F18046
- PIC16F18056
- PIC16F18076

I would accept patches to support other PICs, but larger memory
devices are probably going to be obnoxiously slow to program with
LabJack hardware.

## Performance

The SPI clock of the U3 is relatively slow.  Writing takes maybe 2-3x
as long as a proper programmer that can do the full 5 MHz clock.
