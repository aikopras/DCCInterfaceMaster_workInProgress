## Release notes

Release notes of the DCC Interface Master (C) Philipp Gahtow.
These notes reflect Library Versions up to 6.4.0.
These notes are no longer updated for Version 7.0.0 and above.

Compared to upstream the following has been changed:
* compatibility for ESP32C3
* conditional compilation for ESP Cores 2.x and 3.x for Arduino
* some minor cleanup of if then else structures
* indentation fixes

modified by Philipp Gahtow 2015-2021 digitalmoba@arcor.de
* - add a store for active loco, so you can request the actual state
* - add a store for BasicAccessory states
* - add a repeat queue for Speed and Function packets
* - add Function support F13-F20 and F21-F28
* - add CV POM Messages
* - add BasicAccessory increment 4x (Intellibox - ROCO)
* - add request for state of Loco funktion F0 - F28
* - support DCC generation with Timer1 or Timer2
* - add notify of BasicAccessory even when power is off
* - change praeambel to 16 Bit for Railcom support
* - add Railcom hardware support
* - optimize Railcom signal timing
* - fix bug on ESP32 (https://github.com/crosstool-ng/crosstool-ng/issues/1330)
* - fix DCC ACK Detection
* - fix DCC Timing on ESP8266 and ESP32
* - fix bug with ACK Detection

===========

See also:
- http://pgahtow.de/wiki/index.php?title=DCC#Arduino_DCC_Library
- http://pgahtow.de/wiki/index.php?title=Zentrale
