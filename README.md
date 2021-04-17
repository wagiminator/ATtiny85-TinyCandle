# NeoCandle - Candle Simulation based on ATtiny25/45/85
NeoCandle is a simple tea light candle simulation using 5mm NeoPixels (WS2812), LDR light sensor (GL5528) and IR remote receiver (TSOP4856). It can be controlled by an IR remote control with two buttons and has a timer mode for automatic shutdown after a set time as well as an LDR mode in which the candle is automatically switched on and off depending on the intensity of the ambient light.

- Project Video (YouTube): https://youtu.be/n4UFV3BMcBM
- Design Files (EasyEDA): https://easyeda.com/wagiminator/attiny85-neocandle-dip

![NeoCandle_pic1.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny85-TinyCandle/master/documentation/NeoCandle_pic1.jpg)

# Compiling and Uploading Firmware
Open the NeoCandle Sketch and adjust the IR codes so that they match your remote control. Remember that only the NEC protocol is supported. If necessary, adjust the values for the LDR light sensor and the switch-off timer.

## If using the Arduino IDE
- Make sure you have installed [ATtinyCore](https://github.com/SpenceKonde/ATTinyCore).
- Go to **Tools -> Board -> ATtinyCore** and select **ATtiny25/45/85 (No bootloader)**.
- Go to **Tools** and choose the following board options:
  - **Chip:**           ATtiny25 or 45 or 85 (depending on your chip)
  - **Clock:**          8 MHz (internal)
  - **Millis/Micros:**  disabled
  - **B.O.D.Level:**    disabled
  - Leave the rest at the default settings
- Connect your programmer to your PC and to the ICSP header on the board.
- Go to **Tools -> Programmer** and select your ISP programmer (e.g. [USBasp](https://aliexpress.com/wholesale?SearchText=usbasp)).
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open NeoCandle sketch and click **Upload**.

## If using the precompiled hex-file
- Make sure you have installed [avrdude](https://learn.adafruit.com/usbtinyisp/avrdude).
- Connect your programmer to your PC and to the ISCP header on the board.
- Open a terminal.
- Navigate to the folder with the hex-file.
- Execute the following command (if necessary replace "t85" with your chip and "usbasp" with the programmer you use):
  ```
  avrdude -c usbasp -p t85 -U lfuse:w:0xe2:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m -U flash:w:neocandle.hex
  ```

## If using the makefile (Linux/Mac)
- Make sure you have installed [avr-gcc toolchain and avrdude](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).
- Connect your programmer to your PC and to the ICSP header on the board.
- Open the makefile and change the chip if you are not using ATtiny85 and the programmer if you are not using usbasp.
- Open a terminal.
- Navigate to the folder with the makefile and the Arduino sketch.
- Run "make install" to compile, burn the fuses and upload the firmware.

# Operating Instructions
- Connect a 5V power supply to the Micro-USB socket or a battery to the respective board header.
- The NeoCandle should immediately mimic the flickering of a candle. The device is now in **TIMER mode** and switches off automatically after the time programmed in the code has elapsed. You can switch it off at any time by pressing the power-off button on the IR remote control and switch it back on with the power-on button.
- If the power-on button on the IR remote control is pressed while the NeoCandle is switched on, it changes to **LDR mode**. This change is indicated by a short blue light animation. In LDR mode, NeoCandle switches on and off automatically depending on the intensity of the ambient light. Pressing the power-on or power-off button on the IR remote control switches back to TIMER mode.

# References, Links and Notes
1. [ATtiny25/45/85 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)
2. [WS2812 Datasheet](https://cdn-shop.adafruit.com/datasheets/WS2812.pdf)
3. [TSOP4856 Datasheet](https://www.vishay.com/docs/82459/tsop48.pdf)
4. [GL5528 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/SEN-09088.pdf)
5. [Neopixel Infos by Josh Levine](https://wp.josh.com/category/neopixel/)
6. [Neopixel Implementation by Josh Levine](https://github.com/bigjosh/SimpleNeoPixelDemo)
7. [Candle Simulation Implementation by Mark Sherman](https://github.com/carangil/candle)
8. [Lightweight Random Number Generator by Łukasz Podkalicki](https://blog.podkalicki.com/attiny13-pseudo-random-numbers/)
9. [IR Receiver Implementation](https://github.com/wagiminator/ATtiny13-TinyDecoder)
10. [TinyCandle for ATtiny13A](https://github.com/wagiminator/ATtiny13-TinyCandle)

![NeoCandle_pic2.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny85-TinyCandle/master/documentation/NeoCandle_pic2.jpg)
![NeoCandle_pic3.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny85-TinyCandle/master/documentation/NeoCandle_pic3.jpg)
![NeoCandle_pic4.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny85-TinyCandle/master/documentation/NeoCandle_pic4.jpg)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
