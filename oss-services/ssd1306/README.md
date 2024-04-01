
# oss-services/ssd306

## Overview

This component contains a modified version of an STM32-based library for
a 128x64 display driven by an SSD1306. The original display used to test
this component can be obtained here [Amazon: Hosyond 128x64 Display](https://www.amazon.com/Hosyond-Display-Self-Luminous-Compatible-Raspberry/dp/B09C5K91H7/ref=sr_1_2_sspa?crid=30N98J5XYIK5I&keywords=i2c+oled+display&qid=1683310137&sprefix=i2c+oled+display%2Caps%2C115&sr=8-2-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExMVg0RlZPRVZXRDJQJmVuY3J5cHRlZElkPUEwNDU0NjYwMzJKSTY2VFRHQjlGViZlbmNyeXB0ZWRBZElkPUEwMzk0NTU3M0U3TkMyR0VEMDFPVCZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=)

The display used has a fixed yellow color mask for the top 1/4 or so of
the screen, and the rest is blue. Other displays can be purchased at the
above link for pure white or pure blue displays. 

## Origin

https://github.com/afiskon/stm32-ssd1306

## Hardware configuration

The display was tested with SC584-EZLITE. The display itself needs an I2C bus (SCL/SDA)
and a Vcc/GND connection. Vcc can be 3.3V or 5V. Below were the pins used for testing:

        Header  Pin
Vcc:    P20     2
GND:    P2      10
SCL:    P3      1
SDA:    P3      3

## Required components

- twi-simple
- define a "delay(unsigned ms)" function somewhere in the code. 
  - delay function should be thread-safe for RTOS applications. 

## Recommended components

- FreeRTOS (vTaskDelay function for thread-safe delay; otherwise use a HAL)

## Integrate the source

- Copy the contents of the `oss-services/ssd1306` directory to relevant project 
  source/include directories.
- Add the necessary source and include paths to the project Makefile. 
- Edit `ssd1306_conf.h` as needed for your application. 

## Example usage

```
#include ssd1306.h

/* Open up a global device handle for TWI0 @ 400KHz */
twiResult = twi_open(TWI0, &context->twi0Handle);
if (twiResult != TWI_SIMPLE_SUCCESS) {
    syslog_print("Could not open TWI0 device handle!");
    return;
}
twi_setSpeed(context->twi0Handle, TWI_SIMPLE_SPEED_400);

/* Initialize the SSD1306 Display */
ssd1306_twiOpen(context->twi0Handle);
ssd1306_Init();

/* Test bitmap draws garfield & 2 github logo bitmaps */
ssd1306_TestDrawBitmap();

/* Atomic screen operation -- draw a line */
ssd1306_Line(1,1,SSD1306_WIDTH - 1,SSD1306_HEIGHT - 1,White);
ssd1306_Line(SSD1306_WIDTH - 1,1,1,SSD1306_HEIGHT - 1,White);
ssd1306_UpdateScreen();

/* Atomic screen operation -- draw a bitmap */
//Garfield
ssd1306_Fill(White);
ssd1306_DrawBitmap(0,0,garfield_128x64,128,64,Black);
ssd1306_UpdateScreen();

//Github logo
ssd1306_Fill(Black);
ssd1306_DrawBitmap(32,0,github_logo_64x64,64,64,White);
ssd1306_UpdateScreen()
```

Please refer to ssd1306.h to see all available functions and parameters. 