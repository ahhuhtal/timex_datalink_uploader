# Timex Datalink Uploader

A fast hack to allow data uploading to a Timex Datalink wristwatch without a CRT monitor or the original software.

This has only been tested with a Timex Datalink 150. I think it should work with a Timex Datalink 150s and it might work with the Ironman series. It definitely won't work with a Datalink 50 or 70, though it shouldn't be too much trouble to get it to work. If you're interested in getting this to work with the other Datalink watches, I'm interested in helping out.

This project runs on an "STM32 Blue pill" to drive an LED with the correct timing to allow uploading of data to the watch. The software receives the data to be transmitted over UART, and produces the correct timing internally. It also generates the required sync frames and start frames before data transmission is started.

As it stands, the LED is expected to be connected between PA6 and GND. PA6 is used as TIM3 OC1 to drive the proper waveforms. The original Timex Notebook adapter uses a red LED, and my quick testing also suggests that red indeed works better than green, blue or white.

Data is transmitted to the software through USART1 connected at PA10 (RX) and PA9 (TX). Framing is 9600,N,8,1. No protocol. Anything transmitted to the software will be output to the watch as well as echoed back. The watch data rate is much slower, and the data to be transmitted id buffered in the MCU (now 8kB). It is very likely possible to overflow these buffers, but since the watch can only take about 3kB of data in, should be enough for normal operation, even with the watch upload protocol overheads.

This is such a quick hack, that it is made in STM32CubeIDE and also uses FreeRTOS. If I have time, I'll work on making this smaller, simpler and neater.

To generate useful data for this program, take a look at https://github.com/dfries/datalink_ironman
Using the setwatch utility with the -file switch allows generation of a file that can be cat'ted to the serial port.
