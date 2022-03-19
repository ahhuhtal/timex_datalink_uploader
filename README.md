# Timex Datalink Uploader

A fast hack to allow data uploading to a Timex Datalink wristwatch without a CRT monitor or the original software. For a project, which interfaces with the original Timex software, but replaces the notebook adapter, check out: https://github.com/famiclone6502/DIY_Datalink_Adapter

This software has only been tested with a Timex Datalink 150. I think it should work with a Timex Datalink 150s and it might work with the Ironman series. It definitely won't work with a Datalink 50 or 70, though it shouldn't be too much trouble to get it to work (transfer 1 byte in a frame instead of 2). If you're interested in getting this to work with the other Datalink watches, I'm interested in helping out.

This project runs on an "STM32 Blue pill" to drive an LED with the correct timing to allow uploading of data to the watch. The software receives the data to be transmitted over USB CDC (using the USB port of the Bluepill), and produces the correct timing internally. It also generates the required sync frames and start frames before data transmission is started.

As it stands, the LED (and current limiting resistor) is expected to be connected between PB9 and GND. PB9 is used as TIM4 OC4 to drive the proper waveforms. The original Timex Notebook adapter uses a red LED, and my quick testing also suggests that red indeed works better than green, blue or white.

Data is transmitted to the software through the Bluepill USB port using USB CDC (device will register as /dev/ttyACMx under Linux). The selected bitrate or framing doesn't affect communication.

There is no protocol. Anything transmitted to the device will be output to the watch. The watch data rate is much slower than USB CDC, and the data to be transmitted is buffered in the MCU (now 8kB). It is possible to overflow this buffer by sending more data, but since the watch can ever only take about 3kB of data in, it should be enough for normal operation even with the watch upload protocol overheads.

This is such a quick hack, that it is made in STM32CubeIDE and also uses FreeRTOS. If I have time, I'll work on making this smaller, simpler and neater.

To generate useful data for this program, take a look at https://github.com/dfries/datalink_ironman
Using the setwatch utility with the -file switch allows generation of a file (called DEBUGOUTPUT) that can be cat'ted to the serial port.

E.g.
```
# generate a bitstream containing only date and time using example datafile
./setwatch -150 -file -time datafile

# set tty to suitable settings
stty -F /dev/ttyACM0 raw pass8

# dump bitstream to watch
cat DEBUGOUTPUT > /dev/ttyACM0
```
