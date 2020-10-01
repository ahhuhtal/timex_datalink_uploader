# Timex Datalink Uploader

A fast hack to allow data uploading to a Timex Datalink wristwatch without a CRT monitor or the original software.

This project runs on an "STM32 Blue pill" to drive an LED with the correct timing to allow uploading of data to the watch.

The software receives the data to be transmitted over UART, and produces the correct timing internally. It also generates the required sync frames and start frames before data transmission is started.

This is such a quick hack, that it is made in STM32CubeIDE as well as uses FreeRTOS. If I have time, I'll work on making this smaller, simpler and neater.
