# Timex Datalink Uploader

A fast hack to allow data uploading to a Timex Datalink wristwatch without a CRT monitor or the original software.

This project runs on an "STM32 Blue pill" to drive an LED with the correct timing to allow uploading of data to the watch.

The software receives the data to be transmitted over UART, and produces the correct timing internally. It also generates the required sync frames and start frames before data transmission is started.
