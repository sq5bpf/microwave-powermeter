# microwave-powermeter

This is a standalone microwave power meter using an AD8317 module for measurement and a 20x4 LCD display, featuring extensive multi-point calibration capabilities.

Outputs power in dBm and watts, +- an arbitrary attenuation or gain (so that an external attenuator or amplifier can be used in the measurements).


The device uses an ATmega328p Arduino Nano board (or clone), however it doesn't use the Arduino environment and libraries and is written in plain C, with a proper Makefile.

The AD8317 isn't very linear, and will measure differently at different frequencies. For this reason the meter has 10 calibrations, each for one frequency, and each calibration can consist of up to 10 calibration points. If the measurement falls between the calibration points, it will be interpolated, if it is outside the range, then the calibration will be extrapolated from the two nearest points.

The AD8317 measures 1MHz-10GHz, around -50dBm upto 0dBm according to the datasheet. However the layout of the cheap modules and the board material used will probably limit the frequency range. Even with these problems, proper calibration should let one make decent measurements.

The primary user interface is an 20x4 LCD, with 2 buttons and cheap rotary encoder.

The device also features a command line interface via the serial port, which can be used to read/write calibration data, read the adc and perform other tasks. This works in parallel to the LCD user interface.


To compile and install:

apt-get install avrdude avr-libc make

git clone https://github.com/sq5bpf/microwave-powermeter

cd microwave-powermeter

compile:

make

program the binary to an arduino nano connected to /dev/ttyUSB0:

make prog



(This is just a preliminary description. I wanted to put the project on github fast, so that i don't forget about it. Unfortunately it is now in the "works for me" stage of development, but i will write some better documentation eventually)

VY 73

Jacek / SQ5BPF



