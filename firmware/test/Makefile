# Host-side firmware tests. No Pico, no Arduino toolchain, no hardware.
#
#   make -C firmware/test
#
# This compiles the sketch against a stub Arduino core (ArduinoStub.h) and
# drives it with synthetic SBUS frames and serial input. It cannot tell you
# anything about timing, electrical behaviour or the real USB stack. What it
# does tell you is whether the mode arbitration, the failsafes, the light
# convention and the STATE line still say what they are supposed to say --
# which is exactly the class of thing that broke silently between v3 and the
# Jetson and went unnoticed for months.

CXXFLAGS = -std=c++17 -Wall -Wextra -Wno-unused-parameter -I.

all: run

sketch.inc: ../pico-node_v4/pico-node_v4.ino
	sed -e 's|#include <Servo.h>||' \
	    -e 's|#include <Adafruit_TinyUSB.h>||' \
	    -e '1i #include "ArduinoStub.h"' $< > $@

fwtest: harness.cpp ArduinoStub.h sketch.inc
	$(CXX) $(CXXFLAGS) -o $@ harness.cpp

run: fwtest
	./fwtest

clean:
	rm -f fwtest sketch.inc

.PHONY: all run clean
