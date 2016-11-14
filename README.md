## Relay Timer
Sketch for a user settable timer controlling a relay output. The hardware system for this code is described in the [Arduino++ blog](https://arduinoplusplus.wordpress.com/2016/02/27/led-display-relay-timer/)

## Hardware requirements
* Arduino Uno/Nano/Mini/etc
* 7 segment 4 Digit LED display
* Rotary encoder with built in selection switch
* Relay output or relay module

## Function
* Arduino manages the LED display
* Rotary encoder allows setting the required timer value
* Rotary encoder switch used to start the timer
* Timer can be paused with a press of switch. Second press resumes and long press ends the timer.
* While timer is active (running or paused) the relay output is switched on.