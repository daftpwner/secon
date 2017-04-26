/*
 * Simple support for one to three pin change interrupts. 
 *
 * Simple, in that it's up to the user to determine the pin change with 
 * multiple pins per port.
 * 
 * Copyright (c) 2016 Peter James Soper pete@soper.us
 * April 27, 2016 MIT license
 */

#ifndef SIMPLEPINCHANGE_H

#define SIMPLEPINCHANGE_H

#define __SIMPLE_PIN_CHANGE_VERSION 1

#include <Arduino.h>

#define NUM_PORTS 3

typedef void (*voidFuncPtr)();

class SimplePinChangeClass {
  public:
    // Define a callback function for a change of the given pin. Return true if
    // the port handling this pin has already been subscribed. This is 
    // informational. There's no problem with multiple pins, but the callback
    // function has to determine which of the multiple pins changed in this 
    // case.  An invalid pin number results in an immediate return of false.
    // Ports are assigned as follows on an Atmega328:
    //  Pins 8-13 are in the port for PCINT0
    //  Pins 14-21 (A0-A7) are in the port for PCINT1, but NOTE that pins 20 
    //  and 21 (A6, A7) cannot be used for digital I/O.
    //  Pins 0-7 are in the port for PCINT2.
    static bool attach(uint8_t pin, void (*callback)());

    // Remove the association of a change of the given pin with an interrupt.
    // Return true if this was the last pin for a port (the callback is nulled
    // in this case).
    // An invalid pin number results in an immediate return of false
    static bool detach(uint8_t pin);

    // Return a mask indicating if an undefined interrupt happened. This would
    // suggest other code has enabled a pin change interrupt. Mask is cleared
    // after call. Bit 0 corresponds to PCINT0, 1 to PCINT1, and 2 to PCINT2.
    static uint8_t getError();
};

extern SimplePinChangeClass SimplePinChange;

#endif
