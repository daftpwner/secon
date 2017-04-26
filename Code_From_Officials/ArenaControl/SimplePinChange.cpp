/*
 * Simple support pin change interrupts tested with Atmega328 chips (e.g.
 * Arduino Uno, Nano, Mini). Not likely to work with chips with more or 
 * different ports. 
 *
 * Copyright (c) 2015 Peter James Soper
 * April 27, 2016 MIT license
 *
 * Mods to neaten the code and add some explicit common subexpression
 * elimination introduced bugs that were fixed by Rod Radford March 21, 2017.
 */

#include "SimplePinChange.h"

// Callback for port change or NULL if none defined
static volatile voidFuncPtr callback[NUM_PORTS];

// Mask of spurious interrupts on a port lacking a callback (see vector 
// functions for values)
static uint8_t spurious_interrupt;

// Treat the interface as a global class instance
SimplePinChangeClass SimplePinChange;

// Return spurious interrupt mask. Each of the low order three bit positions,
// if set, means that code outside this library enabled a pin change interrupt 
// for that port: no callback had been registered by this code.

uint8_t SimplePinChangeClass::getError() {
  uint8_t temp = spurious_interrupt;
  spurious_interrupt = 0;
  return temp;
}

// Define a callback function for a change of the given pin. Return true if
// the port handling this pin has already been subscribed (but perhaps to a
// different callback routine). This is informational. There's no problem with 
// multiple pins being registered, but the callback function has to determine 
// which of the multiple pins changed in this case. An invalid pin number 
// is ignored with a return of false;

bool SimplePinChangeClass::attach(uint8_t pin, void (*handler)()) {
  // The address of the pin change mask register for the pins of the relevant
  // port
  volatile uint8_t *p = digitalPinToPCMSK(pin);


  if (p == NULL) {
    // Invalid pin number
    return false;
  }

  // The bit number of the PCICR register bit involved with the port for this
  // pin

  uint8_t pcicr_bit = digitalPinToPCICRbit(pin);

  // Remember if there was any subscription already

  bool subscription = callback[pcicr_bit];

  // Disable any pending interrupt to avoid a spurious one
  PCICR  &= ~bit(pcicr_bit);

  // Define the handler 

  callback[pcicr_bit] = handler;

  // Sets the pin change mask value for the port controlling the given pin to
  // make the chip sensitive to a change.
  *p |= bit(digitalPinToPCMSKbit(pin)); 

  // Enable pin change interrupts for the port containing the given pin
  PCICR  |= bit(pcicr_bit);

  return subscription;
}

// Remove the association of a change of the given pin with an interrupt.
// The callback is unsubscribed if this was the last pin of its port and true
// is returned. An invalid pin is ignored with a return of false.

bool SimplePinChangeClass::detach(uint8_t pin) {

  volatile uint8_t *p = digitalPinToPCMSK(pin);

  if (p == NULL) {
    return false;
  }

  if (*p) {
    // Clear mask bit
    *p &= ~bit (digitalPinToPCMSKbit(pin));

    // If the mask is clear now disable port interrupt and remove the handler
    if (!*p) {
      uint8_t pcicr_bit;
      uint8_t mask = bit(pcicr_bit = digitalPinToPCICRbit(pin));
      // Disable pin change interrupts for the port containing the given pin
      PCICR  &= ~mask;

      // Undefine handler
      callback[pcicr_bit] = NULL;

      return true;
    }
  }

  return false;
}

// Vector for pin 8-13 change interrupt
ISR(PCINT0_vect) {
  if (callback[0]) {
    callback[0]();
  } else {
    // This should be impossible!
    spurious_interrupt |= 1;
  }
}

// Vector for pin 14-21 (A0-A7) change interrupt
ISR(PCINT1_vect) {
  if (callback[1]) {
    callback[1]();
  } else {
    // This should be impossible!
    spurious_interrupt |= 2;
  }
}

// Vector for pin 0-7 change interrupt
ISR(PCINT2_vect) {
  if (callback[2]) {
    callback[2]();
  } else {
    // This should be impossible!
    spurious_interrupt |= 4;
  }
}
