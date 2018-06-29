// -----
// SimplePollRotator.ino - Example for the RotaryEncoder library.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// -----

// This example checks the state of the rotary encoder in the loop() function.
// The current position is printed on output when changed.

// Hardware setup:
// Attach a rotary encoder with output pins to A2 and A3.
// The common contact should be attached to ground.

#include <RotaryEncoder.h>
#include "SevenSegmentTM1637.h"

#define ROTARYMIN 0
#define ROTARYMAX 59

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(A2, A3);
const int rotarySwitchPin = 6;

const byte PIN_CLK = 4;   // define CLK pin (any digital pin)
const byte PIN_DIO = 5;   // define DIO pin (any digital pin)
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);

uint8_t buffer[4];
unsigned long previousMillis;
bool ledState = true;

void setup()
{
  Serial.begin(57600);
  Serial.println("Hello !");
  pinMode(rotarySwitchPin, INPUT_PULLUP);
  
  encoder.setPosition(0);
  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
  
  display.begin();            // initializes the display
  display.setBacklight(40);  // set the brightness to 100 %
  display.print("init");      // display INIT on the display
  delay(1000);                // wait 1000 ms
  display.clear();
} // setup()

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

// Read the current position of the encoder and print out when changed.
void loop()
{
  static int pos = 0;
  encoder.tick();

  int newPos = encoder.getPosition();
  if (newPos < ROTARYMIN) {
    encoder.setPosition(ROTARYMAX);
    newPos = ROTARYMAX;
  }
  if (newPos > ROTARYMAX) {
    encoder.setPosition(ROTARYMIN);
    newPos = ROTARYMIN;
  }
  if (pos != newPos) {
    Serial.print(newPos);
    Serial.println();
    pos = newPos;
  } // if

  unsigned long now = millis();
  if (now - previousMillis >= 250) {
    if (ledState) {
      buffer[0] = display.encode(pos / 10);
      buffer[1] = display.encode(pos % 10);
    } else {
      buffer[0] = 0;
      buffer[1] = 0;
    }
    buffer[2] = 0;
    buffer[3] = 0;
    ledState = !ledState;
    previousMillis = now;
  }
  display.printRaw(buffer, 4, 0);
} // loop ()

// The End

