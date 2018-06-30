#include <Arduino.h>
#include <OneButton.h>
#include <RotaryEncoder.h>
#include <SevenSegmentTM1637.h>

// -----
// SimplePollRotator.ino - Example for the RotaryEncoder library.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de

// Hardware setup:
// Attach a rotary encoder with output pins to A2 and A3.
// The common contact should be attached to ground.

#define ROTARYMIN 0
#define ROTARYMAX 59

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder rotaryEncoder(A2, A3);

// Actions du Rotary Encoder
typedef enum {
  ROTARY_IDLE,          // Il ne se passe rien
  ROTARY_SET_MINUTES,   // Fixe les minutes
  ROTARY_SET_SECONDS,    // Fixe les secondes
  ROTARY_RESET
}
MyActions;

// Switch du rotary encoder sur D6
OneButton rotarySwitch(6, true);

MyActions nextAction = ROTARY_IDLE; // no action when starting

const byte PIN_CLK = 4;   // define CLK pin (any digital pin)
const byte PIN_DIO = 5;   // define DIO pin (any digital pin)
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);

unsigned long previousMillis;
uint16_t timer = 179; // 5 minutes 180; // 3 minutes
uint16_t countDown;
uint8_t minutes;
uint8_t seconds;
uint8_t buffer[4];

bool isBlink;
int pos = 0;

void setup()
{
  Serial.begin(57600);
  Serial.println("Hello !");

  rotaryEncoder.setPosition(0);
  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

  // link the myClickFunction function to be called on a click event.
  rotarySwitch.attachClick(myClickFunction);
  // long press event.
  rotarySwitch.attachLongPressStart(myLongPressFunction);

  display.begin();            // initializes the display
  display.setBacklight(40);  // set the brightness to 100 %
  display.print("init");      // display INIT on the display
  delay(500);                // wait
  display.clear();

  countDown = timer;
  minutes = countDown / 60;
  seconds = countDown % 60;
  // pos = minutes;
} // setup()

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  rotaryEncoder.tick(); // just call tick() to check the state.
}

// Read the current position of the encoder and print out when changed.
void loop()
{
  static int resetBlinkCount = 0; // static : la valeur est préservée entre les loop()
  static bool ledState = true; // sert à gérer les clignotements
//  rotaryEncoder.tick(); // Inutile là, c'est dans ISR() que ça se passe
  // keep watching the push button:
  rotarySwitch.tick();

  unsigned long now = millis();
  minutes = countDown / 60;
  seconds = countDown % 60;

  if (nextAction == ROTARY_IDLE) {
    isBlink = false;
  } else  if (nextAction == ROTARY_SET_MINUTES) {
    isBlink = true;
  } else if (nextAction == ROTARY_SET_SECONDS) {
    isBlink = true;
  }

  if (isBlink) {
    int newPos = rotaryEncoder.getPosition();
    if (newPos < ROTARYMIN) {
      rotaryEncoder.setPosition(ROTARYMAX);
      newPos = ROTARYMAX;
    }
    if (newPos > ROTARYMAX) {
      rotaryEncoder.setPosition(ROTARYMIN);
      newPos = ROTARYMIN;
    }
    if (pos != newPos) {
      pos = newPos;
      if (nextAction == ROTARY_SET_MINUTES) {
        minutes = pos;
      } else if (nextAction == ROTARY_SET_SECONDS) {
        seconds = pos;
      }
      countDown = seconds + (minutes * 60);
    } // if
  } else {
    // empêche la valeur d'être actualisée quand on tourne le rotary pendant que isBlink est false
    rotaryEncoder.setPosition(pos);
  }

  if (now - previousMillis >= 250) {
    if (nextAction == ROTARY_IDLE) {
      buffer[0] = display.encode(minutes / 10);
      buffer[1] = display.encode(minutes % 10);
      buffer[2] = display.encode(seconds / 10);
      buffer[3] = display.encode(seconds % 10);
    } else if (nextAction == ROTARY_SET_MINUTES) {
      if (ledState) {
        buffer[0] = display.encode(minutes / 10);
        buffer[1] = display.encode(minutes % 10);
      } else {
        buffer[0] = 0;
        buffer[1] = 0;
      }
      buffer[2] = display.encode(seconds / 10);
      buffer[3] = display.encode(seconds % 10);
    } else if (nextAction == ROTARY_SET_SECONDS) {
      buffer[0] = display.encode(minutes / 10);
      buffer[1] = display.encode(minutes % 10);
      if (ledState) {
        buffer[2] = display.encode(seconds / 10);
        buffer[3] = display.encode(seconds % 10);
      } else {
        buffer[2] = 0;
        buffer[3] = 0;
      }
    } else if (nextAction == ROTARY_RESET) {
      if (ledState) {
        buffer[0] = display.encode(minutes / 10);
        buffer[1] = display.encode(minutes % 10);
        buffer[2] = display.encode(seconds / 10);
        buffer[3] = display.encode(seconds % 10);
      }else{
        buffer[0] = 0;
        buffer[1] = 0;
        buffer[2] = 0;
        buffer[3] = 0;
      }
      resetBlinkCount++;
      if (resetBlinkCount == 4) {
        resetBlinkCount = 0;
        nextAction = ROTARY_IDLE;
      }
    }
    ledState = !ledState;
    previousMillis = now;
  }
  display.setColonOn(true);
  display.printRaw(buffer, 4, 0);
} // loop ()

// this function will be called when the button was pressed 1 time and them some time has passed.
void myClickFunction() {
  if (nextAction == ROTARY_IDLE) {
    nextAction = ROTARY_SET_MINUTES;
    pos = minutes;
  } else if (nextAction == ROTARY_SET_MINUTES) {
    nextAction = ROTARY_SET_SECONDS;
    pos = seconds;
  } else if (nextAction == ROTARY_SET_SECONDS) {
    nextAction = ROTARY_IDLE;
  }
  rotaryEncoder.setPosition(pos);
} // myClickFunction

// this function will be called when the button was pressed during 1 second.
void myLongPressFunction() {
  // Mise à zéro du compteur
  if (nextAction == ROTARY_IDLE) {
    nextAction = ROTARY_RESET;
    countDown = 0;
  }
}

