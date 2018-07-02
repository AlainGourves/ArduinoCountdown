#include <Arduino.h>
#include <OneButton.h>
#include <RotaryEncoder.h>
#include <SevenSegmentTM1637.h>


#define ROTARYMIN 0
#define ROTARYMAX 59

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder rotaryEncoder(A2, A3);

// Actions du Rotary Encoder
typedef enum {
  COUNTDOWN_IDLE,     // Il ne se passe rien
  ROTARY_SET_MINUTES, // Fixe les minutes
  ROTARY_SET_SECONDS, // Fixe les secondes
  ROTARY_RESET,       // Met countDown à 0
  COUNTDOWN_1,        // countDown > 10s
  COUNTDOWN_2,        // 5s < countDown <= 10s
  COUNTDOWN_3,        // countDown <= 5s
  COUNTDOWN_END       // countDown = 0
} myActions;


// Actions de l'affichage Led
typedef enum {
  LEDS_00, // tout éteint
  LEDS_01, // minutes éteint, secondes allumé 
  LEDS_10, // minutes allumé, secondes éteint
  LEDS_11  // tout allumé
} ledActions;


// Switch du rotary encoder sur D6
OneButton rotarySwitch(6, true);
// Bouton  Start/Stop
// S -> D2 (interrupt pin), - -> GND
OneButton startButton(10, true);
myActions myNextAction = COUNTDOWN_IDLE; // no action when starting

ledActions nextLedAction = LEDS_11;

const byte PIN_CLK = 4;   // define CLK pin (any digital pin)
const byte PIN_DIO = 5;   // define DIO pin (any digital pin)
const byte PIN_BUZZER = 11;   // S sur D11, via réisistance de 100-200 ohms
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);

unsigned long previousMillisBlink;
unsigned long previousMillisTimer;
unsigned long previousMillisAlarm; // pour limiter la duréee de l'alarme de fin
unsigned long alarmLength = 10UL * 1000; 

uint16_t timer = 179; // 5 minutes 180; // 3 minutes
uint16_t countDown;
uint8_t minutes;
uint8_t seconds;
uint8_t buffer[4];

bool isBlink;
// Pour savoir si le timer tourne ou non
bool isTimer;
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

  startButton.attachClick(myStartFunction);

  display.begin();            // initializes the display
  display.setBacklight(40);  // set the brightness to 100 %
  display.print("init");      // display INIT on the display
  delay(500);                // wait
  display.clear();

  countDown = timer;
  minutes = countDown / 60;
  seconds = countDown % 60;
  isTimer = false;
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
  startButton.tick();

  unsigned long now = millis();
  minutes = countDown / 60;
  seconds = countDown % 60;


  switch (myNextAction)
  {
    case COUNTDOWN_IDLE:
      isBlink = false;
      noTone(PIN_BUZZER);
      break;
  
    case ROTARY_SET_MINUTES:
      isBlink = true;
      break;
  
    case ROTARY_SET_SECONDS:
      isBlink = true;
      break;

    case ROTARY_RESET:
      isBlink = false;
      break;
  
    case COUNTDOWN_1:
      isBlink = false;
      if (now - previousMillisTimer >= 1000) {
        previousMillisTimer = now;
        countDown--;
        if (countDown == 10) {
          myNextAction = COUNTDOWN_2;
        }
      }
      break;
    case COUNTDOWN_2:
      isBlink = false;
      if (now - previousMillisTimer >= 1000) {
        previousMillisTimer = now;
        countDown--;
        if (countDown == 5) {
          myNextAction = COUNTDOWN_3;
        }
      }
      break;

    case COUNTDOWN_3:
      isBlink = false;
      if (now - previousMillisTimer >= 1000) {
        previousMillisTimer = now;
        countDown--;
        if (countDown == 0) {
          previousMillisAlarm = now;
          myNextAction = COUNTDOWN_END;
        }
        tone(PIN_BUZZER, 554, 250);
      }
      break;
     
    case COUNTDOWN_END:
      isBlink = false;
      tone(PIN_BUZZER, 1047, 250);
      if (now - previousMillisAlarm >= alarmLength) {
        isTimer = false;
        myNextAction = COUNTDOWN_IDLE;
      }
      break;
  
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
      if (myNextAction == ROTARY_SET_MINUTES) {
        minutes = pos;
      } else if (myNextAction == ROTARY_SET_SECONDS) {
        seconds = pos;
      }
      countDown = seconds + (minutes * 60);
    } // if
  } else {
    // empêche la valeur d'être actualisée quand on tourne le rotary pendant que isBlink est false
    rotaryEncoder.setPosition(pos);
  }
 
  // Gestion des clignotements
  if (now - previousMillisBlink >= 125 && (myNextAction == COUNTDOWN_3 || myNextAction == COUNTDOWN_END)) {
    if (ledState) {
      nextLedAction = LEDS_11;
    }else{
      nextLedAction = LEDS_00;
    }
    ledState = !ledState;
    previousMillisBlink = now;
  } else if (now - previousMillisBlink >= 250) {
    if (myNextAction == COUNTDOWN_IDLE) {
      nextLedAction = LEDS_11;
    } else if (myNextAction == ROTARY_SET_MINUTES) {
      if (ledState) {
        nextLedAction = LEDS_11;
      } else {
        nextLedAction = LEDS_01;
      }
    } else if (myNextAction == ROTARY_SET_SECONDS) {
      if (ledState) {
        nextLedAction = LEDS_11;
      } else {
        nextLedAction = LEDS_10;
      }
    } else if (myNextAction == ROTARY_RESET || myNextAction == COUNTDOWN_2) {
      if (ledState) {
        nextLedAction = LEDS_11;
      }else{
        nextLedAction = LEDS_00;
      }
      if (myNextAction == ROTARY_RESET) {
        resetBlinkCount++;
        if (resetBlinkCount == 4) {
          resetBlinkCount = 0;
          myNextAction = COUNTDOWN_IDLE;
        }
      }
    }
    ledState = !ledState;
    previousMillisBlink = now;
  }

  // Affichage Led
  switch (nextLedAction)
  {
  case LEDS_00:
    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;
    break;

  case LEDS_01:
    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = display.encode(seconds / 10);
    buffer[3] = display.encode(seconds % 10);
    break;

  case LEDS_10:
    buffer[0] = display.encode(minutes / 10);
    buffer[1] = display.encode(minutes % 10);
    buffer[2] = 0;
    buffer[3] = 0;
    break;

  case LEDS_11:
    buffer[0] = display.encode(minutes / 10);
    buffer[1] = display.encode(minutes % 10);
    buffer[2] = display.encode(seconds / 10);
    buffer[3] = display.encode(seconds % 10);
    break;
  }  // end of switch
  
  display.setColonOn(true);
  display.printRaw(buffer, 4, 0);
} // loop ()

// this function will be called when the button was pressed 1 time and them some time has passed.
void myClickFunction() {
  if (myNextAction == COUNTDOWN_IDLE) {
    myNextAction = ROTARY_SET_MINUTES;
    pos = minutes;
  } else if (myNextAction == ROTARY_SET_MINUTES) {
    myNextAction = ROTARY_SET_SECONDS;
    pos = seconds;
  } else if (myNextAction == ROTARY_SET_SECONDS) {
    myNextAction = COUNTDOWN_IDLE;
  }
  rotaryEncoder.setPosition(pos);
} // myClickFunction

// this function will be called when the button was pressed during 1 second.
void myLongPressFunction() {
  // Mise à zéro du compteur
  if (myNextAction == COUNTDOWN_IDLE) {
    myNextAction = ROTARY_RESET;
    countDown = 0;
  }
}

void myStartFunction() {
  if (myNextAction == COUNTDOWN_IDLE) {
    if (!isTimer && countDown != 0) {
      isTimer = true;
      myNextAction = COUNTDOWN_1;
    }
  } else {
    // Stop
    isTimer = false;
    myNextAction = COUNTDOWN_IDLE;
  }
}