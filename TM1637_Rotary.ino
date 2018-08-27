#include <Arduino.h>
#include "LowPower.h"
#include <OneButton.h>
#include <RotaryEncoder.h>
#include <SevenSegmentTM1637.h>


#define ROTARYMIN 0
#define ROTARYMAX 59

// Setup a RoraryEncoder for pins A2 (CLK) and A3 (DT):
RotaryEncoder rotaryEncoder(A2, A3);


// Switch du rotary encoder sur D10
OneButton rotarySwitch(10, true);
// Bouton  Start/Stop
// Use pin 2 as wake up pin
const byte wakeUpPin = 2;
// S -> D2 (interrupt pin), - -> GND
OneButton startButton(wakeUpPin, true);

// Bouton  3 minutes
OneButton buttonPreset1(7, true);
// Bouton  5 minutes
OneButton buttonPreset2(8, true);
// Bouton  10 minutes
OneButton buttonPreset3(9, true);


const byte PIN_CLK = 4;   // define CLK pin (any digital pin)
const byte PIN_DIO = 5;   // define DIO pin (any digital pin)
const byte PIN_TRANS = 6; // transistor qui active le TM1637
const byte PIN_BUZZER = 11;   // S sur D11, via résistance de 100-200 ohms
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);
const uint8_t TM1637Brightness = 30;

unsigned long previousMillisBlink; // temps de clignotement des leds
unsigned long previousMillisTimer; // pour le calcul des secondes
unsigned long previousMillisSleepTimer = 0; // stocke la durée avant la mise en veille
unsigned long previousMillisAlarm; // pour mesurer la durée de l'alarme de fin
unsigned long alarmLength = 10UL * 1000; // durée de l'alarme de fin
unsigned long sleepTime = 20UL * 1000; // temps de déclenchement de la veille

uint16_t timer = 0;
uint16_t countDown;
uint8_t minutes;
uint8_t seconds;
uint8_t buffer[4];

bool isRotaryActive;
// Pour savoir si le timer tourne ou non
bool isTimer;
int pos = 0;

// Presets pour les 3 boutons : 3, 5 et 10 minutes
int presets[] = {180, 300, 600};

// Pour le buzzer
int notes[] = { 932, 1175, 988, 932 };
int notesDurations[] = { 250, 250, 250, 500 };
int notesIntervals[] = { 40, 80, 40, 80 };

// Différents états de la machine
typedef enum {
  COUNTDOWN_IDLE,     // Il ne se passe rien
  ROTARY_SET_MINUTES, // Fixe les minutes
  ROTARY_SET_SECONDS, // Fixe les secondes
  ROTARY_RESET,       // Met countDown à 0
  COUNTDOWN_1,        // countDown > 10s
  COUNTDOWN_2,        // 5s < countDown <= 10s
  COUNTDOWN_3,        // countDown <= 5s
  COUNTDOWN_END,      // countDown = 0
  COUNTDOWN_SLEEP,     // veille
  BATTERY_CHECK,
  BATTERY_SHOW,
  BATTERY_WAIT
} myActions;
myActions myNextAction = COUNTDOWN_IDLE;

// const char* etats[] = {"COUNTDOWN_IDLE", "ROTARY_SET_MINUTES", "ROTARY_SET_SECONDS", "ROTARY_RESET", "COUNTDOWN_1", "COUNTDOWN_2", "COUNTDOWN_3", "COUNTDOWN_END", "COUNTDOWN_SLEEP", "BATTERY_CHECK", "BATTERY_SHOW", "BATTERY_WAIT"};

// Etats de l'affichage Led
typedef enum {
  LEDS_00, // tout éteint
  LEDS_01, // minutes éteint, secondes allumé
  LEDS_10, // minutes allumé, secondes éteint
  LEDS_11,  // tout allumé
  LEDS_BATT
} ledActions;
ledActions nextLedAction = LEDS_11;

// const char* etatsLeds[] = {"LEDS_00", "LEDS_01", "LEDS_10", "LEDS_11", "LEDS_BATT"};

// Pour la mesure de la batterie
unsigned int ADCValue;
double Voltage;
double Vcc;
bool isVoltage = false;

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // Atmel datasheet p317
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  result = (high << 8) | low;
  //  Serial.println(result);

  result = 1105863L / result; // Calculate Vcc (in mV); valeur = [valeur de la ref interne]*1023*1000
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


void setup() {
  Serial.begin(57600);
  Serial.println(F("Hello !"));

  rotaryEncoder.setPosition(pos);
  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

  // link the myRotaryClickFunction function to be called on a click event.
  rotarySwitch.attachClick(myRotaryClickFunction);
  // long press event.
  rotarySwitch.attachLongPressStart(myRotaryLongPressFunction);

  startButton.attachClick(myStartFunction);
  startButton.attachLongPressStart(myStartStartLongPressFunction);    // Lance la mesure de la batterie
  startButton.attachDuringLongPress(myStartDuringLongPressFunction);  // Affiche la charge de la batterie
  startButton.attachLongPressStop(myStartStopLongPressFunction);      // Retour à l'affichage normal

  buttonPreset1.attachClick(myPreset1Function);
  buttonPreset2.attachClick(myPreset2Function);
  buttonPreset3.attachClick(myPreset3Function);

  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_TRANS, HIGH);
  display.begin();                        // initializes the display
  display.setBacklight(TM1637Brightness); // set the brightness
  display.print("init");
  delay(500);
  display.clear();

  countDown = timer;
  isTimer = false;
} // setup()

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  rotaryEncoder.tick(); // just call tick() to check the state.
}
// ISR for INT0
void wakeUp() {
  // Just a handler for the pin interrupt.
}
// Read the current position of the encoder and print out when changed.
void loop() {
  static int resetBlinkCount = 0; // static : la valeur est préservée entre les loop()
  static bool ledState = true; // sert à gérer les clignotements
  static int countBuzzer = 0;

  unsigned long now = millis();
  minutes = countDown / 60;
  seconds = countDown % 60;

  // if (now - previousMillisBlink >= 250) {
  //   Serial.print(etats[myNextAction]);
  //   Serial.print(" / ");
  //   Serial.println(etatsLeds[nextLedAction]);
  // }
  // keep watching the push button:
  rotarySwitch.tick();
  startButton.tick();
  buttonPreset1.tick();
  buttonPreset2.tick();
  buttonPreset3.tick();

  if (myNextAction != COUNTDOWN_IDLE) {
    previousMillisSleepTimer = 0;
  }

  switch (myNextAction)
  {
    case COUNTDOWN_IDLE:
      isRotaryActive = false;
      display.on();
      noTone(PIN_BUZZER);
      if (previousMillisSleepTimer == 0) {
        // déclenche le timer pour la mise en veille
        previousMillisSleepTimer = now;
      }
      if (now - previousMillisSleepTimer > sleepTime) {
        // mis en veille
        myNextAction = COUNTDOWN_SLEEP;
      }
      break;

    case ROTARY_SET_MINUTES:
      isRotaryActive = true;
      break;

    case ROTARY_SET_SECONDS:
      isRotaryActive = true;
      break;

    case ROTARY_RESET:
      countDown = 0;
      // myNextAction = COUNTDOWN_IDLE; // c'est fait après le clignotement
      break;

    case COUNTDOWN_1:
      if (now - previousMillisTimer >= 1000) {
        previousMillisTimer = now;
        countDown--;
        if (countDown <= 10) {
          myNextAction = COUNTDOWN_2;
        }
      }
      break;
    case COUNTDOWN_2:
      if (now - previousMillisTimer >= 1000) {
        previousMillisTimer = now;
        countDown--;
        tone(PIN_BUZZER, notes[0], 10);
        if (countDown <= 5) {
          myNextAction = COUNTDOWN_3;
        }
      }
      break;

    case COUNTDOWN_3:
      if (now - previousMillisTimer >= 1000) {
        previousMillisTimer = now;
        countDown--;
        tone(PIN_BUZZER, notes[0], 50);
        if (countDown == 0) {
          previousMillisAlarm = now;
          myNextAction = COUNTDOWN_END;
        }
      }
      break;

    case COUNTDOWN_END:
      tone(PIN_BUZZER, notes[countBuzzer], notesDurations[countBuzzer]);           // Output sound frequency to buzzerPin
      delay(notesIntervals[countBuzzer]);
      countBuzzer =  (countBuzzer == 3) ? 0 : countBuzzer + 1;
      if (now - previousMillisAlarm >= alarmLength) {
        isTimer = false;
        myNextAction = COUNTDOWN_SLEEP;
      }
      break;

    case COUNTDOWN_SLEEP:
      noTone(PIN_BUZZER);
      // Allow wake up pin to trigger interrupt on low.
      attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW);

      // Coupe l'alim du TM1637
      display.off();
      digitalWrite(PIN_TRANS, LOW);
      // Serial.println(F("Go to sleep !"));
      // Serial.flush();

      // Enter power down state with ADC and BOD module disabled.
      // Wake up when wake up pin is low.
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

      // Disable external pin interrupt on wake up pin.
      detachInterrupt(0);
      // Serial.println(F("Awake !"));
      // Serial.flush();

      // rétablit l'alim du TM1637
      //display.on();
      digitalWrite(PIN_TRANS, HIGH);
      display.begin();                        // initializes the display
      display.setBacklight(TM1637Brightness); // set the brightness to 100 %
      display.clear();                  // display INIT on the display
      myNextAction = COUNTDOWN_IDLE;
      break;

    case BATTERY_CHECK:
      measureVoltage();
      break;

    case BATTERY_SHOW:
      display.clear();
      nextLedAction = LEDS_BATT;
      myNextAction = BATTERY_WAIT;
      break;

    case BATTERY_WAIT:
      // on attend
      break;
  }


  if (isRotaryActive) {
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
    // empêche la valeur d'être actualisée quand on tourne le rotary pendant que isRotaryActive est false
    rotaryEncoder.setPosition(pos);
  }

  // Gestion des clignotements
  if (now - previousMillisBlink >= 125 && (myNextAction == COUNTDOWN_3 || myNextAction == COUNTDOWN_END)) {
    if (ledState) {
      nextLedAction = LEDS_11;
    } else {
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
      } else {
        nextLedAction = LEDS_00;
      }
      if (myNextAction == ROTARY_RESET) {
        resetBlinkCount++;
        if (resetBlinkCount == 4) {
          resetBlinkCount = 0;
          myNextAction = COUNTDOWN_IDLE;
        }
      }
    } else if (myNextAction == BATTERY_WAIT) {
      nextLedAction = LEDS_BATT;
    }
    ledState = !ledState;
    previousMillisBlink = now;
  }

  // Affichage Led
  switch (nextLedAction) {
    case LEDS_00:
      display.setColonOn(false);
      buffer[0] = 0;
      buffer[1] = 0;
      buffer[2] = 0;
      buffer[3] = 0;
      break;

    case LEDS_01:
      display.setColonOn(false);
      buffer[0] = 0;
      buffer[1] = 0;
      buffer[2] = display.encode(seconds / 10);
      buffer[3] = display.encode(seconds % 10);
      break;

    case LEDS_10:
      display.setColonOn(true);
      buffer[0] = display.encode(minutes / 10);
      buffer[1] = display.encode(minutes % 10);
      buffer[2] = 0;
      buffer[3] = 0;
      break;

    case LEDS_11:
      display.setColonOn(true);
      buffer[0] = display.encode(minutes / 10);
      buffer[1] = display.encode(minutes % 10);
      buffer[2] = display.encode(seconds / 10);
      buffer[3] = display.encode(seconds % 10);
      break;

    case LEDS_BATT:
      display.setColonOn(false);
      buffer[0] = B00110110;
      buffer[1] = B00110110;
      buffer[2] = B00110110;
      buffer[3] = B00110110;
      if (Voltage > 3.995) {
        /* 8 */
      } else if (Voltage > 3.91 && Voltage <= 3.995) {
        /* 7 */
        buffer[0] = B00000110;
      } else if (Voltage > 3.87 && Voltage <= 3.91) {
        /* 6 */
        buffer[0] = B00000000;
      } else if (Voltage > 3.775 && Voltage <= 3.87) {
        /* 5 */
        buffer[0] = B00000000;
        buffer[1] = B00000110;
      } else if (Voltage > 3.72 && Voltage <= 3.775) {
        /* 4 */
        buffer[0] = B00000000;
        buffer[1] = B00000000;
      } else if (Voltage > 3.695 && Voltage <= 3.72) {
        /* 3 */
        buffer[0] = B00000000;
        buffer[1] = B00000000;
        buffer[2] = B00000110;
      } else if (Voltage > 3.67 && Voltage <= 3.695) {
        /* 2 */
        buffer[0] = B00000000;
        buffer[1] = B00000000;
        buffer[2] = B00000000;
      } else if (Voltage > 3.5 && Voltage <= 3.67) {
        /* 1 */
        buffer[0] = B00000000;
        buffer[1] = B00000000;
        buffer[2] = B00000000;
        buffer[3] = B00000110;
      } else if (Voltage < 3.5) {
        /* 0  display : "Batt"*/
        buffer[0] = B01111000;
        buffer[1] = B01111000;
        buffer[2] = B01011111;
        buffer[3] = B01111111;
      }
      break;
  }  // end of switch

  display.printRaw(buffer, 4, 0);
} // loop ()

void measureVoltage() {
  Vcc = readVcc() / 1000.0;
  ADCValue = analogRead(0);
  Voltage = (ADCValue / 1024.0) * Vcc;
  Serial.print(Voltage);
  Serial.print("\t");
  Serial.println(Vcc);
  isVoltage = true;
  display.clear();
}

// this function will be called when the button was pressed 1 time and them some time has passed.
void myRotaryClickFunction() {
  switch (myNextAction) {
    case COUNTDOWN_IDLE:
      myNextAction = ROTARY_SET_MINUTES;
      pos = minutes;
      break;
    case ROTARY_SET_MINUTES:
      myNextAction = ROTARY_SET_SECONDS;
      pos = seconds;
      break;
    default:
      myNextAction = COUNTDOWN_IDLE;
      break;
  }
  rotaryEncoder.setPosition(pos);
} // myRotaryClickFunction

// this function will be called when the button was pressed during 1 second.
void myRotaryLongPressFunction() {
  if (myNextAction == COUNTDOWN_SLEEP) {
    myNextAction = COUNTDOWN_IDLE;
  } else if (myNextAction == COUNTDOWN_IDLE) {
    // Mise à zéro du compteur
    myNextAction = ROTARY_RESET;
  }
}

void myPreset1Function() {
  if (myNextAction == COUNTDOWN_IDLE) {
    previousMillisSleepTimer = 0;
    countDown = presets[0];
  }
}
void myPreset2Function() {
  if (myNextAction == COUNTDOWN_IDLE) {
    previousMillisSleepTimer = 0;
    countDown = presets[1];
  }
}
void myPreset3Function() {
  if (myNextAction == COUNTDOWN_IDLE) {
    previousMillisSleepTimer = 0;
    countDown = presets[2];
  }
}

void myStartFunction() {
  if (myNextAction == COUNTDOWN_IDLE) {
    if (!isTimer && countDown != 0) {
      isTimer = true;
      myNextAction = COUNTDOWN_1;
    } else {
      previousMillisSleepTimer = millis();
    }
  } else {
    // Stop
    isTimer = false;
    myNextAction = COUNTDOWN_IDLE;
  }
}

void myStartStartLongPressFunction() {
  if (myNextAction == COUNTDOWN_IDLE) {
    myNextAction = BATTERY_CHECK;
  }
}

void myStartDuringLongPressFunction() {
  if (isVoltage && myNextAction == BATTERY_CHECK) {
    myNextAction = BATTERY_SHOW;
  }
}

void myStartStopLongPressFunction() {
  if (myNextAction == BATTERY_WAIT) {
    isVoltage = false;
    display.clear();
    myNextAction = COUNTDOWN_IDLE;
  }
}
