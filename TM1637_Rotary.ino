#include <Arduino.h>
#include "LowPower.h"
#include <OneButton.h>
#include <RotaryEncoder.h>
#include <SevenSegmentTM1637.h>
#include <avr/power.h>

#define ROTARYMIN 0
#define ROTARYMAX 59

// Setup a RoratyEncoder for pins A2 (CLK) and A3 (DT):
RotaryEncoder rotaryEncoder(A2, A3);

// Switch du rotary encoder sur D7
OneButton rotarySwitch(7, true);
// Bouton  Start/Stop
const byte wakeUpPin = 2; // Use pin 2 as wake up pin
OneButton startButton(wakeUpPin, true);

// Bouton  3 minutes
OneButton buttonPreset1(8, true);
// Bouton  5 minutes
OneButton buttonPreset2(9, true);
// Bouton  10 minutes
OneButton buttonPreset3(10, true);

const byte PIN_CLK = 4; // (TM1637) define CLK pin
const byte PIN_DIO = 5; // (TM1637) define DIO pin
SevenSegmentTM1637 display(PIN_CLK, PIN_DIO);
const uint8_t TM1637Brightness = 30;

// const byte PIN_TRANS = 6; // transistor qui active le TM1637
const byte PIN_BUZZER = 11; // Sur D11, via résistance de 100-200 ohms

unsigned long previousMillisBlink;          // temps de clignotement des leds
unsigned long previousMillisTimer;          // pour le calcul des secondes
unsigned long previousMillisSleepTimer = 0; // stocke la durée avant la mise en veille
unsigned long previousMillisAlarm;          // pour mesurer la durée de l'alarme de fin
const unsigned long alarmLength = 117UL * 100;    // durée de l'alarme de fin (correspond à 5 cycles)
const unsigned long sleepTime = 20UL * 1000;      // temps de déclenchement de la veille

uint16_t timer = 0;
uint16_t countDown;
uint8_t minutes;
uint8_t seconds;
uint8_t buffer[4];

bool isRotaryActive;
int pos = 0; // stocke la position de l'encodeur
// Pour savoir si le timer tourne ou non
bool isTimer;

// Presets pour les 3 boutons : 3, 5 et 10 minutes
const int presets[] = {180, 300, 600};

// Pour le buzzer
const int notes[] =           {698, 880, 988, 1175, 1047, 880, 698, 587}; // F5 A5 B5 D6 C6 A5 F5 D5
const int notesDurations[] =  {250, 100, 200, 100,  200,  100, 200, 100};
const int notesIntervals[] =  {350, 120, 200, 120,  200,  120, 200, 200};

// Différents états de la machine
typedef enum
{
  COUNTDOWN_IDLE,     // Il ne se passe rien
  ROTARY_SET_MINUTES, // Fixe les minutes
  ROTARY_SET_SECONDS, // Fixe les secondes
  ROTARY_RESET,       // Met countDown à 0
  COUNTDOWN_1,        // countDown > 10s
  COUNTDOWN_2,        // 5s < countDown <= 10s
  COUNTDOWN_3,        // countDown <= 5s
  COUNTDOWN_END,      // countDown = 0
  COUNTDOWN_SLEEP,    // veille
  BATTERY_CHECK,      // mesure le voltage de la batterie
  BATTERY_SHOW,       // affiche l'état de la batterie
  BATTERY_WAIT,       // maintient l'affichage de l'état
  BATTERY_ALERT       // Alerte quand la batterie est à plat
} myActions;
myActions myNextAction = COUNTDOWN_IDLE;

// const char* etats[] = {"COUNTDOWN_IDLE", "ROTARY_SET_MINUTES", "ROTARY_SET_SECONDS", "ROTARY_RESET", "COUNTDOWN_1", "COUNTDOWN_2", "COUNTDOWN_3", "COUNTDOWN_END", "COUNTDOWN_SLEEP", "BATTERY_CHECK", "BATTERY_SHOW", "BATTERY_WAIT"};

// Etats de l'affichage Led
typedef enum
{
  LEDS_00,   // tout éteint
  LEDS_01,   // minutes éteint, secondes allumé
  LEDS_10,   // minutes allumé, secondes éteint
  LEDS_11,   // tout allumé
  LEDS_BATT, // bargraph de l'état de la batterie
  LEDS_ALERT // voltage batterie trop bas
} ledActions;
ledActions nextLedAction = LEDS_11;

// const char* etatsLeds[] = {"LEDS_00", "LEDS_01", "LEDS_10", "LEDS_11", "LEDS_BATT"};

// Pour la mesure de la batterie
unsigned int ADCValue;
double Voltage;
double Vcc;
bool isVoltage = false;

long readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // Atmel datasheet p317
  delay(2);                                               // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                                    // Start conversion
  while (bit_is_set(ADCSRA, ADSC));                       // measuring
  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  result = (high << 8) | low;

  result = 1105863L / result; // Calculate Vcc (in mV); valeur = [valeur de la ref interne]*1023*1000
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void setup()
{
  // Serial.begin(57600);
  // Serial.println(F("Hello !"));

  rotaryEncoder.setPosition(pos);
  PCICR |= (1 << PCIE1);                     // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11); // This enables the interrupt for pin 2 and 3 of Port C.

  // link the myRotaryClickFunction function to be called on a click event.
  rotarySwitch.attachClick(myRotaryClickFunction);
  // long press event.
  rotarySwitch.attachLongPressStart(myRotaryLongPressFunction);

  startButton.attachClick(myStartFunction);                          // Bouton Start/Stop
  startButton.attachLongPressStart(myStartStartLongPressFunction);   // Lance la mesure de la batterie
  startButton.attachDuringLongPress(myStartDuringLongPressFunction); // Affiche la charge de la batterie
  startButton.attachLongPressStop(myStartStopLongPressFunction);     // Retour à l'affichage normal

  buttonPreset1.attachClick(myPreset1Function);
  buttonPreset2.attachClick(myPreset2Function);
  buttonPreset3.attachClick(myPreset3Function);

  //pinMode(PIN_BUZZER, OUTPUT);
  DDRB |= (1 << DDB3);
  // Transistor sur D6
  DDRD |= (1 << DDD6);
  PORTD |= (1 << PORTD6);
  // digitalWrite(PIN_TRANS, HIGH);

  // Pins D3 (PD3), D12 (PB4), A1 (PC1), A4 (PC4) et A5 (PC5) ne sont pas utilisés : à mettre à INPUT_PULLUP pour qu'ils ne soient pas flottants
  // Par défaut, ils sont à INPUT => pas besoin de modifier les registres DDR (Port Data Direction)
  // Il faut mettre les bits concernés de PORT à HIGH pour activer le pull-up
  PORTB |= (1 << PORTB4);
  PORTC |= ((1 << PORTC1) | (1 << PORTC4) | (1 << PORTC5));
  PORTD |= (1 << PORTD3);
  // SPI et I2C ne sont pas utlisés, on les désactive
  SPCR = 0; //disable SPI by setting SPCR register to 0
  power_spi_disable();
  power_twi_disable();

  display.begin();                        // initializes the display
  display.setBacklight(TM1637Brightness); // set the brightness
  display.print("init");
  delay(500);
  display.clear();

  countDown = timer;
  isTimer = false;
} // setup()

ISR (PCINT0_vect)
 {
 // handle pin change interrupt for D8 to D13 here
 }
// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect)
{
  rotaryEncoder.tick(); // just call tick() to check the state.
}
// ISR for INT0
void wakeUp()
{
  // Just a handler for the pin interrupt.
}
// Read the current position of the encoder and print out when changed.
void loop()
{
  static int resetBlinkCount = 0; // static : la valeur est préservée entre les loop()
  static bool ledState = true;    // sert à gérer les clignotements
  static int countBuzzer = 0;

  unsigned long now = millis();
  minutes = countDown / 60;
  seconds = countDown % 60;

  // keep watching the buttons:
  rotarySwitch.tick();
  startButton.tick();
  buttonPreset1.tick();
  buttonPreset2.tick();
  buttonPreset3.tick();

  if (myNextAction != COUNTDOWN_IDLE)
  {
    previousMillisSleepTimer = 0;
  }

  switch (myNextAction)
  {
  case COUNTDOWN_IDLE:
    isRotaryActive = false;
    display.on();
    noTone(PIN_BUZZER);
    if (previousMillisSleepTimer == 0)
    {
      // déclenche le timer pour la mise en veille
      previousMillisSleepTimer = now;
    }
    if (now - previousMillisSleepTimer > sleepTime)
    {
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
    if (now - previousMillisTimer >= 1000)
    {
      previousMillisTimer = now;
      countDown--;
      if (countDown <= 10)
      {
        myNextAction = COUNTDOWN_2;
      }
    }
    break;
  case COUNTDOWN_2:
    if (now - previousMillisTimer >= 1000)
    {
      previousMillisTimer = now;
      countDown--;
      tone(PIN_BUZZER, notes[0], 10);
      if (countDown <= 5)
      {
        myNextAction = COUNTDOWN_3;
      }
    }
    break;

  case COUNTDOWN_3:
    if (now - previousMillisTimer >= 1000)
    {
      previousMillisTimer = now;
      countDown--;
      tone(PIN_BUZZER, notes[0], 50);
      if (countDown == 0)
      {
        previousMillisAlarm = now;
        myNextAction = COUNTDOWN_END;
      }
    }
    break;

  case COUNTDOWN_END:
    tone(PIN_BUZZER, notes[countBuzzer], notesDurations[countBuzzer]); // Output sound frequency to buzzerPin
    delay(notesIntervals[countBuzzer]);
    countBuzzer = (countBuzzer == (sizeof(notes)/sizeof(notes[0])-1)) ? 0 : countBuzzer + 1;
    if (now - previousMillisAlarm >= alarmLength)
    {
      isTimer = false;
      myNextAction = COUNTDOWN_SLEEP;
    }
    break;

  case COUNTDOWN_SLEEP:
    noTone(PIN_BUZZER);
    // Interrupts sur les boutons presets
    PCICR |= (1 << PCIE0);   // enables Pin Change Interrupt 0 that covers the pins or Port B.
    PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts (PCIFR: Pin Change Interrupt Flag Register)
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2); // enables the interrupt for pins D8, D9, D10.    
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW);

    // Eteind et coupe l'alim du TM1637
    display.off();
    PORTD &= ~(1 << PORTD6);
    // //digitalWrite(PIN_TRANS, LOW);

    // Déséactive les interrupts du rotary encoder
    PCMSK1 &= ~(1 << PCINT10 | 1 << PCINT11);

    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0);

    // Désactive les interrupts sur les boutons presets
    PCICR &= ~(1 << PCIE0);   // disable Pin Change Interrupt 0 that covers the pins or Port B.
    PCMSK0 &= ~(1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2);
    // Réactive les interrupts du rotary encoder
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11); // This enables the interrupt for pin 2 and 3 of Port C.

    // rétablit l'alim du TM1637
    PORTD |= (1 << PORTD6);
    // // digitalWrite(PIN_TRANS, HIGH);
    display.begin();
    display.setBacklight(TM1637Brightness);
    display.clear();

    // Vérifie l'état de la batterie
    measureVoltage();
    if (Voltage < 3.5)
    {
      myNextAction = BATTERY_ALERT;
    }
    else
    {
      myNextAction = COUNTDOWN_IDLE;
    }
    break;

  case BATTERY_CHECK:
    measureVoltage();
    break;

  case BATTERY_SHOW:
    display.clear();
    if (Voltage < 3.5)
    {
      myNextAction = BATTERY_ALERT;
    }
    else
    {
      myNextAction = BATTERY_WAIT;
    }
    break;

  case BATTERY_ALERT:
    nextLedAction = LEDS_ALERT;
    break;

  case BATTERY_WAIT:
    // on attend
    nextLedAction = LEDS_BATT;
    break;
  }

  if (isRotaryActive)
  {
    int newPos = rotaryEncoder.getPosition();
    if (newPos < ROTARYMIN)
    {
      rotaryEncoder.setPosition(ROTARYMAX);
      newPos = ROTARYMAX;
    }
    if (newPos > ROTARYMAX)
    {
      rotaryEncoder.setPosition(ROTARYMIN);
      newPos = ROTARYMIN;
    }
    if (pos != newPos)
    {
      pos = newPos;
      if (myNextAction == ROTARY_SET_MINUTES)
      {
        minutes = pos;
      }
      else if (myNextAction == ROTARY_SET_SECONDS)
      {
        seconds = pos;
      }
      countDown = seconds + (minutes * 60);
    }
  }
  else
  {
    // empêche la valeur d'être actualisée quand on tourne le rotary pendant que isRotaryActive est false
    rotaryEncoder.setPosition(pos);
  }

  // Gestion des clignotements
  if (now - previousMillisBlink >= 125 && (myNextAction == COUNTDOWN_3 || myNextAction == COUNTDOWN_END))
  {
    if (ledState)
    {
      nextLedAction = LEDS_11;
    }
    else
    {
      nextLedAction = LEDS_00;
    }
    ledState = !ledState;
    previousMillisBlink = now;
  }
  else if (now - previousMillisBlink >= 250)
  {
    if (myNextAction == COUNTDOWN_IDLE)
    {
      nextLedAction = LEDS_11;
    }
    else if (myNextAction == ROTARY_SET_MINUTES)
    {
      if (ledState)
      {
        nextLedAction = LEDS_11;
      }
      else
      {
        nextLedAction = LEDS_01;
      }
    }
    else if (myNextAction == ROTARY_SET_SECONDS)
    {
      if (ledState)
      {
        nextLedAction = LEDS_11;
      }
      else
      {
        nextLedAction = LEDS_10;
      }
    }
    else if (myNextAction == ROTARY_RESET || myNextAction == COUNTDOWN_2)
    {
      if (ledState)
      {
        nextLedAction = LEDS_11;
      }
      else
      {
        nextLedAction = LEDS_00;
      }
      if (myNextAction == ROTARY_RESET)
      {
        resetBlinkCount++;
        if (resetBlinkCount == 4)
        {
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
    // 8 barres par défaut
    buffer[0] = B00110110;
    buffer[1] = B00110110;
    buffer[2] = B00110110;
    buffer[3] = B00110110;
    if (Voltage > 3.91 && Voltage <= 3.995)
    {
      // 7 barres
      buffer[0] = B00000110;
    }
    else if (Voltage > 3.87 && Voltage <= 3.91)
    {
      // 6 barres
      buffer[0] = B00000000;
    }
    else if (Voltage > 3.775 && Voltage <= 3.87)
    {
      // 5 barres
      buffer[0] = B00000000;
      buffer[1] = B00000110;
    }
    else if (Voltage > 3.72 && Voltage <= 3.775)
    {
      // 4 barres
      buffer[0] = B00000000;
      buffer[1] = B00000000;
    }
    else if (Voltage > 3.695 && Voltage <= 3.72)
    {
      // 3 barres
      buffer[0] = B00000000;
      buffer[1] = B00000000;
      buffer[2] = B00000110;
    }
    else if (Voltage > 3.67 && Voltage <= 3.695)
    {
      // 2 barres
      buffer[0] = B00000000;
      buffer[1] = B00000000;
      buffer[2] = B00000000;
    }
    else if (Voltage > 3.5 && Voltage <= 3.67)
    {
      // 1 barre
      buffer[0] = B00000000;
      buffer[1] = B00000000;
      buffer[2] = B00000000;
      buffer[3] = B00000110;
    }
    break;
  case LEDS_ALERT:
    display.setColonOn(false);
    // display : "Batt"
    buffer[0] = B01111111; // "B"
    buffer[1] = B01011111; // "a"
    buffer[2] = B01111000; // "t"
    buffer[3] = B01111000; // "t"
    break;
  } // end of switch

  display.printRaw(buffer, 4, 0);
} // loop ()

void measureVoltage()
{
  // On commence par réactiver l'ADC
  power_adc_enable();
  ADCSRA |= (1 << ADEN); // 'ADC Enable' Writing this bit to one enables the ADC (datasheet p319)
  // power_adc_enable() en premier, sinon ça ne marche pas !!!
  delay (20);  // let it stabilize

  Vcc = readVcc() / 1000.0;
  ADCValue = analogRead(0);
  Voltage = (ADCValue / 1024.0) * Vcc;
  isVoltage = true;

  // Désactive l'ADC
  // You must use the PRR after setting ADCSRA to zero, otherwise the ADC is "frozen" in an active state.
  // voir http://www.gammon.com.au/power
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
}

// this function will be called when the button was pressed 1 time and them some time has passed.
void myRotaryClickFunction()
{
  if (!isTimer)
  {
    switch (myNextAction)
    {
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
  }
} // myRotaryClickFunction

// this function will be called when the button was pressed during 1 second.
void myRotaryLongPressFunction()
{
  if (myNextAction == COUNTDOWN_SLEEP)
  {
    myNextAction = COUNTDOWN_IDLE;
  }
  else if (myNextAction == COUNTDOWN_IDLE)
  {
    // Mise à zéro du compteur
    myNextAction = ROTARY_RESET;
  }
}

void myPreset1Function()
{
  if (myNextAction == COUNTDOWN_IDLE)
  {
    previousMillisSleepTimer = 0;
    countDown = presets[0];
  }
}
void myPreset2Function()
{
  if (myNextAction == COUNTDOWN_IDLE)
  {
    previousMillisSleepTimer = 0;
    countDown = presets[1];
  }
}
void myPreset3Function()
{
  if (myNextAction == COUNTDOWN_IDLE)
  {
    previousMillisSleepTimer = 0;
    countDown = presets[2];
  }
}

void myStartFunction()
{
  if (myNextAction == COUNTDOWN_IDLE)
  {
    if (!isTimer && countDown != 0) {
      isTimer = true;
      if (countDown <= 5) {
        myNextAction = COUNTDOWN_3;
      } else if (countDown <= 10) {
        myNextAction = COUNTDOWN_2;
      } else {
        myNextAction = COUNTDOWN_1;
      }
    } else {
      previousMillisSleepTimer = millis();
    }
  }
  else
  {
    // Stop
    isTimer = false;
    myNextAction = COUNTDOWN_IDLE;
  }
}

void myStartStartLongPressFunction()
{
  if (myNextAction == COUNTDOWN_IDLE)
  {
    myNextAction = BATTERY_CHECK;
  }
}

void myStartDuringLongPressFunction()
{
  if (isVoltage && myNextAction == BATTERY_CHECK)
  {
    myNextAction = BATTERY_SHOW;
  }
}

void myStartStopLongPressFunction()
{
  if (myNextAction == BATTERY_WAIT)
  {
    isVoltage = false;
    display.clear();
    myNextAction = COUNTDOWN_IDLE;
  }
}
