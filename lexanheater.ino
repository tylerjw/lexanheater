/*************************************************** 
  This started out as an example from the Adafruit library
  for the MAX31855.h chip.  It was adapted to be a pid 
  controller for a lexan heating furnace.
  
  Arduino
  p1 - NC
  p2 - NC
  p3 - MAX31855
  p4 - MAX31855
  p5 - MAX31855
  p6 - relay
  p7 - LCD
  p8 - LCD
  p9 - LCD
  p10 - LCD
  p11 - LCD
  p12 - LCD
  p13 - green LED (PREHEATED_LED)
  p14 - start / stop button
  p15 - up button
  p16 - down button
  p17 - relay on led (red)
  p18
  p19
  
 ****************************************************/
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_MAX31855.h"
#include <LiquidCrystal.h>
#include <PID_v1.h>

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

// State machine
#define SETTING 0
#define HEATING 1

// button
#define START_STOP 14
#define UP         15
#define DOWN       16

// relay
#define RELAY      6
#define RELAY_LED  17
#define SAMPLE_TIME 1000
#define WINDOW_SIZE 5000

// preheated led
#define PREHEATED_LED  13

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// global variables;
double setTemp;
double curTemp;
double pidOutput;
int state = SETTING;
int relaystate = 0; // off
unsigned long windowStartTime;
unsigned long loopStartTime;
unsigned long loopTime;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&curTemp, &pidOutput, &setTemp, Kp, Ki, Kd, DIRECT);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif
  
void setup() {
  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

  pinMode(RELAY, OUTPUT);
  pinMode(RELAY_LED, OUTPUT);
  pinMode(START_STOP, INPUT);
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(PREHEATED_LED, OUTPUT);

  setTemp = 250; // start here
  
  lcd.clear();
  lcd.print("Lexan Heater");
  // wait for MAX chip to stabilize
  delay(500);

  myPID.SetOutputLimits(0, WINDOW_SIZE);
  myPID.SetMode(MANUAL); // default to off
  myPID.SetSampleTime(SAMPLE_TIME);

  windowStartTime = millis();
}

void loop() {
  loopStartTime = millis();

  // basic readout test, just print the current temp
   lcd.clear();
   lcd.setCursor(0, 1);
   Serial.print("Int. Temp = ");
   Serial.println(thermocouple.readInternal());
     
   curTemp = thermocouple.readFahrenheit();
   if (isnan(curTemp)) 
   {
      lcd.setCursor(0, 1);
      lcd.print("T/C Problem");
      relaystate = 0; // for now we don't want the relay on if there is a problem
      state = SETTING;
   } 
   else 
   {
      lcd.print("F = "); 
      lcd.print(int(curTemp));
      lcd.print(" ("); 
      lcd.print(int(setTemp));
      lcd.print(")");

      Serial.print("Thermocouple Temp = *");
      Serial.println(curTemp);

      if(curTemp >= (setTemp - 6) && curTemp <= (setTemp + 6)) {
        digitalWrite(PREHEATED_LED, LOW); // active low
      } else {
        digitalWrite(PREHEATED_LED, HIGH);
      }


      lcd.setCursor(0, 0);
      int startstop = digitalRead(START_STOP);
      int up = digitalRead(UP);
      int down = digitalRead(DOWN);
      switch(state) {
      case SETTING:
        lcd.print("UP  DOWN   START");
        if(up == 0) {
          setTemp += 10;
        }
        if(down == 0) {
          setTemp -= 10;
        }
        if(startstop == 0) {
          state = HEATING;
          myPID.SetMode(AUTOMATIC);
        }
        break;
      case HEATING:
        lcd.print("            STOP");
        if(startstop == 0) {
          state = SETTING;
          relaystate = 0;
          myPID.SetMode(MANUAL);
          break;
        }

        myPID.Compute();
        if(pidOutput < millis() - windowStartTime) {
          relaystate = 1; // turn on
        } else {
          relaystate = 0; // turn off
        }

        break;
      }
   }

   switch(relaystate) {
   case 0:
    digitalWrite(RELAY, LOW);
    digitalWrite(RELAY_LED, HIGH); // active low
    break;
   case 1:
    digitalWrite(RELAY, HIGH);
    digitalWrite(RELAY_LED, LOW); // active low
    break;
   }

   loopTime = millis() - loopStartTime;
   if(loopTime > SAMPLE_TIME) {
    Serial.print("Loop timing Problem\n");
    lcd.setCursor(0, 1);
    lcd.print("Loop timing Problem");
    state = SETTING;
    relaystate = 0;
    myPID.SetMode(MANUAL);
   } else {
    delay(SAMPLE_TIME - loopTime);
   }
}
