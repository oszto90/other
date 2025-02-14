/*
   Burn-out Control System

   Created by Oszto
   Created Date: 2017.10.22.
   Last Modify Date: 2017.11.27.
   Hardware version: 2.0
   Software version: 1.1

   Details:
   8 csatornán méri az áramokat és jelez ha kint van a tűeésből.

   HW:2.0:
   Új vezérlődobozra integrálva.

   SW:1.0:
   Az alap program.

   SW:1.1:
    A mérés átprogramozása a mérési hibák száma alapján. 

*/
#include <Wire.h>
#include <Time.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROMex.h>
#include <SPI.h>
#include "EmonLib.h"
//#include <Adafruit_Sensor.h>


//Verzió változók
String hwver = "2.0";
String swver = "1.1";
String lastupdate = "2017.11.27.";

//Terminál
boolean stringComplete = false;
String inputString = "";

//PROGMEM
const unsigned int maxAllowedWrites PROGMEM = 4000;
const unsigned int memBase PROGMEM = 0;
const unsigned int CS1MaxAddr PROGMEM = 0;
const unsigned int CS1MinAddr PROGMEM = 50;
const unsigned int CS2MaxAddr PROGMEM = 100;
const unsigned int CS2MinAddr PROGMEM = 150;
const unsigned int CS3MaxAddr PROGMEM = 200;
const unsigned int CS3MinAddr PROGMEM = 250;
const unsigned int CS4MaxAddr PROGMEM = 300;
const unsigned int CS4MinAddr PROGMEM = 350;
const unsigned int CS5MaxAddr PROGMEM = 400;
const unsigned int CS5MinAddr PROGMEM = 450;
const unsigned int CS6MaxAddr PROGMEM = 500;
const unsigned int CS6MinAddr PROGMEM = 550;
const unsigned int CS7MaxAddr PROGMEM = 600;
const unsigned int CS7MinAddr PROGMEM = 650;
const unsigned int CS8MaxAddr PROGMEM = 700;
const unsigned int CS8MinAddr PROGMEM = 750;
const unsigned int CS1CalAddr PROGMEM = 800;
const unsigned int CS2CalAddr PROGMEM = 850;
const unsigned int CS3CalAddr PROGMEM = 900;
const unsigned int CS4CalAddr PROGMEM = 950;
const unsigned int CS5CalAddr PROGMEM = 1000;
const unsigned int CS6CalAddr PROGMEM = 1050;
const unsigned int CS7CalAddr PROGMEM = 1100;
const unsigned int CS8CalAddr PROGMEM = 1150;
const unsigned int mestimeAddr PROGMEM = 1800;
const unsigned int updatetimeAddr PROGMEM = 2000;




//Mérési változók
float MCS1 = 0;
float MCS2 = 0;
float MCS3 = 0;
float MCS4 = 0;
float MCS5 = 0;
float MCS6 = 0;
float MCS7 = 0;
float MCS8 = 0;
int MCS1err = 0;
int MCS2err = 0;
int MCS3err = 0;
int MCS4err = 0;
int MCS5err = 0;
int MCS6err = 0;
int MCS7err = 0;
int MCS8err = 0;
int maxhiba = 20;
float MCS1Max = 0;
float MCS2Max = 0;
float MCS3Max = 0;
float MCS4Max = 0;
float MCS5Max = 0;
float MCS6Max = 0;
float MCS7Max = 0;
float MCS8Max = 0;
float MCS1Min = 0;
float MCS2Min = 0;
float MCS3Min = 0;
float MCS4Min = 0;
float MCS5Min = 0;
float MCS6Min = 0;
float MCS7Min = 0;
float MCS8Min = 0;
int updatetime = 5000;
unsigned long lastupdate2 = 0;
unsigned long lastupdate3 = 0;
unsigned long mestime = 0;
unsigned long mesupdate1 = 0;
unsigned long mesupdate2 = 0;
unsigned long mesupdate3 = 0;
unsigned long mesupdate4 = 0;
unsigned long mesupdate5 = 0;
unsigned long mesupdate6 = 0;
unsigned long mesupdate7 = 0;
unsigned long mesupdate8 = 0;

uint32_t delayMS;

//Senosor adatok 

float MCS1Cal = 0;
float MCS2Cal = 0;
float MCS3Cal = 0;
float MCS4Cal = 0;
float MCS5Cal = 0;
float MCS6Cal = 0;
float MCS7Cal = 0;
float MCS8Cal = 0;

//Sysetm adatok
boolean automode = HIGH;
boolean onoff = LOW;

//Relay's & Input's
const byte CS1Work PROGMEM = 12;
const byte CS2Work PROGMEM = 11;
const byte CS3Work PROGMEM = 22;
const byte CS4Work PROGMEM = 23;
const byte CS5Work PROGMEM = 24;
const byte CS6Work PROGMEM = 25;
const byte CS7Work PROGMEM = 26;
const byte CS8Work PROGMEM = 27;

const byte errorpin PROGMEM = 32;
const byte felpin PROGMEM = 42;
const byte lepin PROGMEM = 48; //OK
const byte okpin PROGMEM = 41; //OK
const byte quitpin PROGMEM = 44; //OK
const byte onoffpin PROGMEM = 49; //OK
//const byte EmrgStop PROGMEM = 12;
const byte piezo PROGMEM = 46;
const byte cled PROGMEM = 5;
//Gombok előző állapota
boolean lastcontrol = LOW;
boolean lastfel = LOW;
boolean lastle = LOW;
boolean lastok = LOW;
boolean lastquit = LOW;
boolean lastonoff = LOW;
//Történ e esemény a gombsoron és ha igen akkor mi?
boolean event = HIGH;
byte eventin = 0;
//Error event változók
byte errorcode = 0;
boolean lastemrgstop = LOW;
//Relay állapotok
boolean syserror = LOW;

//Sensor analóg lábak
const byte CS1In PROGMEM = A4;
const byte CS2In PROGMEM = A3;
const byte CS3In PROGMEM = A2;
const byte CS4In PROGMEM = A1;
const byte CS5In PROGMEM = A12;
const byte CS6In PROGMEM = A13;
const byte CS7In PROGMEM = A15;
const byte CS8In PROGMEM = A14;
boolean errorcode1 = LOW;
boolean errorcode2 = LOW;
boolean errorcode3 = LOW;
boolean errorcode4 = LOW;
boolean errorcode5 = LOW;
boolean errorcode6 = LOW;
boolean errorcode7 = LOW;
boolean errorcode8 = LOW;

//Kijelző változól
boolean mainpos = LOW; //Low: alap; HIGH: Menü
byte mpos = 0; //Főmenü pozíció
byte spos = 0; //Almenü pozíció
boolean medit = LOW; //szerkesztő mód
boolean submenu = LOW; //Főmenüben vagy almenüben vagyunk?
float idszam = 0; //ideiglenes érték tároló - Menüből változtatáshoz
boolean iddont = LOW;
int reftime = 2;
int lastref = 0;
//Idő beállítás változók
byte setSec = 0;
byte setMin = 0;
byte setHour = 0;
byte setDay = 1;
byte setMonth = 1;
int setYear = 2015;
int setpos = 0;
//LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//Current meter instance
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;
EnergyMonitor emon4;
EnergyMonitor emon5;
EnergyMonitor emon6;
EnergyMonitor emon7;
EnergyMonitor emon8;


void setup() {

  //Digitális lábak beállítása
  digitalWrite(errorpin, HIGH);
  pinMode(errorpin, OUTPUT);
  pinMode(felpin, INPUT_PULLUP);
  pinMode(lepin, INPUT_PULLUP);
  pinMode(okpin, INPUT_PULLUP);
  pinMode(quitpin, INPUT_PULLUP);
  pinMode(onoffpin, INPUT_PULLUP);
  pinMode(piezo, HIGH);
   pinMode(piezo, HIGH);
 // pinMode(EmrgStop, INPUT_PULLUP);
  pinMode(CS1Work, INPUT_PULLUP);
  pinMode(CS2Work, INPUT_PULLUP);
  pinMode(CS3Work, INPUT_PULLUP);
  pinMode(CS4Work, INPUT_PULLUP);
  pinMode(CS5Work, INPUT_PULLUP);
  pinMode(CS6Work, INPUT_PULLUP);
  pinMode(CS7Work, INPUT_PULLUP);
  pinMode(CS8Work, INPUT_PULLUP);
  //LCD config

  lcd.begin(20, 4);

  lcd.clear();
  lcd.noBacklight();
  delay(250);
  lcd.backlight();
  delay(250);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Burnout Control Sys."));
  lcd.setCursor(2, 1);
  lcd.print(F("Created by OsztO"));

  lcd.setCursor(4, 2);
  lcd.print(F("Loading....."));
  delay(1000);
  //Serial config
  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor
  delay(200);
  lcd.setCursor(4, 3);
  lcd.print(F("Serial Load"));
  delay(500);
  //EEPROM config
  EEPROM.setMemPool(memBase, EEPROMSizeMega);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  lcd.setCursor(4, 3);
  lcd.print(F("EEPROM Config"));
  delay(500);
  //EEPROM írása --- Ideiglenes (Feltöltés indulási adatokkal.)
  if (!digitalRead(felpin) && !digitalRead(lepin)) {
    EEPROM.writeFloat(CS1MaxAddr, 2000);
    EEPROM.writeFloat(CS1MinAddr, 1000);
    EEPROM.writeFloat(CS1CalAddr, 70);
    EEPROM.writeFloat(CS2MaxAddr, 2000);
    EEPROM.writeFloat(CS2MinAddr, 1000);
    EEPROM.writeFloat(CS2CalAddr, 70);
    EEPROM.writeFloat(CS3MaxAddr, 2000);
    EEPROM.writeFloat(CS3MinAddr, 1000);
    EEPROM.writeFloat(CS3CalAddr, 70);
    EEPROM.writeFloat(CS4MaxAddr, 2000);
    EEPROM.writeFloat(CS4MinAddr, 1000);
    EEPROM.writeFloat(CS4CalAddr, 70);
    EEPROM.writeFloat(CS5MaxAddr, 2000);
    EEPROM.writeFloat(CS5MinAddr, 1000);
    EEPROM.writeFloat(CS5CalAddr, 70);
    EEPROM.writeFloat(CS6MaxAddr, 2000);
    EEPROM.writeFloat(CS6MinAddr, 1000);
    EEPROM.writeFloat(CS6CalAddr, 70);
    EEPROM.writeFloat(CS7MaxAddr, 2000);
    EEPROM.writeFloat(CS7MinAddr, 1000);
    EEPROM.writeFloat(CS7CalAddr, 70);
    EEPROM.writeFloat(CS8MaxAddr, 2000);
    EEPROM.writeFloat(CS8MinAddr, 1000);
    EEPROM.writeFloat(CS8CalAddr, 70);
    EEPROM.writeFloat(updatetimeAddr, 5000);
    EEPROM.writeFloat(mestimeAddr, 1000);
  }
  //--Ideiglenes szakasz vége

  //EEPROM adatbetöltés
  lcd.setCursor(4, 3);
  lcd.print(F("EEPROM Read...."));
  delay(500);

  MCS1Max = EEPROM.readFloat(CS1MaxAddr);
  MCS1Min = EEPROM.readFloat(CS1MinAddr);
  MCS1Cal = EEPROM.readFloat(CS1CalAddr);
  MCS2Max = EEPROM.readFloat(CS2MaxAddr);
  MCS2Min = EEPROM.readFloat(CS2MinAddr);
  MCS2Cal = EEPROM.readFloat(CS2CalAddr);
  MCS3Max = EEPROM.readFloat(CS3MaxAddr);
  MCS3Min = EEPROM.readFloat(CS3MinAddr);
  MCS3Cal = EEPROM.readFloat(CS3CalAddr);
  MCS4Max = EEPROM.readFloat(CS4MaxAddr);
  MCS4Min = EEPROM.readFloat(CS4MinAddr);
  MCS4Cal = EEPROM.readFloat(CS4CalAddr);
  MCS5Max = EEPROM.readFloat(CS5MaxAddr);
  MCS5Min = EEPROM.readFloat(CS5MinAddr);
  MCS5Cal = EEPROM.readFloat(CS5CalAddr);
  MCS6Max = EEPROM.readFloat(CS6MaxAddr);
  MCS6Min = EEPROM.readFloat(CS6MinAddr);
  MCS6Cal = EEPROM.readFloat(CS6CalAddr);
  MCS7Max = EEPROM.readFloat(CS7MaxAddr);
  MCS7Min = EEPROM.readFloat(CS7MinAddr);
  MCS7Cal = EEPROM.readFloat(CS7CalAddr);
  MCS8Max = EEPROM.readFloat(CS8MaxAddr);
  MCS8Min = EEPROM.readFloat(CS8MinAddr);
  MCS8Cal = EEPROM.readFloat(CS8CalAddr);
 
  updatetime = EEPROM.readFloat(updatetimeAddr);
  mestime = EEPROM.readFloat(mestimeAddr);
  

// Measurement config
lcd.setCursor(4, 3);
  lcd.print(F("Current Config"));
  delay(500);
emon1.current(CS1In, MCS1Cal);
emon2.current(CS2In, MCS2Cal);
emon3.current(CS3In, MCS3Cal);
emon4.current(CS4In, MCS4Cal);
emon5.current(CS5In, MCS5Cal);
emon6.current(CS6In, MCS6Cal);
emon7.current(CS7In, MCS7Cal);
emon8.current(CS8In, MCS8Cal);


  //Start
  lcd.setCursor(4, 3);
  lcd.print(F("  Starting     "));
  delay(500);

  //Welcome screen
  Serial.println(F("Burnout Control System"));
  Serial.println(F("Created by OsztO"));
  Serial.println(F("Created Date: 2017.10.22."));
  Serial.println(F("Last Modify Date: "));
  Serial.println(lastupdate);
  Serial.print(F("Hardware version: "));
  Serial.println(hwver);
  Serial.print(F("Software version: "));
  Serial.println(swver);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Burnout Control Sys."));
  lcd.setCursor(2, 1);
  lcd.print(F("Created by OsztO"));
  lcd.setCursor(4, 2);
  lcd.print(F("HW ver.: "));
  lcd.print(hwver);
  lcd.setCursor(4, 3);
  lcd.print(F("SW ver.: "));
  lcd.print(swver);
  delay(3000);
  lcd.clear();

  Serial.println(F("Adj meg egy parancsot: (-help)"));
  Serial.print(F(">"));


}

void loop() {
  buttonact();
  screen();
  unsigned long currentMillis = millis();
  if (currentMillis -  lastupdate2 >= updatetime) {
    lastupdate2 = currentMillis;
    automanipulator();
    checkError();
    ControlSignal();
  }

}



void ControlSignal() {
  if (lastcontrol) {
    lastcontrol = LOW;
  } else {
    lastcontrol = HIGH;
  }
  digitalWrite(cled, lastcontrol);
  digitalWrite(piezo, lastcontrol);
 /* if (errorcode != 0) {
    digitalWrite(piezo, lastcontrol);
  } else {
    digitalWrite(piezo, LOW);
  }*/
}


String ErrString(int e) {
  switch (e) {
    case 0:
      return "NO ERROR";
      break;
    case 1:
      return "ERROR";
      break;
    default:
      break;
  }
}

void checkError(){
if (MCS1err >= maxhiba || MCS2err >= maxhiba ||MCS3err >= maxhiba || MCS4err >= maxhiba || MCS5err >= maxhiba || MCS6err >= maxhiba || MCS7err >= maxhiba || MCS8err >= maxhiba){
if (errorcode1 || errorcode2 || errorcode3 || errorcode4 || errorcode5 || errorcode6 || errorcode7 || errorcode8){
  errorcode = 1;
  manipulator(HIGH);
} else {
  errorcode = 0;
  manipulator(LOW);
}
}
}

void automanipulator() {
  meter();
  if (automode) {
    if (onoff) {
      unsigned long currentMillis = millis();

       if (!digitalRead(CS1Work)){
      if (MCS1 > MCS1Max){
        errorcode1 = HIGH;
        MCS1err++;
       // manipulator(HIGH);
      } else if (MCS1 < MCS1Min){
        errorcode1 = HIGH;
        MCS1err++;
       // manipulator(HIGH);
      } else {
        errorcode1 = LOW;
        MCS1err = 0;
       // manipulator(LOW);
      }
      } 

 if (!digitalRead(CS2Work)){
      if (MCS2 > MCS2Max){
        errorcode2 = HIGH;
        MCS2err++;
       // manipulator(HIGH);
      } else if (MCS2 < MCS2Min){
        errorcode2 = HIGH;
        MCS2err++;
       // manipulator(HIGH);
      } else {
        errorcode2 = LOW;
        MCS2err = 0;
       // manipulator(LOW);
      }
      }

       if (!digitalRead(CS3Work)){
      if (MCS3 > MCS3Max){
        errorcode3 = HIGH;
        MCS3err++;
       // manipulator(HIGH);
      } else if (MCS3 < MCS3Min){
        errorcode3 = HIGH;
        MCS3err++;
       // manipulator(HIGH);
      } else {
        errorcode3 = LOW;
        MCS3err = 0;
        //manipulator(LOW);
      }
      }

 if (!digitalRead(CS4Work)){
      if (MCS4 > MCS4Max){
        errorcode4 = HIGH;
        MCS4err++;
        //manipulator(HIGH);
      } else if (MCS4 < MCS4Min){
        errorcode4 = HIGH;
        MCS4err++;
     //   manipulator(HIGH);
      } else {
        errorcode4 = LOW;
        MCS4err = 0;
      //  manipulator(LOW);
      }
      }

       if (!digitalRead(CS5Work)){
      if (MCS5 > MCS5Max){
        errorcode5 = HIGH;
        MCS5err++;
       // manipulator(HIGH);
      } else if (MCS5 < MCS5Min){
        errorcode5 = HIGH;
        MCS5err++;
        //manipulator(HIGH);
      } else {
        errorcode5 = LOW;
        MCS5err = 0;
        //manipulator(LOW);
      }
      }

 if (!digitalRead(CS6Work)){
      if (MCS6 > MCS6Max){
        errorcode6 = HIGH;
        MCS6err++;
       // manipulator(HIGH);
      } else if (MCS6 < MCS6Min){
        errorcode6 = HIGH;
        MCS6err++;
       // manipulator(HIGH);
      } else {
        errorcode6 = LOW;
        MCS6err = 0;
        //manipulator(LOW);
      }
      }

       if (!digitalRead(CS7Work)){
      if (MCS7 > MCS7Max){
        errorcode7 = HIGH;
        MCS7err++;
        //manipulator(HIGH);
      } else if (MCS7 < MCS7Min){
        errorcode7 = HIGH;
        MCS7err++;
        //manipulator(HIGH);
      } else {
        errorcode7 = LOW;
        MCS7err = 0;
       // manipulator(LOW);
      }
      }

       if (!digitalRead(CS8Work)){
      if (MCS8 > MCS8Max){
        errorcode8 = HIGH;
        MCS8err++;
       // manipulator(HIGH);
      } else if (MCS8 < MCS8Min){
        errorcode8 = HIGH;
        MCS8err++;
        //manipulator(HIGH);
      } else {
        errorcode8 = LOW;
        MCS8err = 0;
       // manipulator(LOW);
      }
      }
      
    } else {
      manipulator(LOW);
      MCS1err = 0;
      MCS2err = 0;
      MCS3err = 0;
      MCS4err = 0;
      MCS5err = 0;
      MCS6err = 0;
      MCS7err = 0;
      MCS8err = 0;
    }
  } else {
    manipulator(LOW);
  }
}

void buttonact() {
  //Kapcsolók állapota
  if (automode) {
    if (digitalRead(onoffpin)) {
      onoff = LOW;
    } else {
      onoff = HIGH;
    }
  }

  if (lastonoff != onoff) {
    lastonoff = onoff;
    event = HIGH;
     Serial.println("OnOff");
  }

  if (!digitalRead(okpin)) {
    if (!lastok) {
      lastok = HIGH;
      event = HIGH;
      eventin = 1;
      // Serial.println("OK");
    }
  } else {
    lastok = LOW;
  }

  if (!digitalRead(felpin)) {
    if (!lastfel) {
      lastfel = HIGH;
      event = HIGH;
      eventin = 2;
      // Serial.println("FEL");
    }
  } else {
    lastfel = LOW;
  }

  if (!digitalRead(lepin)) {
    if (!lastle) {
      lastle = HIGH;
      event = HIGH;
      eventin = 3;
      // Serial.println("LE");
    }
  } else {
    lastle = LOW;
  }

  if (!digitalRead(quitpin)) {
    if (!lastquit) {
      lastquit = HIGH;
      event = HIGH;
      eventin = 4;
      //Serial.println("Quit");
    }
  } else {
    lastquit = LOW;
  }


}

void screen() {
  if (!mainpos) {
    if (event) {
      if (eventin == 1) {
        eventin = 0;
        mainpos = HIGH;
      }
    }
    unsigned long currentMillis2 = millis();

    if (currentMillis2 -  lastupdate3 >= 1000) {
      lastupdate3 = currentMillis2;

      if (event && eventin == 0) {
        lcd.setCursor(0, 0);
        lcd.print(F("C1:    W        W:C2"));
        lcd.setCursor(0, 1);
        lcd.print(F("C3:    W        W:C4"));
        lcd.setCursor(0, 2);
        lcd.print(F("C5:    W        W:C6"));
        lcd.setCursor(0, 3);
        lcd.print(F("C7:    W        W:C8"));
        event = LOW;
      }


      if (MCS1 >= 1000) {
        lcd.setCursor(3, 0);
        lcd.print(MCS1, 0);
      } else if (MCS1 >= 100) {
        lcd.setCursor(3, 0);
        lcd.print(F(" "));
       // lcd.setCursor(5, 0);
        lcd.print(MCS1, 0);
      } else if (MCS1 >= 10) {
        lcd.setCursor(3, 0);
        lcd.print(F("  "));
       // lcd.setCursor(6, 0);
        lcd.print(MCS1, 0);
      } else {
        lcd.setCursor(3, 0);
        lcd.print(F("   "));
        //lcd.setCursor(7, 0);
        lcd.print(MCS1, 0);
      }

if (MCS2 >= 1000) {
        lcd.setCursor(12, 0);
        lcd.print(MCS2, 0);
      } else if (MCS2 >= 100) {
        lcd.setCursor(12, 0);
        lcd.print(F(" "));
        //lcd.setCursor(14, 0);
        lcd.print(MCS2, 0);
      } else if (MCS2 >= 10) {
        lcd.setCursor(12, 0);
        lcd.print(F("  "));
        //lcd.setCursor(15, 0);
        lcd.print(MCS2, 0);
      } else {
        lcd.setCursor(12, 0);
        lcd.print(F("   "));
       // lcd.setCursor(16, 0);
        lcd.print(MCS2, 0);
      }


 if (MCS3 >= 1000) {
        lcd.setCursor(3, 1);
        lcd.print(MCS3, 0);
      } else if (MCS3 >= 100) {
        lcd.setCursor(3, 1);
        lcd.print(F(" "));
       // lcd.setCursor(5, 0);
        lcd.print(MCS3, 0);
      } else if (MCS3 >= 10) {
        lcd.setCursor(3, 1);
        lcd.print(F("  "));
       // lcd.setCursor(6, 0);
        lcd.print(MCS3, 0);
      } else {
        lcd.setCursor(3, 1);
        lcd.print(F("   "));
        //lcd.setCursor(7, 0);
        lcd.print(MCS3, 0);
      }

if (MCS4 >= 1000) {
        lcd.setCursor(12, 1);
        lcd.print(MCS4, 0);
      } else if (MCS4 >= 100) {
        lcd.setCursor(12, 1);
        lcd.print(F(" "));
        //lcd.setCursor(14, 0);
        lcd.print(MCS4, 0);
      } else if (MCS4 >= 10) {
        lcd.setCursor(12, 1);
        lcd.print(F("  "));
        //lcd.setCursor(15, 0);
        lcd.print(MCS4, 0);
      } else {
        lcd.setCursor(12, 1);
        lcd.print(F("   "));
       // lcd.setCursor(16, 0);
        lcd.print(MCS4, 0);
      }


 if (MCS5 >= 1000) {
        lcd.setCursor(3, 2);
        lcd.print(MCS5, 0);
      } else if (MCS5 >= 100) {
        lcd.setCursor(3, 2);
        lcd.print(F(" "));
       // lcd.setCursor(5, 0);
        lcd.print(MCS5, 0);
      } else if (MCS5 >= 10) {
        lcd.setCursor(3, 2);
        lcd.print(F("  "));
       // lcd.setCursor(6, 0);
        lcd.print(MCS5, 0);
      } else {
        lcd.setCursor(3, 2);
        lcd.print(F("   "));
        //lcd.setCursor(7, 0);
        lcd.print(MCS5, 0);
      }

if (MCS6 >= 1000) {
        lcd.setCursor(12, 2);
        lcd.print(MCS6, 0);
      } else if (MCS6 >= 100) {
        lcd.setCursor(12, 2);
        lcd.print(F(" "));
        //lcd.setCursor(14, 0);
        lcd.print(MCS6, 0);
      } else if (MCS6 >= 10) {
        lcd.setCursor(12, 2);
        lcd.print(F("  "));
        //lcd.setCursor(15, 0);
        lcd.print(MCS6, 0);
      } else {
        lcd.setCursor(12, 2);
        lcd.print(F("   "));
       // lcd.setCursor(16, 0);
        lcd.print(MCS6, 0);
      }


      
       if (MCS7 >= 1000) {
        lcd.setCursor(3, 3);
        lcd.print(MCS7, 0);
      } else if (MCS7 >= 100) {
        lcd.setCursor(3, 3);
        lcd.print(F(" "));
       // lcd.setCursor(5, 0);
        lcd.print(MCS7, 0);
      } else if (MCS7 >= 10) {
        lcd.setCursor(3, 3);
        lcd.print(F("  "));
       // lcd.setCursor(6, 0);
        lcd.print(MCS7, 0);
      } else {
        lcd.setCursor(3, 3);
        lcd.print(F("   "));
        //lcd.setCursor(7, 0);
        lcd.print(MCS7, 0);
      }

if (MCS8 >= 1000) {
        lcd.setCursor(12, 3);
        lcd.print(MCS8, 0);
      } else if (MCS8 >= 100) {
        lcd.setCursor(12, 3);
        lcd.print(F(" "));
        //lcd.setCursor(14, 0);
        lcd.print(MCS8, 0);
      } else if (MCS8 >= 10) {
        lcd.setCursor(12, 3);
        lcd.print(F("  "));
        //lcd.setCursor(15, 0);
        lcd.print(MCS8, 0);
      } else {
        lcd.setCursor(12, 3);
        lcd.print(F("   "));
       // lcd.setCursor(16, 0);
        lcd.print(MCS8, 0);
      }
      }

if (errorcode1){
        lcd.setCursor(8, 0);
        lcd.print(F("<-"));
      } else {
        lcd.setCursor(8, 0);
        lcd.print(F("  "));
      }
      if (errorcode2){
        lcd.setCursor(10, 0);
        lcd.print(F("->"));
      } else {
        lcd.setCursor(10, 0);
        lcd.print(F("  "));
      }

      if (errorcode3){
        lcd.setCursor(8, 1);
        lcd.print(F("<-"));
      } else {
        lcd.setCursor(8, 1);
        lcd.print(F("  "));
      }
      if (errorcode4){
        lcd.setCursor(10, 1);
        lcd.print(F("->"));
      } else {
        lcd.setCursor(10, 1);
        lcd.print(F("  "));
      }


if (errorcode5){
        lcd.setCursor(8, 2);
        lcd.print(F("<-"));
      } else {
        lcd.setCursor(8, 2);
        lcd.print(F("  "));
      }
      if (errorcode6){
        lcd.setCursor(10, 2);
        lcd.print(F("->"));
      } else {
        lcd.setCursor(10, 2);
        lcd.print(F("  "));
      }

if (errorcode7){
        lcd.setCursor(8, 3);
        lcd.print(F("<-"));
      } else {
        lcd.setCursor(8, 3);
        lcd.print(F("  "));
      }
      if (errorcode8){
        lcd.setCursor(10, 3);
        lcd.print(F("->"));
      } else {
        lcd.setCursor(10, 3);
        lcd.print(F("  "));
      }

  } else {
    menu();
  }
}

void menu() {
  if (event) {
    switch (mpos) {
      case 0:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS1 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS1Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 9;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 1;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS1          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS1Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS1Max = idszam;
                    EEPROM.writeFloat(CS1MaxAddr, MCS1Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS1Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS1Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS1Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS1Min = idszam;
                    EEPROM.writeFloat(CS1MinAddr, MCS1Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS1Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS1Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS1Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS1Cal = idszam;
                    EEPROM.writeFloat(CS1CalAddr, MCS1Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS1Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS1Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;
      

case 1:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS2 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS2Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 0;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 2;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS2          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS2Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS2Max = idszam;
                    EEPROM.writeFloat(CS2MaxAddr, MCS2Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS2Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS2Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS2Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS2Min = idszam;
                    EEPROM.writeFloat(CS2MinAddr, MCS2Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS2Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS2Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS2Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS2Cal = idszam;
                    EEPROM.writeFloat(CS2CalAddr, MCS2Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS2Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS2Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;



        case 2:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS3 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS3Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 1;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 3;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS3          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS3Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS3Max = idszam;
                    EEPROM.writeFloat(CS3MaxAddr, MCS3Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS3Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS3Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS3Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS3Min = idszam;
                    EEPROM.writeFloat(CS3MinAddr, MCS3Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS3Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS3Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS3Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS3Cal = idszam;
                    EEPROM.writeFloat(CS3CalAddr, MCS3Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS3Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS3Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;


        case 3:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS4 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS4Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 2;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 4;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS4          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS4Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS4Max = idszam;
                    EEPROM.writeFloat(CS4MaxAddr, MCS4Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS4Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS4Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS4Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS4Min = idszam;
                    EEPROM.writeFloat(CS4MinAddr, MCS4Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS4Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS4Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS4Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS4Cal = idszam;
                    EEPROM.writeFloat(CS4CalAddr, MCS4Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS4Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS4Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;

        case 4:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS5 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS5Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 3;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 5;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS5          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS5Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS5Max = idszam;
                    EEPROM.writeFloat(CS5MaxAddr, MCS5Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS5Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS5Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS5Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS5Min = idszam;
                    EEPROM.writeFloat(CS5MinAddr, MCS5Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS5Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS5Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS5Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS5Cal = idszam;
                    EEPROM.writeFloat(CS5CalAddr, MCS5Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS5Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS5Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;

        case 5:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS6 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS6Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 4;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 6;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS6          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS6Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS6Max = idszam;
                    EEPROM.writeFloat(CS6MaxAddr, MCS6Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS6Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS6Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS6Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS6Min = idszam;
                    EEPROM.writeFloat(CS6MinAddr, MCS6Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS6Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS6Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS6Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS6Cal = idszam;
                    EEPROM.writeFloat(CS6CalAddr, MCS6Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS6Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS6Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;


        case 6:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS7 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS7Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 5;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 7;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS7          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS7Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS7Max = idszam;
                    EEPROM.writeFloat(CS7MaxAddr, MCS7Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS7Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS7Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS7Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS7Min = idszam;
                    EEPROM.writeFloat(CS7MinAddr, MCS7Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS7Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS7Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS7Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS7Cal = idszam;
                    EEPROM.writeFloat(CS7CalAddr, MCS7Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS7Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS7Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;

        case 7:
        if (!submenu) {
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("     CS8 Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = MCS8Max;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 6;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 8;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> CS8          "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MAX Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS8Max;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS8Max = idszam;
                    EEPROM.writeFloat(CS8MaxAddr, MCS8Max);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS8Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS8Min;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F("   MIN Limit        "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" W   "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS8Min;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                     if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS8Min = idszam;
                    EEPROM.writeFloat(CS8MinAddr, MCS8Min);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS8Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
  if (idszam >= 2500)
                      idszam = 2500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = MCS8Cal;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 2:
              lcd.setCursor(0, 2);
              lcd.print(F("  Calibration value "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" value"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = MCS8Cal;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                       if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                    MCS8Cal = idszam;
                    EEPROM.writeFloat(CS8CalAddr, MCS8Cal);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 1;
                    idszam = MCS8Min;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.5;
                    if (idszam >= 500)
                      idszam = 500;
                    if (idszam <= 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    idszam = MCS8Max;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;

      
    case 8:

        if (!submenu) {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("   COMMON Setting   "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              idszam = updatetime;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 7;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 9;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> COMMON       "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F(" Updatetime Setting "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" msec         "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = updatetime;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 10000)
                      idszam = 10000;
                    if (idszam < 5000)
                      idszam = 5000;
                    updatetime = idszam;
                    EEPROM.writeFloat(updatetimeAddr, updatetime);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 1;
                    if (idszam > 10000)
                      idszam = 10000;
                    if (idszam < 5000)
                      idszam = 5000;
                  } else {
                    spos = 1;
                    idszam = mestime;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 10000)
                      idszam = 10000;
                    if (idszam < 5000)
                      idszam = 5000;
                  } else {
                    spos = 1;
                    idszam = mestime;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
            case 1:
              lcd.setCursor(0, 2);
              lcd.print(F(" Mes. Update time   "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" msec         "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = mestime;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= 5000)
                      idszam = 5000;
                    if (idszam <= 100)
                      idszam = 100;
                    mestime = idszam;
                    EEPROM.writeFloat(mestimeAddr, mestime);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 50;
                    if (idszam >= 5000)
                      idszam = 5000;
                    if (idszam <= 100)
                      idszam = 100;
                  } else {
                    spos = 0;
                    idszam = updatetime;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 50;
                    if (idszam >= 5000)
                      idszam = 5000;
                    if (idszam <= 100)
                      idszam = 100;
                  } else {
                    spos = 0;
                    idszam = updatetime;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;


      case 9:

        if (!submenu) {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("   ERROR Setting    "));
          lcd.setCursor(0, 3);
          lcd.print(F("                    "));
          switch (eventin) {
            case 4:
              eventin = 0;
              event = HIGH;
              mainpos = LOW;
              break;
            case 1:
              eventin = 0;
              spos = 0;
              submenu = HIGH;
              iddont = LOW;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 8;
              event = HIGH;
              break;
            case 3:
              eventin = 0;
              mpos = 0;
              event = HIGH;
              break;
            default:
              event = LOW;
              break;
          }
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU-> ERROR        "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   ERROR Reset      "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }

              if (iddont) {
                lcd.print(F("Start - Press OK  "));
              } else {
                lcd.print(F("Press->OK->UP->OK "));
              }

              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    iddont = LOW;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (iddont) {
                      lcd.setCursor(0, 3);
                      lcd.print(F("Error reset start..."));
                      delay(3000);
                      errorcode = 0;
                      errorcode1 = LOW;
                      errorcode2 = LOW;
                      errorcode3 = LOW;
                      errorcode4 = LOW;
                      errorcode5 = LOW;
                      errorcode6 = LOW;
                      errorcode7 = LOW;
                      errorcode8 = LOW;
                      syserror = LOW;
                      lastemrgstop = LOW;
                      relayupdate();
                      lcd.setCursor(0, 3);
                      lcd.print(F("   Reset Complete   "));
                      delay(3000);
                      iddont = LOW;
                    }
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    iddont = HIGH;
                  } else {
                    spos = 0;
                    iddont = LOW;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 0;
                    iddont = LOW;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;
          }
        }
        break;

    }
  }
}

void manipulator(boolean h) {
  /*
     g változó: 1: error channel;
     h változó: HIGH: Active; LOW: Off
  */
 
    if (h) {
        syserror = HIGH;
        relayupdate();
        delay(500);
    } else {
      syserror = LOW;
        relayupdate();
        delay(500);
    }
 

}

void relayupdate() {
  digitalWrite(errorpin, !syserror);
}

void meter() {
int minta = 10;
MCS1 = emon1.calcIrms(1480)*230;
MCS2 = emon2.calcIrms(1480)*230;
MCS3 = emon3.calcIrms(1480)*230;
MCS4 = emon4.calcIrms(1480)*230;
MCS5 = emon5.calcIrms(1480)*230;
MCS6 = emon6.calcIrms(1480)*230;
MCS7 = emon7.calcIrms(1480)*230;
MCS8 = emon8.calcIrms(1480)*230;
  /*for (int i=0;i < minta;i++){
MCS1 = MCS1 + (emon1.calcIrms(1480)*230);
MCS2 = MCS2 + (emon2.calcIrms(1480)*230);
MCS3 = MCS3 + (emon3.calcIrms(1480)*230);
MCS4 = MCS4 + (emon4.calcIrms(1480)*230);
MCS5 = MCS5 + (emon5.calcIrms(1480)*230);
MCS6 = MCS6 + (emon6.calcIrms(1480)*230);
MCS7 = MCS7 + (emon7.calcIrms(1480)*230);
MCS8 = MCS8 + (emon8.calcIrms(1480)*230);
  }

MCS1 = MCS1/minta;
MCS2 = MCS2/minta;
MCS3 = MCS3/minta;
MCS4 = MCS4/minta;
MCS5 = MCS5/minta;
MCS6 = MCS6/minta;
MCS7 = MCS7/minta;
MCS8 = MCS8/minta;
*/
if (MCS1 < 50)
MCS1 = 0;
if (MCS2 < 50)
MCS2 = 0;
if (MCS3 < 50)
MCS3 = 0;
if (MCS4 < 50)
MCS4 = 0;
if (MCS5 < 50)
MCS5 = 0;
if (MCS6 < 50)
MCS6 = 0;
if (MCS7 < 50)
MCS7 = 0;
if (MCS8 < 50)
MCS8 = 0;
}

void serialEvent() {
  while (Serial.available()) {

    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.print('0');
  }
  Serial.print(number);
}

void print2digitslcd(int number) {
  if (number >= 0 && number < 10) {
    lcd.print("0");
  }
  lcd.print(number);
}

void print2digitslcdures(int number) {
  if (number >= 0 && number < 10) {
    lcd.print(" ");
  }
  lcd.print(number);
}

