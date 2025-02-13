/*
   Magnetic Field System

   Created by Oszto
   Created Date: 2017.11.01.
   Last Modify Date: 2018.01.30.
   Hardware version: 2.0
   Software version: 1.0

   Details:
   4 csatornán még mágneses teret. A mérté értéket minősíti és visszajelez.

   HW:2.0:
   Új vezérlődobozra integrálva.

   SW:1.1:
    A CS2 csatorna Limit max léptetése pozitív irányba hibás volt.
   
   SW:1.0:
   Az alap program.

*/
#include <Wire.h>
#include <Time.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROMex.h>
#include <SPI.h>
#include "EmonLib.h"
#include <MCP3208.h>
//#include <Adafruit_Sensor.h>


//Verzió változók
String hwver = "2.0";
String swver = "1.1";
String lastupdate = "2018.01.30.";

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

const unsigned int CS1CalAddr PROGMEM = 800;
const unsigned int CS2CalAddr PROGMEM = 850;
const unsigned int CS3CalAddr PROGMEM = 900;
const unsigned int CS4CalAddr PROGMEM = 950;

const unsigned int updatetimeAddr PROGMEM = 2000;




//Mérési változók
float MCS1 = 0;
float MCS2 = 0;
float MCS3 = 0;
float MCS4 = 0;

float MCS1Max = 0;
float MCS2Max = 0;
float MCS3Max = 0;
float MCS4Max = 0;

float MCS1Min = 0;
float MCS2Min = 0;
float MCS3Min = 0;
float MCS4Min = 0;

int updatetime = 5000;
unsigned long lastupdate2 = 0;
unsigned long lastupdate3 = 0;
unsigned long mestime = 0;
unsigned long mesupdate1 = 0;
unsigned long mesupdate2 = 0;
unsigned long mesupdate3 = 0;
unsigned long mesupdate4 = 0;

uint32_t delayMS;

//Senosor adatok 
//#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG
// #define TOMILLIGAUSS 3756L  // For A1302: 1.3mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 3756mG
//#define TOMILLIGAUSS 0.48828125
 #define TOMILLIGAUSS 0.93900240
MCP3208 adc(A7);
float MCS1Cal = 0;
float MCS2Cal = 0;
float MCS3Cal = 0;
float MCS4Cal = 0;

//Sysetm adatok
boolean automode = HIGH;
boolean onoff = LOW;

//Relay's & Input's

const byte failpin PROGMEM = 32;
const byte passpin PROGMEM = 30;
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

boolean errorcode1 = LOW;
boolean errorcode2 = LOW;
boolean errorcode3 = LOW;
boolean errorcode4 = LOW;
boolean failstate = LOW;
boolean passstate = LOW;

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
LiquidCrystal_I2C lcd(0x23, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


void setup() {

  //Digitális lábak beállítása
  digitalWrite(failpin, HIGH);
  pinMode(failpin, OUTPUT);
  digitalWrite(passpin, HIGH);
  pinMode(passpin, OUTPUT);
  pinMode(felpin, INPUT_PULLUP);
  pinMode(lepin, INPUT_PULLUP);
  pinMode(okpin, INPUT_PULLUP);
  pinMode(quitpin, INPUT_PULLUP);
  pinMode(onoffpin, INPUT_PULLUP);
  pinMode(piezo, HIGH);
   pinMode(piezo, HIGH);
 // pinMode(EmrgStop, INPUT_PULLUP);
  //LCD config

  lcd.begin(20, 4);

  lcd.clear();
  lcd.noBacklight();
  delay(250);
  lcd.backlight();
  delay(250);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Magnetic Field  Sys."));
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
    EEPROM.writeFloat(CS1MaxAddr, -800);
    EEPROM.writeFloat(CS1MinAddr, -900);
    EEPROM.writeFloat(CS1CalAddr, 2476);
    EEPROM.writeFloat(CS2MaxAddr, 900);
    EEPROM.writeFloat(CS2MinAddr, 800);
    EEPROM.writeFloat(CS2CalAddr, 2488);
    EEPROM.writeFloat(CS3MaxAddr, -800);
    EEPROM.writeFloat(CS3MinAddr, -900);
    EEPROM.writeFloat(CS3CalAddr, 2488);
    EEPROM.writeFloat(CS4MaxAddr, 900);
    EEPROM.writeFloat(CS4MinAddr, 800);
    EEPROM.writeFloat(CS4CalAddr, 2488);
    EEPROM.writeFloat(updatetimeAddr, 500);
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
  
 
  updatetime = EEPROM.readFloat(updatetimeAddr);
  

// Measurement config
lcd.setCursor(4, 3);
  lcd.print(F("ADC Config"));
  delay(500);
adc.begin();


  //Start
  lcd.setCursor(4, 3);
  lcd.print(F("  Starting     "));
  delay(500);

  //Welcome screen
  Serial.println(F("Magnetic Field  Sys."));
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
  lcd.print(F("Magnetic Field  Sys."));
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
if (onoff){
if (errorcode1 || errorcode2 || errorcode3 || errorcode4){
  errorcode = 1;
  manipulator(2,LOW);
  manipulator(1,HIGH);
} else {
  errorcode = 0;
   manipulator(1,LOW);
  manipulator(2,HIGH);
}
} else  {
  manipulator(1,LOW);
   manipulator(2,LOW);
}
}

void automanipulator() {
  meter();
  if (automode) {
    if (onoff) {


      if (MCS1 > MCS1Max){
        errorcode1 = HIGH;
       // manipulator(HIGH);
      } else if (MCS1 < MCS1Min){
        errorcode1 = HIGH;
       // manipulator(HIGH);
      } else {
        errorcode1 = LOW;
       // manipulator(LOW);
      }
      

 
      if (MCS2 > MCS2Max){
        errorcode2 = HIGH;
       // manipulator(HIGH);
      } else if (MCS2 < MCS2Min){
        errorcode2 = HIGH;
       // manipulator(HIGH);
      } else {
        errorcode2 = LOW;
       // manipulator(LOW);
      }
      

      
      if (MCS3 > MCS3Max){
        errorcode3 = HIGH;
       // manipulator(HIGH);
      } else if (MCS3 < MCS3Min){
        errorcode3 = HIGH;
       // manipulator(HIGH);
      } else {
        errorcode3 = LOW;
        //manipulator(LOW);
      }
      


      if (MCS4 > MCS4Max){
        errorcode4 = HIGH;
        //manipulator(HIGH);
      } else if (MCS4 < MCS4Min){
        errorcode4 = HIGH;
     //   manipulator(HIGH);
      } else {
        errorcode4 = LOW;
      //  manipulator(LOW);
      }
     

      
      
    } else {
      manipulator(1,LOW);
      manipulator(2,LOW);
    }
  } else {
    manipulator(1,LOW);
      manipulator(2,LOW);
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

    if (currentMillis2 -  lastupdate3 >= 200) {
      lastupdate3 = currentMillis2;

      if (event && eventin == 0) {
        lcd.setCursor(0, 0);
        lcd.print(F("1:     G         G:2"));
        lcd.setCursor(0, 1);
        lcd.print(F("3:     G         G:4"));
        lcd.setCursor(0, 2);
        lcd.print(F("Prog:xx  Result:    "));
        lcd.setCursor(0, 3);
        lcd.print(F("                    "));
        event = LOW;
      }


      if (abs(MCS1) >= 1000) {
        lcd.setCursor(3, 0);
        lcd.print(abs(MCS1), 0);
      } else if (abs(MCS1) >= 100) {
        lcd.setCursor(3, 0);
        lcd.print(F(" "));
       // lcd.setCursor(5, 0);
        lcd.print(abs(MCS1), 0);
      } else if (abs(MCS1) >= 10) {
        lcd.setCursor(3, 0);
        lcd.print(F("  "));
       // lcd.setCursor(6, 0);
        lcd.print(abs(MCS1), 0);
      } else {
        lcd.setCursor(3, 0);
        lcd.print(F("   "));
        //lcd.setCursor(7, 0);
        lcd.print(abs(MCS1), 0);
      }

      if (MCS1 > 0){
        lcd.setCursor(2, 0);
        lcd.print(F("S"));
      } else if (MCS1 < 0) {
        lcd.setCursor(2, 0);
        lcd.print(F("N"));
      } else {
        lcd.setCursor(2, 0);
        lcd.print(F(" "));
      }

if (abs(MCS2) >= 1000) {
        lcd.setCursor(13, 0);
        lcd.print(abs(MCS2), 0);
      } else if (abs(MCS2) >= 100) {
        lcd.setCursor(13, 0);
        lcd.print(F(" "));
        //lcd.setCursor(14, 0);
        lcd.print(abs(MCS2), 0);
      } else if (abs(MCS2) >= 10) {
        lcd.setCursor(13, 0);
        lcd.print(F("  "));
        //lcd.setCursor(15, 0);
        lcd.print(abs(MCS2), 0);
      } else {
        lcd.setCursor(13, 0);
        lcd.print(F("   "));
       // lcd.setCursor(16, 0);
        lcd.print(abs(MCS2), 0);
      }


if (MCS2 > 0){
        lcd.setCursor(12, 0);
        lcd.print(F("S"));
      } else if (MCS2 < 0) {
        lcd.setCursor(12, 0);
        lcd.print(F("N"));
      } else {
        lcd.setCursor(12, 0);
        lcd.print(F(" "));
      }

 if (abs(MCS3) >= 1000) {
        lcd.setCursor(3, 1);
        lcd.print(abs(MCS3), 0);
      } else if (abs(MCS3) >= 100) {
        lcd.setCursor(3, 1);
        lcd.print(F(" "));
       // lcd.setCursor(5, 0);
        lcd.print(abs(MCS3), 0);
      } else if (abs(MCS3) >= 10) {
        lcd.setCursor(3, 1);
        lcd.print(F("  "));
       // lcd.setCursor(6, 0);
        lcd.print(abs(MCS3), 0);
      } else {
        lcd.setCursor(3, 1);
        lcd.print(F("   "));
        //lcd.setCursor(7, 0);
        lcd.print(abs(MCS3), 0);
      }

      if (MCS3 > 0){
        lcd.setCursor(2, 1);
        lcd.print(F("S"));
      } else if (MCS3 < 0) {
        lcd.setCursor(2, 1);
        lcd.print(F("N"));
      } else {
        lcd.setCursor(2, 1);
        lcd.print(F(" "));
      }

if (abs(MCS4) >= 1000) {
        lcd.setCursor(13, 1);
        lcd.print(abs(MCS4), 0);
      } else if (abs(MCS4) >= 100) {
        lcd.setCursor(13, 1);
        lcd.print(F(" "));
        //lcd.setCursor(14, 0);
        lcd.print(abs(MCS4), 0);
      } else if (abs(MCS4) >= 10) {
        lcd.setCursor(13, 1);
        lcd.print(F("  "));
        //lcd.setCursor(15, 0);
        lcd.print(abs(MCS4), 0);
      } else {
        lcd.setCursor(13, 1);
        lcd.print(F("   "));
       // lcd.setCursor(16, 0);
        lcd.print(abs(MCS4), 0);
      }

      if (MCS4 > 0){
        lcd.setCursor(12, 1);
        lcd.print(F("S"));
      } else if (MCS4 < 0) {
        lcd.setCursor(12, 1);
        lcd.print(F("N"));
      } else {
        lcd.setCursor(12, 1);
        lcd.print(F(" "));
      }


if (failstate && !passstate){
   lcd.setCursor(16, 2);
        lcd.print(F("FAIL"));
} else if (!failstate && passstate){
  lcd.setCursor(16, 2);
        lcd.print(F("PASS"));
} else {
  lcd.setCursor(16, 2);
        lcd.print(F("    "));
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
              lcd.print(F(" G   "));
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
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 2;
                    idszam = MCS1Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(F(" G   "));
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
                     if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 0;
                    idszam = MCS1Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
  if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(idszam, 0);
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
                       if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam += 1;
                    if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam -= 1;
                    if (idszam >= 4096)
                      idszam = 4096;
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
              lcd.print(F(" G   "));
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
                    idszam += 1;
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
                    idszam -= 1;
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
              lcd.print(F(" G   "));
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
                     if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 0;
                    idszam = MCS2Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
 if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(idszam, 0);
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
                       if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam += 1;
                    if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam -= 1;
                   if (idszam >= 4096)
                      idszam = 4096;
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
              lcd.print(F(" G   "));
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
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                     if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 2;
                    idszam = MCS3Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                   if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(F(" G   "));
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
                     if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 0;
                    idszam = MCS3Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
  if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(idszam, 0);
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
                       if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam += 1;
                   if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam -= 1;
                    if (idszam >= 4096)
                      idszam = 4096;
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
              mpos = 8;
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
              lcd.print(F(" G   "));
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
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                     if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 2;
                    idszam = MCS4Cal;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(F(" G   "));
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
                     if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
                    idszam += 1;
                    if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
                  } else {
                    spos = 0;
                    idszam = MCS4Max;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
  if (idszam >= 1000)
                      idszam = 1000;
                    if (idszam <= -1000)
                      idszam = -1000;
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
              lcd.print(idszam, 0);
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
                       if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam += 1;
                   if (idszam >= 4096)
                      idszam = 4096;
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
                    idszam -= 1;
                    if (idszam >= 4096)
                      idszam = 4096;
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
              mpos = 3;
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
                    if (idszam < 500)
                      idszam = 500;
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
                    idszam += 50;
                    if (idszam > 10000)
                      idszam = 10000;
                    if (idszam < 500)
                      idszam = 500;
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
                    if (idszam > 10000)
                      idszam = 10000;
                    if (idszam < 500)
                      idszam = 500;
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

void manipulator(int g, boolean h) {
  /*
     g változó: 1: FAIL channel;2: PASS channel
     h változó: HIGH: Active; LOW: Off
  */
 switch (g) {
  case 1:
    if (h) {
        failstate = HIGH;
        relayupdate();
    } else {
     failstate = LOW;
        relayupdate();
    }
    break;

    case 2:
if (h) {
  
        passstate = HIGH;
        relayupdate();
    } else {
      passstate = LOW;
        relayupdate();
    }
    break;
 }
}

void relayupdate() {
 // digitalWrite(errorpin, !syserror);
  digitalWrite(failpin, !failstate);
  digitalWrite(passpin, !passstate);
}

void meter() {

  MCS1 = (adc.analogRead(7) - MCS1Cal) * TOMILLIGAUSS;
  MCS2 = (adc.analogRead(6) - MCS2Cal) * TOMILLIGAUSS;
  MCS3 = (adc.analogRead(5) - MCS3Cal) * TOMILLIGAUSS;
  MCS4 = (adc.analogRead(4) - MCS4Cal) * TOMILLIGAUSS;
  /*if (gauss > 0)     Serial.println("(South pole)");
  else if(gauss < 0) Serial.println("(North pole)");
  else               Serial.println();*/
  
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

