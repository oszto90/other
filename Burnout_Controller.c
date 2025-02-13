/*
   Air Pressure System

   Created by Oszto
   Created Date: 2015.12.13.
   Last Modify Date: 2016.11.02.
   Hardware version: 2.0
   Software version: 1.7

   Details:
   Két kompresszort képes vezérelni, ki lehet választani melyik legyen a preferált. Három üzemód van: 1.:Preferált#1 2.:Teljesítmény 3.:Preferált#2

   SW:1.7:
   UDP config/get
   Screen refresh method módosítás. RTC helyett millis()

   SW:1.6:
   Mérési értékek számolásának módosítása. A range alsó értékének kivonása a fesz értékből.
   A kalibrációs funkciók eltávolítva.
   RTC futásközi hiba javítása.

   SW:1.5:
   Webes felület létrehozása
   RTC boot hiba javítása

   SW:1.4:
   Hálózati kártya használható
   Bővítve a menü (Ethernet)

   SW:1.3:
   Bekerültek az alábbi funkciók:
   -Túlnyomás hiba érzékelése
   -Csatorna vezérlő kontaktor ellenőrzés
   -Túláram figyelő relék hibajelzése
   -E-stop funkció
   -Main kontaktor vezérlése

   HW:2.0:
   Új vezérlődobozra integrálva.

   SW:1.2:
   A nyomás értékek rosszul íródnak ki amikor prió váltás van.
   A nyomógombok a beépítéskor fordítva lettek beszerelve így fel kell cserélni szimetrikusan mindent
   Off állapotban nem frissül a nyomás adat.

   SW:1.1:
   Main Screen módosítás függölegesen szimetrikus nézetre.

   SW:1.0:
   Az alap program.

*/
#include <Wire.h>
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROMex.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>


//Verzió változók
String hwver = "2.0";
String swver = "1.7";
String lastupdate = "2016.11.02.";

//Terminál
boolean stringComplete = false;
String inputString = "";

//PROGMEM
const unsigned int maxAllowedWrites PROGMEM = 4000;
const unsigned int memBase PROGMEM = 0;
const unsigned int masterSPAddr PROGMEM = 0;
const unsigned int masterHysAddr PROGMEM = 50;
const unsigned int slaveSPAddr PROGMEM = 100;
const unsigned int slaveHysAddr PROGMEM = 150;
const unsigned int AcCalibAddr PROGMEM = 200;
const unsigned int BcCalibAddr PROGMEM = 250;
const unsigned int updatetimeAddr PROGMEM = 300;
const unsigned int mintaAddr PROGMEM = 350;
const unsigned int AcpsibarAddr PROGMEM = 400;
const unsigned int AcrangeAddr PROGMEM = 450;
const unsigned int AcrangemaxAddr PROGMEM = 500;
const unsigned int AcrangeminAddr PROGMEM = 550;
const unsigned int AcsensorrangeAddr PROGMEM = 600;
const unsigned int BcpsibarAddr PROGMEM = 650;
const unsigned int BcrangeAddr PROGMEM = 700;
const unsigned int BcrangemaxAddr PROGMEM = 750;
const unsigned int BcrangeminAddr PROGMEM = 800;
const unsigned int BcsensorrangeAddr PROGMEM = 850;
const unsigned int checkampfbAddr PROGMEM = 900;
const unsigned int checkrelayfbAddr PROGMEM = 950;
const unsigned int ip1Addr PROGMEM = 1000;
const unsigned int ip2Addr PROGMEM = 1050;
const unsigned int ip3Addr PROGMEM = 1100;
const unsigned int ip4Addr PROGMEM = 1150;
const unsigned int dns1Addr PROGMEM = 1200;
const unsigned int dns2Addr PROGMEM = 1250;
const unsigned int dns3Addr PROGMEM = 1300;
const unsigned int dns4Addr PROGMEM = 1350;
const unsigned int gateway1Addr PROGMEM = 1400;
const unsigned int gateway2Addr PROGMEM = 1450;
const unsigned int gateway3Addr PROGMEM = 1500;
const unsigned int gateway4Addr PROGMEM = 1550;
const unsigned int subnet1Addr PROGMEM = 1600;
const unsigned int subnet2Addr PROGMEM = 1650;
const unsigned int subnet3Addr PROGMEM = 1700;
const unsigned int subnet4Addr PROGMEM = 1750;
const unsigned int webportAddr PROGMEM = 1800;
const unsigned int udpportAddr PROGMEM = 1850;
const unsigned int dhcpAddr PROGMEM = 1900;

//Ethernet változók
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 15, 177);
IPAddress dnServer(192, 168, 15, 5);
IPAddress gateway(192, 168, 15, 254);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(80);
byte ip1 = 192;
byte ip2 = 168;
byte ip3 = 15;
byte ip4 = 177;
byte dns1 = 192;
byte dns2 = 168;
byte dns3 = 15;
byte dns4 = 5;
byte gateway1 = 192;
byte gateway2 = 168;
byte gateway3 = 15;
byte gateway4 = 254;
byte subnet1 = 255;
byte subnet2 = 255;
byte subnet3 = 255;
byte subnet4 = 0;
byte webport = 80;
unsigned int udpport = 8888;
boolean dhcp = HIGH;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char  ReplyStart[] = "start-data";       // a string to send back
char ReplyEnd[] = "data-end";
char ReplyAlive[] = "alive";
EthernetUDP Udp;

//Mérési változók
float masterSP = 0;
float masterHys = 0;
float slaveSP = 0;
float slaveHys = 0;
int AcCalib = 0;
int BcCalib = 0;
int updatetime = 0;
unsigned long lastupdate2 = 0;
unsigned long lastupdate3 = 0;

float AchAct = 0;
float BchAct = 0;
//Senosor adatok A channel
boolean Acpsibar = LOW;
int Acrange = 1000;
float Acrangemin = 0.5;
float Acrangemax = 4.5;
float Acsensorrange = 0.004; //0.5 to 4.5 mér a szenzor így 4/1000 mert 1000PSI ben mér.
//Sensor adatok B channel
boolean Bcpsibar = LOW;
int Bcrange = 1000;
float Bcrangemin = 0.5;
float Bcrangemax = 4.5;
float Bcsensorrange = 0.004; //0.5 to 4.5 mér a szenzor így 4/1000 mert 1000PSI ben mér.

const float barvalto PROGMEM = 0.0689475729; //PSI to BAR váltószám
const float felbontas PROGMEM = 0.004887585532746823069403714565; //Felbontás 5voltra 5/1023
int minta = 20;
boolean Acprio = LOW;
boolean Bcprio = LOW;
boolean automode = HIGH;
boolean onoff = LOW;
boolean AcRun = LOW;
boolean BcRun = LOW;

//Relay's & Input's
const byte Achannelpin PROGMEM = 40;
const byte Bchannelpin PROGMEM = 38;
const byte ATchannelpin PROGMEM = 39;
const byte BTchannelpin PROGMEM = 36;
const byte Acerrorpin PROGMEM = 37;
const byte Bcerrorpin PROGMEM = 34;

const byte MainContactpin PROGMEM = 33;
const byte felpin PROGMEM = 42;
const byte lepin PROGMEM = 48; //OK
const byte okpin PROGMEM = 41; //OK
const byte quitpin PROGMEM = 44; //OK
const byte Acpriopin PROGMEM = 47; //OK
const byte Bcpriopin PROGMEM = 49; //OK
const byte onoffpin PROGMEM = 45; //OK
const byte Acrelayfb PROGMEM = 22;
const byte Bcrelayfb PROGMEM = 23;
const byte Acampfb PROGMEM = 25;
const byte Bcampfb PROGMEM = 24;
const byte Acpresfb PROGMEM = 12;
const byte Bcpresfb PROGMEM = 11;
const byte EmrgStop PROGMEM = 27;
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
boolean checkrelayfb = HIGH;
boolean checkampfb = HIGH;
boolean MainContact = LOW;
//Relay állapotok
boolean Achannel = LOW;
boolean Bchannel = LOW;
boolean ATchannel = LOW;
boolean BTchannel = LOW;
boolean Acerror = LOW;
boolean Bcerror = LOW;
//Sensor analóg lábak
const byte sensorA PROGMEM = 8;
const byte sensorB PROGMEM = 9;
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
//LiquidCrystal_I2C lcd2(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// RTC Változók START
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;
//RTC END


void setup() {
  //RTC beállítása
  bool parse = false;
  bool config = false;

  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  //Digitális lábak beállítása
  digitalWrite(Achannelpin, HIGH);
  digitalWrite(Bchannelpin, HIGH);
  digitalWrite(ATchannelpin, HIGH);
  digitalWrite(BTchannelpin, HIGH);
  digitalWrite(Acerrorpin, HIGH);
  digitalWrite(Bcerrorpin, HIGH);
  digitalWrite(MainContactpin, HIGH);
  pinMode(Achannelpin, OUTPUT);
  pinMode(Bchannelpin, OUTPUT);
  pinMode(ATchannelpin, OUTPUT);
  pinMode(BTchannelpin, OUTPUT);
  pinMode(Acerrorpin, OUTPUT);
  pinMode(Bcerrorpin, OUTPUT);
  pinMode(MainContactpin, OUTPUT);
  pinMode(felpin, INPUT_PULLUP);
  pinMode(lepin, INPUT_PULLUP);
  pinMode(okpin, INPUT_PULLUP);
  pinMode(quitpin, INPUT_PULLUP);
  pinMode(Acpriopin, INPUT_PULLUP);
  pinMode(Bcpriopin, INPUT_PULLUP);
  pinMode(onoffpin, INPUT_PULLUP);
  pinMode(Acrelayfb, INPUT_PULLUP);
  pinMode(Bcrelayfb, INPUT_PULLUP);
  pinMode(Acampfb, INPUT_PULLUP);
  pinMode(Bcampfb, INPUT_PULLUP);
  pinMode(Acpresfb, INPUT_PULLUP);
  pinMode(Bcpresfb, INPUT_PULLUP);
  pinMode(EmrgStop, INPUT_PULLUP);
  //LCD config

  lcd.begin(20, 4);
  lcd2.begin(16, 2);

  lcd.clear();
  lcd2.clear();
  lcd.noBacklight();
  lcd2.noBacklight();
  delay(250);
  lcd.backlight();
  lcd2.backlight();
  delay(250);
  lcd.clear();
  lcd2.clear();
  lcd.setCursor(0, 0);
  lcd2.setCursor(0, 0);
  lcd.print(F("Air Pressure-System-"));
  lcd2.print(F("--Air Pressure--"));
  lcd.setCursor(2, 1);
  lcd2.setCursor(0, 1);
  lcd.print(F("Created by OsztO"));
  lcd2.print(F("Created by OsztO"));

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
    EEPROM.writeFloat(masterSPAddr, 8);
    EEPROM.writeFloat(masterHysAddr, 2);
    EEPROM.writeFloat(slaveSPAddr, 8);
    EEPROM.writeFloat(slaveHysAddr, 3);
    EEPROM.writeFloat(AcCalibAddr, 0);
    EEPROM.writeFloat(BcCalibAddr, 0);
    EEPROM.writeFloat(updatetimeAddr, 1000);
    EEPROM.writeFloat(mintaAddr, 20);
    EEPROM.writeFloat(AcpsibarAddr, LOW);
    EEPROM.writeFloat(AcrangeAddr, 150);
    EEPROM.writeFloat(AcrangemaxAddr, 4.5);
    EEPROM.writeFloat(AcrangeminAddr, 0.5);
    EEPROM.writeFloat(AcsensorrangeAddr, 0.004);
    EEPROM.writeFloat(BcpsibarAddr, LOW);
    EEPROM.writeFloat(BcrangeAddr, 150);
    EEPROM.writeFloat(BcrangemaxAddr, 4.5);
    EEPROM.writeFloat(BcrangeminAddr, 0.5);
    EEPROM.writeFloat(BcsensorrangeAddr, 0.004);
    EEPROM.writeFloat(checkampfbAddr, LOW);
    EEPROM.writeFloat(checkrelayfbAddr, LOW);
    EEPROM.writeFloat(ip1Addr, 192);
    EEPROM.writeFloat(ip2Addr, 168);
    EEPROM.writeFloat(ip3Addr, 15);
    EEPROM.writeFloat(ip4Addr, 177);
    EEPROM.writeFloat(dns1Addr, 192);
    EEPROM.writeFloat(dns2Addr, 168);
    EEPROM.writeFloat(dns3Addr, 15);
    EEPROM.writeFloat(dns4Addr, 5);
    EEPROM.writeFloat(gateway1Addr, 192);
    EEPROM.writeFloat(gateway2Addr, 168);
    EEPROM.writeFloat(gateway3Addr, 15);
    EEPROM.writeFloat(gateway4Addr, 254);
    EEPROM.writeFloat(subnet1Addr, 255);
    EEPROM.writeFloat(subnet2Addr, 255);
    EEPROM.writeFloat(subnet3Addr, 255);
    EEPROM.writeFloat(subnet4Addr, 0);
    EEPROM.writeFloat(webportAddr, 80);
    EEPROM.writeFloat(udpportAddr, 8888);
    EEPROM.writeFloat(dhcpAddr, LOW);
  }
  //--Ideiglenes szakasz vége

  //EEPROM adatbetöltés
  masterSP = EEPROM.readFloat(masterSPAddr);
  masterHys = EEPROM.readFloat(masterHysAddr);
  slaveSP = EEPROM.readFloat(slaveSPAddr);
  slaveHys = EEPROM.readFloat(slaveHysAddr);
  AcCalib = EEPROM.readFloat(AcCalibAddr);
  BcCalib = EEPROM.readFloat(BcCalibAddr);
  updatetime = EEPROM.readFloat(updatetimeAddr);
  minta = EEPROM.readFloat(mintaAddr);
  Acpsibar = EEPROM.readFloat(AcpsibarAddr);
  Acrange = EEPROM.readFloat(AcrangeAddr);
  Acrangemax  = EEPROM.readFloat(AcrangemaxAddr);
  Acrangemin = EEPROM.readFloat(AcrangeminAddr);
  Acsensorrange = EEPROM.readFloat(AcsensorrangeAddr);
  Bcpsibar = EEPROM.readFloat(BcpsibarAddr);
  Bcrange = EEPROM.readFloat(BcrangeAddr);
  Bcrangemax = EEPROM.readFloat(BcrangemaxAddr);
  Bcrangemin = EEPROM.readFloat(BcrangeminAddr);
  Bcsensorrange = EEPROM.readFloat(BcsensorrangeAddr);
  checkampfb = EEPROM.readFloat(checkampfbAddr);
  checkrelayfb = EEPROM.readFloat(checkrelayfbAddr);
  ip1 = EEPROM.readFloat(ip1Addr);
  ip2 = EEPROM.readFloat(ip2Addr);
  ip3 = EEPROM.readFloat(ip3Addr);
  ip4 = EEPROM.readFloat(ip4Addr);
  dns1 = EEPROM.readFloat(dns1Addr);
  dns2 = EEPROM.readFloat(dns2Addr);
  dns3 = EEPROM.readFloat(dns3Addr);
  dns4 = EEPROM.readFloat(dns4Addr);
  gateway1 = EEPROM.readFloat(gateway1Addr);
  gateway2 = EEPROM.readFloat(gateway2Addr);
  gateway3 = EEPROM.readFloat(gateway3Addr);
  gateway4 = EEPROM.readFloat(gateway4Addr);
  subnet1 = EEPROM.readFloat(subnet1Addr);
  subnet2 = EEPROM.readFloat(subnet2Addr);
  subnet3 = EEPROM.readFloat(subnet3Addr);
  subnet4 = EEPROM.readFloat(subnet4Addr);
  webport = EEPROM.readFloat(webportAddr);
  udpport = EEPROM.readFloat(udpportAddr);
  dhcp = EEPROM.readFloat(dhcpAddr);

  lcd.setCursor(4, 3);
  lcd.print(F("EEPROM Read...."));
  delay(500);

  lcd.setCursor(4, 3);
  lcd.print(F("ETHERNET Start "));
  delay(500);

  //Ethernet Starting
  IPAddress ip(ip1, ip2, ip3, ip4);
  IPAddress dnServer(dns1, dns2, dns3, dns4);
  IPAddress gateway(gateway1, gateway2, gateway3, gateway4);
  IPAddress subnet(subnet1, subnet2, subnet3, subnet4);
  EthernetServer server(webport);
  if (dhcp) {
    Ethernet.begin(mac);
  } else {
    Ethernet.begin(mac, ip, dnServer, gateway, subnet);
  }
  server.begin();
  Udp.begin(udpport);
  Serial.print(F("Server is at "));
  Serial.println(Ethernet.localIP());
  lcd.setCursor(4, 3);
  lcd.print(Ethernet.localIP());
  delay(1000);
  //Start
  lcd.setCursor(4, 3);
  lcd.print(F("  Starting     "));
  delay(500);

  //Welcome screen
  Serial.println(F("Air Pressure System"));
  Serial.println(F("Created by OsztO"));
  Serial.println(F("Created Date: 2015.12.13."));
  Serial.println(F("Last Modify Date: "));
  Serial.println(lastupdate);
  Serial.print(F("Hardware version: "));
  Serial.println(swver);
  Serial.print(F("Software version: "));
  Serial.println(swver);

  lcd.clear();
  lcd2.clear();
  lcd.setCursor(0, 0);
  lcd2.setCursor(0, 0);
  lcd2.print(F("HW ver.: "));
  lcd2.print(hwver);
  lcd2.setCursor(0, 1);
  lcd2.print(F("SW ver.: "));
  lcd2.print(swver);
  lcd.print(F("Air Pressure-System-"));
  lcd.setCursor(2, 1);
  lcd.print(F("Created by OsztO"));
  lcd.setCursor(4, 2);
  lcd.print(F("HW ver.: "));
  lcd.print(hwver);
  lcd.setCursor(4, 3);
  lcd.print(F("SW ver.: "));
  lcd.print(swver);
  delay(3000);
  lcd2.clear();
  lcd.clear();

  //Idő kiíratása
  if (RTC.read(tm)) {
    Serial.print(F("Time = "));
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(F(", Date (D/M/Y) = "));
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  }  else {
    if (RTC.chipPresent()) {
      Serial.println(F("The DS1307 is stopped.  Please run the SetTime"));
      Serial.println();
    } else {
      Serial.println(F("DS1307 read error!  Please check the circuitry."));
      Serial.println();
    }
  }
  //Idő adatok config után
  /*  if (parse && config) {
      Serial.print(F("DS1307 configured Time="));
      Serial.print(__TIME__);
      Serial.print(F(", Date="));
      Serial.println(__DATE__);
    } else if (parse) {
      Serial.println(F("DS1307 Communication Error :-{"));
      Serial.println(F("Please check your circuitry"));
    } else {
      Serial.print(F("Could not parse info from the compiler, Time=\""));
      Serial.print(__TIME__);
      Serial.print(F("\", Date=\""));
      Serial.print(__DATE__);
      Serial.println(F("\""));
    }
  */
  Serial.println(F("Adj meg egy parancsot: (-help)"));
  Serial.print(F(">"));


}

void loop() {
  Estop();
  if (stringComplete) {
    terminal();
  }
  buttonact();
  // UDPcontrol();
  EthernetLoad();
  screen();
  unsigned long currentMillis = millis();
  if (currentMillis -  lastupdate2 >= updatetime) {
    lastupdate2 = currentMillis;
    checkerror();
    automanipulator();
    ControlSignal();
  }

}

void UDPcontrol() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++)
    {
      Serial.print(remote[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    if (packetBuffer == "getconfig") {
      Udp.write(ReplyStart);
      Udp.write(masterSP);
      Udp.write(";");
      Udp.write(slaveSP);
      Udp.write(ReplyEnd);
    } else if (packetBuffer == "setconfig") {
      Udp.write(ReplyStart);
      Udp.write("Setup");
      Udp.write(ReplyEnd);
    } else {
      Udp.write(ReplyAlive);
    }
    Udp.endPacket();
  }
}

void EthernetLoad() {
  EthernetClient client = server.available();
  if (client) {
    // Serial.println();
    //Serial.println("--New ethernet client--");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        //Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          EthernetPage(client);
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    //Serial.println("--Ethernet client disconnected--");
  }
}

void EthernetPage(EthernetClient client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));  // the connection will be closed after completion of the response
  // client.println(F("Refresh: 5"));  // refresh the page automatically every 5 sec
  client.println();

  client.println(F("<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Transitional//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd'>"));
  client.println(F("<html xmlns='http://www.w3.org/1999/xhtml'>"));
  client.println(F("<head>"));
  client.println(F("<meta http-equiv='Content-Type' content='text/html; charset=utf-8' />"));
  client.println(F("<title>Air Pressure Controler</title>"));
  client.println(F("<style type='text/css'>"));
  client.println(F("<!--"));
  client.println(F(".MainTitle {"));
  client.println(F("text-align: center;"));
  client.println(F("font-family: 'Comic Sans MS';"));
  client.println(F("}"));
  client.println(F(".SubTitle {"));
  client.println(F("font-family: 'Comic Sans MS';"));
  client.println(F("text-align:left;"));
  client.println(F("}"));
  client.println(F(".footer {"));
  client.println(F("text-align: center;"));
  client.println(F("font-family: 'Comic Sans MS';"));
  client.println(F("}"));
  client.println();
  client.println(F("p {"));
  client.println(F("font-family: 'Comic Sans MS';"));
  client.println(F("font-size: 14px;"));
  client.println(F("}"));
  client.println(F("-->"));
  client.println(F("</style>"));
  client.println(F("</head>"));
  client.println();
  client.println(F("<body>"));
  client.println(F("<hr />"));
  client.println(F("<table width='800' border='0'>"));
  client.println(F("<tr>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(F("<img alt=\"Embedded Image\" src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAH0AAABLCAYAAABdo4iVAAAAD3RFWHRBdXRob3IATG9nYXN0ZXL0WrQKAAAS3ElEQVR42u1dC3RVxbk+J4mKVnwUAuR58jwn7+Rkn31OcvJCwlNtqy1BuXq9ropoL75atQpXCcYEEnkIvhBZVq2r2Nrb672WXnEVq5VWa7VLniK5WusLayuWiiAJJOd+3z7/pOPmhAbIygP2XutfM3vmn39m/m/+f/6ZswMul/M4z4nwRFwuN8nRhPM4z/Fs4aAzQCWqzNHM8Q16nKTLQSG9zHmOT8DjJZ0CetOx8hPLyjeC1oubj3c0c/wDXgSKgH6hW7/zHJ+gJ0h6C6gbtAs0ztnTT4z9/AmxdNKzLCfozt5+nLl0m3tfJ4B3SPq4WhQKeMfyj49zeZyy9C6X62cC9gFQp+Qf1LcAJ6ofhiBLysuXCq38FEnv00CPaMDfLvWloCudIG8Y7t+gk0FPgp4DGZrVX2cDvVvLF4AyJJ/nAD88j2eZmjVfL2V1UtalBXQHJV0vPG/IWd6t7/XOM3yAX6+B+3W6edAezcq7tDQiln6J5JfoQZ4D/jA4lwtQbQIqaZsAuMNm4crdk+c7oK9pi+BCR5tDO0p3xwB9gS1gKwNt0MBmfq8G8oPavs9F8IrISwalOBY/tN26uoz5iQC4X1Ja8i8lvw+UDdqqLYKnQas10NluBGgm6GH70c55BveIdrZu5Zp1/t1m6bTkX0n+f4TvBQ3kd0Bv24K9HNDVIuNM5xw/dCzbK666UnP30zTwFIC/k+ic+YuEd0OMPV5/Nxj9S/4qx9qHyLlc0ocEmAZ5v9x2DlfWzHQ3vYPs/e/EOMbZQf+utH3VuasfItaufQK1S0AeC7pYc+3dNiDXaVvAHttFTYdtEfDn2PkaT9C5uBla1q4Ct2/L0SvW0YzpPFkoXm2/J30MWmNrVwP6jcaz0HHxQ+tcvlqscSXP2FqUvs8G5FTt1m6/Big9xQXaQqDV3yipsvynHEsfQmd0CdQIzMugWZK/H7RKc99Mi4R/FOhTDfTP+XGFXMOq9022E8APHEsfOoCniUV3i5tu1Vx9hbZPs75Aiwc2aaC/K+WrDrM1zHW+rxt84EdIeoUt8n5T8s1S/4FWb2jtH9ICvV9J2Q0a0Jts9/MThec0R/sDHLiJtU0EnStlawlKV/SyRbfQtdp+f0DAq9VkXaAtlPul7Gqt/c2gv0l+j8QPhdo537H4AY7Wfy0A0U3/VYDh3flr2g8tbwvvDAXkQez3jdE2BPAroD9L3RzhXaQFcvywYou8r9H2/EudvX0A93FJvyqg3if7ubLMa4R6buL2u1z5n44YkX4warWTQGN0mQdg7Qfd7nWdOJ6J7J+J5bfLglJHtkclXmBdoj4e5xkYK5+oHbN+pO2594JGi6v/osvtvofvPQLi4lyXTpo0pikUym01jPS2cHikLv/JhoZ47X7+Ielriy2Ya3Vc+wCfySW9zHa1+qUvYAD4XXTNzM8xzVEtgcD1Cw1jXXMg8O6dgcCepkCgA/R5k2m+h/IXWgOBxoWm6RXZY7uin0hfIO+faJ6ELj9XfXTpIDKwoDcICF/I+Vzt4R+/53KdqvgXBQLzAPhHiysqIndXVkaWgZYgf1coFGEZ3++WsmbT/Ay8K4zzz7ci860FBfzWbmJXVG6nnNs/0D60dFz7ALv38QI6fw+/Vbn3LrkmXZmbmwLLfWExwCXAsG7SS6BWWPfld5jmhQtM8xJYeyPoWdC+xbIQsFB2NPr9AWu/j49Xv9b9pxzvPtN+3XNAHyDQ9Z9TI/IZVFgAX8u6u0xzHNz4NoK9KBiMIL++JRisOZxc1BeA79EW04y0oR3S3Qv9fuunWsQFyyD/G7yNkz5HO6APTvR+hlyh0u2OYRS//+STvY1wybDmVwjcQgCOfJPetrGuLoHEgE3lG7W9GRZ/KQD/ojXa9mMGe6xvj35U+XuC3uFy+fUF6DwDC7y6a7/E5Y4a3R1w35aLBmjIN6uInHQ4mQR2lWGcZAFvGNMBfBc9xR2G8azV1+mnjxbXzv7qnOh98IK5H8hlC8/RcW3FxT5Y5/62KFjPWGDCkl1H4IbpKWTxNFMOLb7ZMM5jnzjLv9IdBb3KAX3wQOcvad2dbvdjBLa5vHwZo3AAvw9u2seyhiMHxt0Ar7Bi6tRTAPxbSysruyHXOgYejItT17NFjnsfvGBO/RXLDLr3Ow1jO45g3QsM47+UWz8a+eIdXAsCgblLo4to77/W14+SP4bg/XuKA/ogAt8V/bDRqJs5czRA7+B+ziOZCtqOCvSobHdTMFgKwA/Szc8LBuni3XDxz6pzuvMMEuidCQn8rfys74VC1a3R41mkqbzc+um0sbHxaC3RigFuDodHwsV/yAuc+aZ5I8s64uJmOse1wX4kagfQM2jlCOD23en3ezSLPaYHscGm5eFwBGmb7vqdZxCf13DMIuy8YeMRi3fqcPPZxwi6tZKeRBAIS39jOSwdcUKLA/oQeRokWAPolbxNsy5kTLNWrzvae4CWsrJEeI5PGcwhqJvjgD7ELmrmhkJjYZW7CRDSm44FIPl51Q2PMZ6LyFpIyB/LicB5+vlRbrzJMF7kkQ3W+fKxBHJqsWDxPCDyPmoMBs/QXb/zDDboCiTDuIb7+kLT7AZg047G2uUzKjeOammg3UuinmO1Y+VD08W7bzGMM3nEwrm6G4C13yxfxfQVLAKuFgm8xtM8nyM+6Gzy+/P76zTgPP34KGARvV/cJj+pAvh1jR6P9Yn0bET5vYFmv8iBjKVsb130GMYix8qHAfBw8/ctkV/amk1zw0K/P0c/jqmfVVXApipa4SlaAoFHLMB5/DOMF7hYRK6zlw/VqxoFPKz8kcXqV7JA4BP+rq6+gbM/beFwMurnYIH8X6t8PYP8bxcVF/MfO3D+yZHhALxy49iPb8PZvYNWvyTqqvnr2yvwBI8jfz/2/0eQ/hqA72IAqLwD2j0MCz/N2ceHKfD81g0W/lNY+x6CyitVO1m/pCFgg2t/rlmifgfwYb7HW1ZvGNktweBV3LNh2S/C2jc2wYUD7CfwfhMsvLQH7Oj53nHpw/nipq+XNBH5gMLR2vEE/qERe08k38+u3H0MP+sOv71Us5S+Wo27LnpGHgwlWX33p3XbZB33W4T7RJpsTG8i1k3g6+vrvUepi8PGE7Ko4oYM4LCas0DWH+0bhnFSXXX11H+moNra2popU6Y8jHSaXj4Q4w2HwyNrq6tX19TUNPejDqrPGT/+daT7KioqvirlQ88IlEuqDoe/O3HChM3V1dUTba7KAgLliaDNVVVVP9UmGkcXWVlZ6amtqXm7KhzeEwwGU2uqq5+vq62NQLHJio9EmaSpU6eewn0UsuacO21aBOkCUVqC2hrspCnWcsl1FRU54+vqNldXVa3mO2/P7O61FznWfDDmMWgfQfudsbYpUqM27pycnFPA+xzoJS4YawyMB7BQmUfZo/UTJkSgh71YTD9GXUYvoMc12G751LixUOrQ/lX0sRm63KYI2GyF7rdggT6FfnyqjdJFTVXVE9D5Zuqe70b0G/7eF5so2lUZCi3joNHxDL1cKQnlKRgM67fb9mPWrUGnj2FQSaFQqA0D/AWUGia4GuiHPGh3BSzjANLv871AvjvvyzbChcaFVVlR8XwvbvLw+7lhjEbbjopQaGtftyboaBfbxKoLh0LfwbyvhMJHH6nhaXpcxIWIPrqFIkJWXua7ya4ntHsd+o9MjnqXvncYCARaAWpX0DAaWAYawZQdMEV9WtA0u8xA4A+6gkzTLATNMEpKQuAJM++33XeLy8/GSjwH9fXhYHAy0gBoFgeL9FbFXFZWllhpmrXgnUBeoUpamtVfaWkhAGfZ5aFgsCsYCLxKHsifBBoPBZyqy9Fk1HN8Sk6oqGisNR/T3K5bG60EfNPQ/3TKxFyS2R/0Mhlz/wvoM7O8/HzKQ18TwXcGdVMbDleWlJR8RXTCfqfWyQ8+msW7wZsF45hiyGdduvGgzW3EwDSMa6VfD2XT4MCfhPK3MeYIFxboTBjYJGtugcAbnAvKZlJvLGf7Xhey6rC8rKyNq6m8vHxKrMUBgacZ5eWRcr9/o7KmstLSe9BhBMqPwGIs4qBIUHqbBuT1AcPorBA+uCzKaUf5FdLnPPJhoJXo46MKTZ5FeEf5puLi4rPR7jF6HPaBfARyLR7KYRl5SktLyyw5uox/yNlIhVBpaN/tLysj6BbgaJeCuucoC3z7kK8HfZv9haJtrT6VPM4DcwiD5xq49Ah0yP9Fwo33tziWwsLCNKUrFa+gv+fDlZUEbrLupkVPt1v6KC29xK5/Lij0/ynaf8Y24J1EORyHGhfHSJJxXWfz2IeCDmUtZuOS4uK1yN+DTu5H/t6SoqL7SvEOehggs36TpaDCwiryl5aUvIeyB0vJH6VVyO/kICDHB0WehfeDoD2g1ai/F8pZCXk3IH8FLJV8Fuiof5Ht2C/qb0N9I/q/He+Pc3HhvQ3yzjP8fo7tcY4Hbd6nTPA/gHQ554P8OktOdOw9cpD/ochZTsWhLftqZ99FRUWVnAvrwbchPz/fI3qp8JeWcl6U9Tn4O5B/iH1CsSsJLGRfi3YHwWv9xz/g2cg54z1VeQ9ZDCUAjeUb9f1eYQCZ82V8l4HnBsxvJfst5tzQxtJ3cfFKa/EUFubLuDj3jzkX8D1GrNDPA5xPr8Gx6hCDXwrBVEIEAzuEBPAI3P0m4b8I5V1FBQWNdpmF+flLUNddlJc3qcTrzeSA0O7nh/AVFs6ibNTdYr0XFLwPsgdWVHwW+u5E3f+qstzc3BTK1ct65BYU/LEgP//P9nIvxiJy/pvbVlFhIdu/hnF8E/m9xUVFtM4HU1NTT41x7lbj+5tdLmTdKvOYLXybKVuBrvZg6OUZ8qGPi3X5CgPwzWd9UX7+pZDRTpCV/mWBWuMF3zjbuF5if1lZWWce0Z6e5/XexYZIW/Jyc8/L93q/kZeXd77P5/sa0vNAl+Xn5bF+M/lRPp1KQtrEfZLuh8Q8lLucdUjraTFYGN0+r3ct68aOHdvDh35mw0oo0wI9z+f7AHztkDkl3+fbifIPSSj7XPq6Wp0YMJ4iAMuy9XynPDUXlP0Rbd7Ky8mZZJOzpzja36w6lytB5vNbyFplyfd6/+TxeM7SrNOl+mOsYMnx+XbTe6kyWivHb80D8xFdbqZsn8eToYJKX27uddQvxvYbdZqJgcHt1jhycy+H7qoLfb5vEQfQBRjjTLR9piSqB/7zZ272b+nC632V/dHAVBymnfnjewU9Ozu7rQANsVqmxVocEHR6TnZ2BLRF+GdA6V252dlz7bwoWwRZXQBiEq0LiurOycp6ys6H+n+BjG6kViAH2Tsh901vZqbpzc3dgfI3QNth1e1IW+qif7wYL5aeDx6O55cirmdyKPsTqB2yjNycHMrZruSgjF/DxGFco1HXjbFu49yQ/pyK8+bkbKQX0WSqQPQkyPwQbXaTX18YkH0LFyBkzJb+N0NORPHlZmVdhQUVwXh352Vmeu0nDIUB+G6jHMhr6AWDk9H/AY5ZD9LQ3++pC2CX3qfTi+owKyNjEQVmZmZOV+dSdeZjmp2cnJaZkXEA9LrF7/GcC/5IRkbGeljHNzPT079FYj7D49nAuvT09CpvUtLo7Kws8u1A/XTUX8g+rDQjYw0Hi/drKBPtdmRlZu616tLSajJSU+tQV6PcrT4h8ORBLse7zg5QpsezFXL2HU4O5peI/jtA25RCkX+K40a6JS0tLVlfTNQDdPQu6j5JTEz8EuiY283eqO5mSf8bwdeJ8pswp6dF5t8x/4n2BapjAP75APQA0rupS6QXUVegBqZ4/zeOGTLbddBR9jLme0AD3ZLPOUAHFbFwTxCGe2nJUMxMvVwpOSUlJTUjPT2Slpr6Dt9H+Xwjkf8DOoxAGZxURM+j7iWlZMheE4svK8r3PhcUJwC+qzGhHh5FnqSkPJlIz+UKZBdhomz/O22c1mTTU1KujCUH8gsVD7aZMZwP5OzUgqqE9LS0JygX6S7Umao/Aoy+9oIio0aNGqmDnpaSsoC6Q79WxAyeN1WfHAdkPcPxxgLchsGdVpsYevqSbtPSmvTFovrDnDL0GAJ9/oRzSUpKOsQDWJnk5OSvQ9gKgFtqY7BWE4ME8KxITUr6vmrI/Q/814GWpiQlRQl58FyrFCPtT0L5VXY+0FwuJr0/DHBKSnJyG/q6W9G4ceMSbedd15gxY8ZiUitQ/++286hb5ExOTU5uRf0yJYdAq7HTWlG2HP3Ps803DuObC9krUf89BQi9HcqbUNamzvoqEEPZRA90hz6rpO8LOTbqAXVlGrjxh7lzJ0g51Is15qSkLxF1xnKks/Vzv/R/I+a6Ap71bL0fyDoHdf+hxtsfV8L9eafs7meZ7iEwJ7vcIflza5wESoebeIJttbqlLBa5+8hnV0b8P5Fll9mb9fRFTqz2+ljjY/AnHEZ3cTH6jjvCxZHQB+ptrrE8iPP3dyf68//Gkf597EmqFwAAAABJRU5ErkJggg==\" />"));
  client.println(F("</td>"));
  client.println(F("<td align='center' valign='middle'><h1 class='MainTitle'>Air Pressure Controler <br />"));
  client.println(F("(Dual control Box)</h1></td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println();
  client.println(F("<hr />"));
  client.print(F("<p>Last refresh: "));
  if (RTC.read(tm)) {
    print2digitsEth(tm.Hour, client);
    client.print(F(":"));
    print2digitsEth(tm.Minute, client);
    client.print(F(":"));
    print2digitsEth(tm.Second, client);
    client.print(F(" - "));
    client.print(tmYearToCalendar(tm.Year));
    client.print('/');
    client.print(tm.Month);
    client.print('/');
    client.print(tm.Day);
  } else {
    if (RTC.chipPresent()) {
      client.print(F("The DS1307 is stopped.  Please run the SetTime"));
      client.print(F("example to initialize the time and begin running."));
    } else {
      client.print(F("DS1307 read error!  Please check the circuitry."));
    }
  }

  client.println(F("</p>"));
  client.println(F("<hr />"));
  client.print(F("<h3 class='SubTitle'>Actual Status:"));
  if (errorcode != 0) {
    client.print(F("ERROR"));
  } else if (onoff) {
    client.print(F("RUN"));
  } else {
    client.print(F("STOP"));
  }
  client.println(F("</h3>"));
  client.println(F("<table width='600' border='1'>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' scope='col'>Actual Status</th>"));
  client.println(F("<th scope='col'>A ch</th>"));
  client.println(F("<th scope='col'>B ch</th>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("    <th width='160' align='right' scope='row'>Run/Down status</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (AcRun) {
    client.print(F("RUN"));
  } else {
    client.print(F("DOWN"));
  }
  client.println(F("</td>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (BcRun) {
    client.print(F("RUN"));
  } else {
    client.print(F("DOWN"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Pressure</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(AchAct, 1);
  client.println(F(" bar</td>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(BchAct, 1);
  client.println(F(" bar</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Operation mode</th>"));
  client.println(F("<td align='center' valign='middle'>"));
  if (Acprio) {
    client.println(F("MASTER"));
  } else if (Bcprio) {
    client.println(F("SLAVE"));
  } else {
    client.println(F("COMMON"));
  }
  client.println(F("</td>"));
  client.println(F("<td align='center' valign='middle'>"));
  if (Acprio) {
    client.println(F("SLAVE"));
  } else if (Bcprio) {
    client.println(F("MASTER"));
  } else {
    client.println(F("COMMON"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Pressure Set Point</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (Acprio) {
    client.println(masterSP, 1);
  } else if (Bcprio) {
    client.println(slaveSP, 1);
  } else {
    client.println(masterSP, 1);
  }
  client.println(F(" bar</td>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (Acprio) {
    client.println(slaveSP, 1);
  } else if (Bcprio) {
    client.println(masterSP, 1);
  } else {
    client.println(masterSP, 1);
  }
  client.println(F(" bar</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Hysteresys</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (Acprio) {
    client.println(masterHys, 1);
  } else if (Bcprio) {
    client.println(slaveHys, 1);
  } else {
    client.println(masterHys, 1);
  }
  client.println(F(" bar</td>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (Acprio) {
    client.println(slaveHys, 1);
  } else if (Bcprio) {
    client.println(masterHys, 1);
  } else {
    client.println(masterHys, 1);
  }
  client.println(F(" bar</td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println(F("<hr />"));
  client.println(F("<h3 class='SubTitle'>Settings:</h3>"));
  client.println(F("<table width='600' border='1'>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' scope='col'>Sensor Settings</th>"));
  client.println(F("<th scope='col'>A ch</th>"));
  client.println(F("<th scope='col'>B ch</th>"));
  client.println(F("  </tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>PSI/BAR</th>"));
  client.print(F("<td align='center'>"));
  if (Acpsibar) {
    client.print(F("BAR"));
  } else {
    client.print(F("PSI"));
  }
  client.println(F("</td>"));
  client.print(F("<td align='center'>"));
  if (Bcpsibar) {
    client.print(F("BAR"));
  } else {
    client.print(F("PSI"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Range</th>"));
  client.print(F("<td align='center'>"));
  client.print(Acrange);
  if (Acpsibar) {
    client.print(F(" bar"));
  } else {
    client.print(F(" psi"));
  }
  client.println(F("</td>"));
  client.print(F("<td align='center'>"));
  client.print(Bcrange);
  if (Bcpsibar) {
    client.print(F(" bar"));
  } else {
    client.print(F(" psi"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Range MAX</th>"));
  client.print(F("<td align='center'>"));
  client.print(Acrangemax, 1);
  client.println(F(" V</td>"));
  client.print(F("<td align='center'>"));
  client.print(Bcrangemax, 1);
  client.println(F(" V</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='160' align='right' scope='row'>Range MIN</th>"));
  client.print(F("<td align='center'>"));
  client.print(Acrangemin, 1);
  client.println(F(" V</td>"));
  client.println(F("<td align='center'>"));
  client.print(Bcrangemin, 1);
  client.println(F(" V</td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println(F("<br />"));
  client.println(F("<table width='600' border='1'>"));
  client.println(F("<tr>"));
  client.println(F("<th colspan='2' scope='col'>Common Settings</th>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Update time</th>"));
  client.print(F("<td align='center'>"));
  client.print(updatetime);
  client.println(F(" msec</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Sample rate</th>"));
  client.print(F("<td align='center'>"));
  client.print(minta);
  client.println(F(" s/cyc</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Calibration</th>"));
  client.print(F("<td align='center'>A ch: "));
  client.print(AcCalib);
  client.print(F(" - B ch: "));
  client.print(BcCalib);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println(F("<hr />"));
  client.println(F("<h3 class='SubTitle'>Ethernet Settings:</h3>"));
  client.println(F("<table width='600' border='1'>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>DHCP</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (dhcp) {
    client.print(F("YES"));
  } else {
    client.print(F("NO"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>IP Address</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (dhcp) {
    client.print(Ethernet.localIP());
  } else {
    client.print(ip1);
    client.print(F("."));
    client.print(ip2);
    client.print(F("."));
    client.print(ip3);
    client.print(F("."));
    client.print(ip4);
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Subnet mask</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (dhcp) {
    client.print(Ethernet.subnetMask());
  } else {
    client.print(subnet1);
    client.print(F("."));
    client.print(subnet2);
    client.print(F("."));
    client.print(subnet3);
    client.print(F("."));
    client.print(subnet4);
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Gateway Address</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (dhcp) {
    client.print(Ethernet.gatewayIP());
  } else {
    client.print(gateway1);
    client.print(F("."));
    client.print(gateway2);
    client.print(F("."));
    client.print(gateway3);
    client.print(F("."));
    client.print(gateway4);
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>DNS Server</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (dhcp) {
    client.print(Ethernet.dnsServerIP());
  } else {
    client.print(dns1);
    client.print(F("."));
    client.print(dns2);
    client.print(F("."));
    client.print(dns3);
    client.print(F("."));
    client.print(dns4);
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Web port</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(webport);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>UDP port</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(udpport);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println(F("<hr />"));
  client.println(F("<h3 class='SubTitle'>Error Status:</h3>"));
  client.println(F("<table width='600' border='1'>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Error code</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(errorcode);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Error text</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(ErrString(errorcode));
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Channel Relay check</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (checkrelayfb) {
    client.print(F("ON"));
  } else {
    client.print(F("OFF"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Overcurrent contactor check</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  if (checkampfb) {
    client.print(F("ON"));
  } else {
    client.print(F("OFF"));
  }
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println(F("<hr />"));
  client.println(F("<h3 class='SubTitle'>Version information:</h3>"));
  client.println(F("<table width='600' border='1'>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Hardware version</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(hwver);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Software version</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(swver);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("<tr>"));
  client.println(F("<th width='200' align='right' scope='row'>Last Modify Date</th>"));
  client.print(F("<td align='center' valign='middle'>"));
  client.print(lastupdate);
  client.println(F("</td>"));
  client.println(F("</tr>"));
  client.println(F("</table>"));
  client.println(F("<hr />"));
  client.println(F("<h3 class='SubTitle'>Contact:</h3>"));
  client.println(F("<p>Hődetektív Műszaki és Szolgáltató Bt.<br />"));
  client.println(F("3413 Cserépfalu, Kossuth út 153.<br />"));
  client.println(F("Tel: +36/70/539-4344<br />"));
  client.println(F("Email: hodetektiv@hodetektiv.hu<br />"));
  client.println(F("Web: <a href='http://www.hodetektiv.hu' target='_blank'>www.hodetektiv.hu</a> </p>"));
  client.println(F("<hr />"));
  client.println(F("<h5 class='footer'>©2016 Hődetektív Bt. | Create by OsztO</h5>"));
  client.println(F("</body>"));
  client.println(F("</html>"));
}

void ControlSignal() {
  if (lastcontrol) {
    lastcontrol = LOW;
  } else {
    lastcontrol = HIGH;
  }
  digitalWrite(cled, lastcontrol);
  if (errorcode != 0) {
    digitalWrite(piezo, lastcontrol);
  } else {
    digitalWrite(piezo, LOW);
  }
}

void Estop() {
  if (digitalRead(EmrgStop)) {
    errorcode = 100;
    lastemrgstop = HIGH;
    //Serial.print("EMERGENCY STOP ACTIVE!");
  } else {
    if (lastemrgstop) {
      errorcode = 0;
      automode = HIGH;
      MainContact = HIGH;
      Acerror = LOW;
      Bcerror = LOW;
      lastemrgstop = LOW;
    }
  }
  if (errorcode != 0) {
    automode = LOW;
    Achannel = LOW;
    Bchannel = LOW;
    ATchannel = LOW;
    BTchannel = LOW;
    MainContact = LOW;
    relayupdate();
    // Serial.print("ERROR! - Error code: ");
    //  Serial.println(errorcode);
  }
}

void checkerror() {
  if (checkrelayfb) {
    if (AcRun) {
      if (digitalRead(Acrelayfb)) {
        errorcode = 1;
        Acerror = HIGH;
      }
    } else {
      if (!digitalRead(Acrelayfb)) {
        errorcode = 2;
        Acerror = HIGH;
      }
    }
    if (BcRun) {
      if (digitalRead(Bcrelayfb)) {
        errorcode = 3;
        Bcerror = HIGH;
      }
    } else {
      if (!digitalRead(Bcrelayfb)) {
        errorcode = 4;
        Bcerror = HIGH;
      }
    }
  }
  if (digitalRead(Acpresfb)) {
    errorcode = 5;
    Acerror = HIGH;
  }
  if (digitalRead(Bcpresfb)) {
    errorcode = 6;
    Bcerror = HIGH;
  }

  if (checkampfb) {
    if (digitalRead(Bcampfb)) {
      errorcode = 7;
      Bcerror = HIGH;
    }
    if (digitalRead(Acampfb)) {
      errorcode = 8;
      Acerror = HIGH;
    }
  }
}

String ErrString(int e) {
  switch (e) {
    case 0:
      return "NO ERROR";
      break;
    case 1:
      return "A channel relay not pull.";
      break;
    case 2:
      return "A channel relay not release.";
      break;
    case 3:
      return "B channel relay not pull.";
      break;
    case 4:
      return "B channel relay not release.";
      break;
    case 5:
      return "A channel over pressure.";
      break;
    case 6:
      return "B channel over pressure.";
      break;
    case 7:
      return "A channel over amper.";
      break;
    case 8:
      return "B channel over amper.";
      break;
    case 98:
      return "FATAL ERROR: RTC chip error!";
      break;
    case 99:
      return "RTC Error: Need setup time/date!";
      break;
    case 100:
      return "E-Stop Activate!";
      break;
    default:
      break;
  }
}

void automanipulator() {
  pressuremeter();
  if (automode) {
    MainContact = HIGH;
    if (onoff) {
      if (Acprio && !Bcprio) {
        if (AchAct <= masterSP - masterHys) {
          manipulator(1, HIGH);
        } else if (AchAct >= masterSP) {
          manipulator(1, LOW);
        }
        if (BchAct <= slaveSP - slaveHys) {
          manipulator(2, HIGH);
        } else if (BchAct >= slaveSP) {
          manipulator(2, LOW);
        }
      } else if (Bcprio && !Acprio) {
        if (BchAct <= masterSP - masterHys) {
          manipulator(2, HIGH);
        } else if (BchAct >= masterSP) {
          manipulator(2, LOW);
        }
        if (AchAct <= slaveSP - slaveHys) {
          manipulator(1, HIGH);
        } else if (AchAct >= slaveSP) {
          manipulator(1, LOW);
        }

      } else if (!Bcprio && !Acprio) {
        if (AchAct <= masterSP - masterHys) {
          manipulator(3, HIGH);
        } else if (AchAct >= masterSP) {
          manipulator(3, LOW);
        }
      }
    } else {
      manipulator(3, LOW);
    }
  } else {
    manipulator(3, LOW);
    MainContact = LOW;
  }
}

void buttonact() {
  //Kapcsolók állapota
  if (digitalRead(Acpriopin)) {
    Acprio = LOW;
  } else {
    Acprio = HIGH;
  }
  if (digitalRead(Bcpriopin)) {
    Bcprio = LOW;
  } else {
    Bcprio = HIGH;
  }
  /* if (!Acprio)
    Serial.println("Acprio");
    if (!Bcprio)
    Serial.println("Bcprio");
    if(!onoff)
    Serial.println("ONOFF");
  */
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
    // Serial.println("OnOff");
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
        lcd.print(F("A~ch:    ||"));
        lcd.setCursor(11, 0);
        lcd.print(F("B~ch:     "));
        lcd.setCursor(5, 1);
        lcd.print(F("bar ||"));
        lcd.setCursor(16, 1);
        lcd.print(F("bar"));
        lcd2.setCursor(4, 0);
        lcd2.print(F("bar  "));
        lcd2.setCursor(13, 0);
        lcd2.print(F("bar"));
        lcd.setCursor(0, 2);
        lcd.print(F("         ||         "));
        lcd.setCursor(0, 1);
        lcd.print(F("     "));
        lcd.setCursor(11, 1);
        lcd.print(F("     "));
        lcd2.setCursor(0, 0);
        lcd2.print(F("    "));
        lcd2.setCursor(9, 0);
        lcd2.print(F("    "));
        event = LOW;
      }

      lcd.setCursor(5, 0);
      if (AcRun) {
        lcd.print(F("RUN "));
      } else {
        lcd.print(F("Down"));
      }
      lcd.setCursor(16, 0);
      if (BcRun) {
        lcd.print(F("RUN "));
      } else {
        lcd.print(F("Down"));
      }

      if (AchAct >= 10) {
        lcd.setCursor(1, 1);
        lcd.print(AchAct, 1);
        lcd2.setCursor(0, 0);
        lcd2.print(AchAct, 1);
      } else {
        lcd.setCursor(1, 1);
        lcd.print(F(" "));
        lcd.setCursor(2, 1);
        lcd.print(AchAct, 1);
        lcd2.setCursor(0, 0);
        lcd2.print(F(" "));
        lcd2.setCursor(1, 0);
        lcd2.print(AchAct, 1);
      }
      if (Acprio && !Bcprio) {
        lcd.setCursor(0, 2);
        lcd.print(F(">MASTER<"));
        lcd.setCursor(12, 2);
        lcd.print(F(" >SLAVE< "));
        if (errorcode == 0) {
          lcd2.setCursor(0, 1);
          lcd2.print(F(">MASTER<"));
          lcd2.setCursor(8, 1);
          lcd2.print(F(" >SLAVE< "));
        }
      } else if (Bcprio && !Acprio) {
        lcd.setCursor(0, 2);
        lcd.print(F(">SLAVE< "));
        lcd.setCursor(12, 2);
        lcd.print(F(">MASTER<"));
        if (errorcode == 0) {
          lcd2.setCursor(0, 2);
          lcd2.print(F(">SLAVE< "));
          lcd2.setCursor(8, 2);
          lcd2.print(F(">MASTER<"));
        }
      } else if (!Bcprio && !Acprio) {
        lcd.setCursor(0, 2);
        lcd.print(F(">COMMON<"));
        lcd.setCursor(12, 2);
        lcd.print(F(">COMMON<"));
        if (errorcode == 0) {
          lcd2.setCursor(0, 2);
          lcd2.print(F(">COMMON<"));
          lcd2.setCursor(8, 2);
          lcd2.print(F(">COMMON<"));
        }
      }

      if (BchAct >= 10) {
        lcd.setCursor(12, 1);
        lcd.print(BchAct, 1);
        lcd2.setCursor(9, 0);
        lcd2.print(BchAct, 1);
      } else {
        lcd.setCursor(12, 1);
        lcd.print(F(" "));
        lcd.setCursor(13, 1);
        lcd.print(BchAct, 1);
        lcd2.setCursor(9, 0);
        lcd2.print(F(" "));
        lcd2.setCursor(10, 0);
        lcd2.print(BchAct, 1);
      }
      if (errorcode == 0) {
        if (RTC.read(tm)) {
          if (tm.Second != lastref) {
            lastref = tm.Second;
            lcd.setCursor(0, 3);
            lcd.print(tmYearToCalendar(tm.Year));
            lcd.print(F("."));
            lcd.print(tm.Month);
            lcd.print(F("."));
            lcd.print(tm.Day);
            lcd.print(F(". "));
            print2digitslcd(tm.Hour);
            lcd.print(F(":"));
            print2digitslcd(tm.Minute);
            lcd.print(F(":"));
            print2digitslcd(tm.Second);
          }
        } else {
          lcd.setCursor(0, 3);
          lcd.print(F("      RTC ERROR!   "));
          lcd2.setCursor(0, 1);
          lcd2.print(F("  RTC ERROR!   "));
        }
      } else {
        if (errorcode == 100) {
          lcd.setCursor(0, 3);
          lcd.print(F("   E-STOP ACTIVE!   "));
          lcd2.setCursor(0, 1);
          lcd2.print(F(" E-STOP ACTIVE! "));
        } else {
          lcd.setCursor(0, 3);
          lcd.print(F(" ERROR!  -- Code: "));
          print2digitslcd(errorcode);
          lcd2.setCursor(0, 1);
          lcd2.print(F("ERROR!--Code: "));
          print2digitslcd2(errorcode);

        }
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
          lcd.setCursor(0, 1);
          lcd.print(F("                    "));
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("   MASTER Setting   "));
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
              idszam = masterSP;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 7;
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
          lcd.print(F("MENU-> MASTER       "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   MASTER Setpoint  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" bar"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = masterSP;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 10)
                      idszam = 10;
                    if (idszam < 1)
                      idszam = 1;
                    masterSP = idszam;
                    EEPROM.writeFloat(masterSPAddr, masterSP);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.1;
                    if (idszam > 10)
                      idszam = 10;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 1;
                    idszam = masterHys;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.1;
                    if (idszam > 10)
                      idszam = 10;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 1;
                    idszam = masterHys;
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
              lcd.print(F(" MASTER Hysteresis  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" bar"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = masterHys;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 4)
                      idszam = 4;
                    if (idszam < 1)
                      idszam = 1;
                    masterHys = idszam;
                    EEPROM.writeFloat(masterHysAddr, masterHys);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.1;
                    if (idszam > 4)
                      idszam = 4;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 0;
                    idszam = masterSP;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.1;
                    if (idszam > 4)
                      idszam = 4;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 0;
                    idszam = masterSP;
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
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F("   SLAVE Setting    "));
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
              idszam = slaveSP;
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
          lcd.print(F("MENU-> SLAVE        "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   SLAVE  Setpoint  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" bar"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = slaveSP;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 10)
                      idszam = 10;
                    if (idszam < 1)
                      idszam = 1;
                    slaveSP = idszam;
                    EEPROM.writeFloat(slaveSPAddr, slaveSP);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.1;
                    if (idszam > 10)
                      idszam = 10;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 1;
                    idszam = slaveHys;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.1;
                    if (idszam > 10)
                      idszam = 10;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 1;
                    idszam = slaveHys;
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
              lcd.print(F(" SLAVE  Hysteresis  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 1);
              lcd.print(F(" bar"));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = slaveHys;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 4)
                      idszam = 4;
                    if (idszam < 1)
                      idszam = 1;
                    slaveHys = idszam;
                    EEPROM.writeFloat(slaveHysAddr, slaveHys);
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.1;
                    if (idszam > 4)
                      idszam = 4;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 0;
                    idszam = slaveSP;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.1;
                    if (idszam > 4)
                      idszam = 4;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 0;
                    idszam = slaveSP;
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
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F(" A channel Setting  "));
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
              iddont = Acpsibar;
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
          lcd.print(F("MENU-> A channel    "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F(" A channel PSI/BAR  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (iddont) {
                lcd.print(F("BAR         "));
              } else {
                lcd.print(F("PSI         "));
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    iddont = Acpsibar;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    Acpsibar = iddont;
                    EEPROM.writeFloat(AcpsibarAddr, Acpsibar);
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
                    spos = 3;
                    idszam = Acrangemin;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 1;
                    idszam = Acrange;
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
              lcd.print(F(" A channel Range    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              if (Acpsibar) {
                lcd.print(F(" bar     "));
              } else {
                lcd.print(F(" psi     "));
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = Acrange;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 2000)
                      idszam = 2000;
                    if (idszam < 1)
                      idszam = 1;
                    Acrange = idszam;
                    EEPROM.writeFloat(AcrangeAddr, Acrange);
                    dataupdate();
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
                    if (idszam > 2000)
                      idszam = 2000;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 0;
                    iddont = Acpsibar;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 2000)
                      idszam = 2000;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 2;
                    idszam = Acrangemax;
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
              lcd.print(F(" A channel Range MAX"));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 2);
              lcd.print(F(" volt     "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = Acrangemax;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 5)
                      idszam = 5;
                    if (idszam <= Acrangemin)
                      idszam = Acrangemin + 0.01;
                    Acrangemax = idszam;
                    EEPROM.writeFloat(AcrangemaxAddr, Acrangemax);
                    dataupdate();
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.01;
                    if (idszam > 5)
                      idszam = 5;
                    if (idszam <= Acrangemin)
                      idszam = Acrangemin + 0.01;
                  } else {
                    spos = 1;
                    idszam = Acrange;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.01;
                    if (idszam > 5)
                      idszam = 5;
                    if (idszam <= Acrangemin)
                      idszam = Acrangemin + 0.01;
                  } else {
                    spos = 3;
                    idszam = Acrangemin;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;


            case 3:
              lcd.setCursor(0, 2);
              lcd.print(F(" A channel Range MIN"));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 2);
              lcd.print(F(" volt     "));

              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = Acrangemin;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= Acrangemax)
                      idszam = Acrangemax - 0.01;
                    if (idszam < 0)
                      idszam = 0;
                    Acrangemin = idszam;
                    EEPROM.writeFloat(AcrangeminAddr, Acrangemin);
                    dataupdate();
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.01;
                    if (idszam >= Acrangemax)
                      idszam = Acrangemax - 0.01;
                    if (idszam < 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = Acrangemax;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.01;
                    if (idszam >= Acrangemax)
                      idszam = Acrangemax - 0.01;
                    if (idszam < 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    iddont = Acpsibar;
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
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F(" B channel Setting  "));
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
              iddont = Bcpsibar;
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
          lcd.print(F("MENU-> B channel    "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F(" B channel PSI/BAR  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (iddont) {
                lcd.print(F("BAR         "));
              } else {
                lcd.print(F("PSI         "));
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    iddont = Bcpsibar;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    Bcpsibar = iddont;
                    EEPROM.writeFloat(BcpsibarAddr, Bcpsibar);
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
                    spos = 3;
                    idszam = Bcrangemin;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 1;
                    idszam = Bcrange;
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
              lcd.print(F(" B channel Range    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              if (Bcpsibar) {
                lcd.print(F(" bar     "));
              } else {
                lcd.print(F(" psi     "));
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = Bcrange;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 2000)
                      idszam = 2000;
                    if (idszam < 1)
                      idszam = 1;
                    Bcrange = idszam;
                    EEPROM.writeFloat(BcrangeAddr, Bcrange);
                    dataupdate();
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
                    if (idszam > 2000)
                      idszam = 2000;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 0;
                    iddont = Bcpsibar;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 2000)
                      idszam = 2000;
                    if (idszam < 1)
                      idszam = 1;
                  } else {
                    spos = 2;
                    idszam = Bcrangemax;
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
              lcd.print(F(" B channel Range MAX"));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 2);
              lcd.print(F(" volt      "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = Bcrangemax;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 5)
                      idszam = 5;
                    if (idszam <= Bcrangemin)
                      idszam = Bcrangemin + 0.01;
                    Bcrangemax = idszam;
                    EEPROM.writeFloat(BcrangemaxAddr, Bcrangemax);
                    dataupdate();
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.01;
                    if (idszam > 5)
                      idszam = 5;
                    if (idszam <= Bcrangemin)
                      idszam = Bcrangemin + 0.01;
                  } else {
                    spos = 1;
                    idszam = Bcrange;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.01;
                    if (idszam > 5)
                      idszam = 5;
                    if (idszam <= Bcrangemin)
                      idszam = Bcrangemin + 0.01;
                  } else {
                    spos = 3;
                    idszam = Bcrangemin;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;


            case 3:
              lcd.setCursor(0, 2);
              lcd.print(F(" B channel Range MIN"));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 2);
              lcd.print(F(" volt     "));

              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = Bcrangemin;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam >= Bcrangemax)
                      idszam = Bcrangemax - 0.01;
                    if (idszam < 0)
                      idszam = 0;
                    Bcrangemin = idszam;
                    EEPROM.writeFloat(BcrangeminAddr, Bcrangemin);
                    dataupdate();
                    medit = LOW;
                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    idszam += 0.01;
                    if (idszam >= Bcrangemax)
                      idszam = Bcrangemax - 0.01;
                    if (idszam < 0)
                      idszam = 0;
                  } else {
                    spos = 2;
                    idszam = Bcrangemax;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 0.01;
                    if (idszam >= Bcrangemax)
                      idszam = Bcrangemax - 0.01;
                    if (idszam < 0)
                      idszam = 0;
                  } else {
                    spos = 0;
                    iddont = Bcpsibar;
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
              mpos = 5;
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
                    if (idszam > 1500)
                      idszam = 1500;
                    if (idszam < 100)
                      idszam = 100;
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
                    if (idszam > 1500)
                      idszam = 1500;
                    if (idszam < 100)
                      idszam = 100;
                  } else {
                    spos = 1;
                    idszam = minta;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 1500)
                      idszam = 1500;
                    if (idszam < 100)
                      idszam = 100;
                  } else {
                    spos = 1;
                    idszam = minta;
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
              lcd.print(F(" Samlpes Settings   "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              lcd.print(F(" s/cyc          "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = minta;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 100)
                      idszam = 100;
                    if (idszam < 20)
                      idszam = 20;
                    minta = idszam;
                    EEPROM.writeFloat(mintaAddr, minta);
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
                    if (idszam > 100)
                      idszam = 100;
                    if (idszam < 20)
                      idszam = 20;
                  } else {
                    spos = 0;
                    idszam = updatetime;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 100)
                      idszam = 100;
                    if (idszam < 20)
                      idszam = 20;
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
              /*
                        case 2:
                        lcd.setCursor(0,2);
                      lcd.print(F("   Calibration      "));
                      lcd.setCursor(0,3);
                    if (!onoff){
                    if (medit) {
                      lcd.print(F("* "));
                      } else {
                        lcd.print(F("  "));
                      }
                    }
                      if (onoff) {
                        lcd.print(F("Please Turn OFF Auto"));
                      } else {
                      if (iddont){
                        lcd.print(F("Start - Press OK  "));
                      } else {
                        lcd.print(F("Press->OK->UP->OK "));
                      }
                      }
                        switch (eventin){
                          case 4:
                        eventin = 0;
                        if (medit){
                         medit = LOW;
                         iddont = LOW;
                        } else {
                        submenu = LOW;
                        }
                        event = HIGH;
                        break;
                        case 1:
                        eventin = 0;
                       if (medit){
                        if (iddont && !onoff) {
                        lcd.setCursor(0,3);
                        lcd.print(F("Calibration in 5 sec"));
                        delay(5000);
                        calibration();
                        lcd.setCursor(0,3);
                        lcd.print(F("Calibration Complete"));
                        delay(3000);
                        iddont = LOW;
                        }
                          medit = LOW;
                        } else {
                          if (!onoff)
                        medit = HIGH;
                        }
                           event = HIGH;
                           break;
                      case 2:
                        eventin = 0;
                       if (medit){
                         iddont = HIGH;
                        } else {
                            spos = 1;
                            idszam = minta;
                        }
                           event = HIGH;
                           break;
                      case 3:
                        eventin = 0;
                       if (medit){
                         iddont = LOW;
                        } else {
                            spos = 3;
                            idszam = updatetime;
                        }
                           event = HIGH;
                           break;
                           default:
                        event = LOW;
                        break;
                        }
                        break;
              */
              /*
                      case 3:
                        lcd.setCursor(0,2);
                      lcd.print(F("   Calibration Reset"));
                      lcd.setCursor(0,3);
                    if (!onoff){
                    if (medit) {
                      lcd.print(F("* "));
                      } else {
                        lcd.print(F("  "));
                      }
                    }
                      if (onoff) {
                        lcd.print(F("Please Turn OFF Auto"));
                      } else {
                      if (iddont){
                        lcd.print(F("Start - Press OK  "));
                      } else {
                        lcd.print(F("Press->OK->UP->OK "));
                      }
                      }
                        switch (eventin){
                          case 4:
                        eventin = 0;
                        if (medit){
                         medit = LOW;
                         iddont = LOW;
                        } else {
                        submenu = LOW;
                        }
                        event = HIGH;
                        break;
                        case 1:
                        eventin = 0;
                       if (medit){
                        if (iddont && !onoff) {
                        lcd.setCursor(0,3);
                        lcd.print(F("Calibration Reset   "));
                        delay(3000);
                        calibrationReset();
                        lcd.setCursor(0,3);
                        lcd.print(F("  Reset Complete    "));
                        delay(3000);
                        iddont = LOW;
                        }
                          medit = LOW;
                        } else {
                          if (!onoff)
                        medit = HIGH;
                        }
                           event = HIGH;
                           break;
                      case 2:
                        eventin = 0;
                       if (medit){
                         iddont = HIGH;
                        } else {
                            spos = 2;
                            idszam = minta;
                        }
                           event = HIGH;
                           break;
                      case 3:
                        eventin = 0;
                       if (medit){
                         iddont = LOW;
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
              */

          }
        }
        break;

      case 5:
        if (!submenu) {
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F(" TIME/DATE Setting  "));
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
              setpos = 0;
              submenu = HIGH;
              if (RTC.read(tm)) {
                setYear = tmYearToCalendar(tm.Year);
                setMonth = tm.Month;
                setDay = tm.Day;
              }
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
          lcd.print(F("MENU-> DATE/TIME    "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   Date Setting     "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(setYear);
              lcd.print(F("."));
              lcd.print(setMonth);
              lcd.print(F("."));
              lcd.print(setDay);
              lcd.print(F(".      "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    setpos = 0;
                    medit = LOW;
                    if (RTC.read(tm)) {
                      setYear = tmYearToCalendar(tm.Year);
                      setMonth = tm.Month;
                      setDay = tm.Day;
                    }
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 3:
                        if (setYear > 9999) {
                          setYear = 9999;
                        }
                        if (setYear < 1000) {
                          setYear = 1000;
                        }
                        if (setMonth > 12) {
                          setMonth = 12;
                        }
                        if (setMonth < 1) {
                          setMonth = 1;
                        }
                        switch (setMonth) {
                          case 4:
                            if (setDay > 30) {
                              setDay = 30;
                            }
                            break;
                          case 6:
                            if (setDay > 30) {
                              setDay = 30;
                            }
                            break;
                          case 9:
                            if (setDay > 30) {
                              setDay = 30;
                            }
                            break;
                          case 11:
                            if (setDay > 30) {
                              setDay = 30;
                            }
                            break;
                          case 2:
                            if (setDay > 29) {
                              setDay = 29;
                            }
                            break;
                          default:
                            if (setDay > 31) {
                              setDay = 31;
                            }
                            break;
                        }
                        if (setDay < 1) {
                          setDay = 1;
                        }

                        //Dátum mentése.

                        tm.Year = CalendarYrToTm(setYear);
                        tm.Month = setMonth;
                        tm.Day = setDay;
                        if (RTC.write(tm)) {
                          lcd.setCursor(0, 3);
                          lcd.print(F("   Save Completed   "));
                        } else {
                          lcd.setCursor(0, 3);
                          lcd.print(F("   Save Failed!     "));
                        }
                        delay(2000);
                        medit = LOW;
                        setpos = 0;
                        break;
                      default:
                        setpos += 1;
                        break;
                    }

                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        setYear += 1;
                        break;
                      case 1:
                        setMonth += 1;
                        break;
                      case 2:
                        setDay += 1;
                        break;
                    }
                    if (setYear > 9999) {
                      setYear = 9999;
                    }
                    if (setYear < 1000) {
                      setYear = 1000;
                    }
                    if (setMonth > 12) {
                      setMonth = 12;
                    }
                    if (setMonth < 1) {
                      setMonth = 1;
                    }
                    switch (setMonth) {
                      case 4:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 6:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 9:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 11:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 2:
                        if (setDay > 29) {
                          setDay = 29;
                        }
                        break;
                      default:
                        if (setDay > 31) {
                          setDay = 31;
                        }
                        break;
                    }
                    if (setDay < 1) {
                      setDay = 1;
                    }


                  } else {
                    spos = 1;
                    setpos = 0;
                    if (RTC.read(tm)) {
                      setHour = tm.Hour;
                      setMin = tm.Minute;
                      setSec = tm.Second;
                    }
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        setYear -= 1;
                        break;
                      case 1:
                        setMonth -= 1;
                        break;
                      case 2:
                        setDay -= 1;
                        break;
                    }
                    if (setYear > 9999) {
                      setYear = 9999;
                    }
                    if (setYear < 1000) {
                      setYear = 1000;
                    }
                    if (setMonth > 12) {
                      setMonth = 12;
                    }
                    if (setMonth < 1) {
                      setMonth = 1;
                    }
                    switch (setMonth) {
                      case 4:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 6:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 9:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 11:
                        if (setDay > 30) {
                          setDay = 30;
                        }
                        break;
                      case 2:
                        if (setDay > 29) {
                          setDay = 29;
                        }
                        break;
                      default:
                        if (setDay > 31) {
                          setDay = 31;
                        }
                        break;
                    }
                    if (setDay < 1) {
                      setDay = 1;
                    }

                  } else {
                    spos = 1;
                    setpos = 0;
                    if (RTC.read(tm)) {
                      setHour = tm.Hour;
                      setMin = tm.Minute;
                      setSec = tm.Second;
                    }
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
              lcd.print(F("   Time Setting     "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              print2digitslcd(setHour);
              lcd.print(F(":"));
              print2digitslcd(setMin);
              lcd.print(F(":"));
              print2digitslcd(setSec);
              lcd.print(F("         "));
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    setpos = 0;
                    medit = LOW;
                    if (RTC.read(tm)) {
                      setHour = tm.Hour;
                      setMin = tm.Minute;
                      setSec = tm.Second;
                    }
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 3:
                        if (setHour > 23) {
                          setHour = 23;
                        }
                        if (setHour < 0) {
                          setHour = 0;
                        }
                        if (setMin > 59) {
                          setMin = 59;
                        }
                        if (setMin < 0) {
                          setMin = 0;
                        }
                        if (setSec > 59) {
                          setSec = 59;
                        }
                        if (setSec < 0) {
                          setSec = 0;
                        }

                        //Dátum mentése.

                        tm.Hour = setHour;
                        tm.Minute = setMin;
                        tm.Second = setSec;
                        if (RTC.write(tm)) {
                          lcd.setCursor(0, 3);
                          lcd.print(F("   Save Completed   "));
                        } else {
                          lcd.setCursor(0, 3);
                          lcd.print(F("   Save Failed!     "));
                        }
                        delay(2000);
                        medit = LOW;
                        setpos = 0;
                        break;
                      default:
                        setpos += 1;
                        break;
                    }

                  } else {
                    medit = HIGH;
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        setHour += 1;
                        break;
                      case 1:
                        setMin += 1;
                        break;
                      case 2:
                        setSec += 1;
                        break;
                    }
                    if (setHour > 23) {
                      setHour = 23;
                    }
                    if (setHour < 0) {
                      setHour = 0;
                    }
                    if (setMin > 59) {
                      setMin = 59;
                    }
                    if (setMin < 0) {
                      setMin = 0;
                    }
                    if (setSec > 59) {
                      setSec = 59;
                    }
                    if (setSec < 0) {
                      setSec = 0;
                    }


                  } else {
                    spos = 0;
                    setpos = 0;
                    if (RTC.read(tm)) {
                      setYear = tmYearToCalendar(tm.Year);
                      setMonth = tm.Month;
                      setDay = tm.Day;
                    }
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        setHour -= 1;
                        break;
                      case 1:
                        setMin -= 1;
                        break;
                      case 2:
                        setSec -= 1;
                        break;
                    }
                    if (setHour > 23) {
                      setHour = 23;
                    }
                    if (setHour < 0) {
                      setHour = 0;
                    }
                    if (setMin > 59) {
                      setMin = 59;
                    }
                    if (setMin < 0) {
                      setMin = 0;
                    }
                    if (setSec > 59) {
                      setSec = 59;
                    }
                    if (setSec < 0) {
                      setSec = 0;
                    }

                  } else {
                    spos = 0;
                    setpos = 0;
                    if (RTC.read(tm)) {
                      setYear = tmYearToCalendar(tm.Year);
                      setMonth = tm.Month;
                      setDay = tm.Day;
                    }
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
              iddont = checkrelayfb;
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
          lcd.print(F("MENU-> ERROR        "));
          switch (spos) {
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F(" Contactor Setting  "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (iddont) {
                lcd.print(F("ON                "));
              } else {
                lcd.print(F("OFF               "));
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    iddont = checkrelayfb;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (iddont) {
                      EEPROM.writeFloat(checkrelayfbAddr, HIGH);
                    } else {
                      EEPROM.writeFloat(checkrelayfbAddr, LOW);
                    }
                    checkrelayfb = iddont;
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
                    spos = 2;
                    iddont = LOW;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 1;
                    iddont = checkampfb;
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
              lcd.print(F("Overcurrent Settings"));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (iddont) {
                lcd.print(F("ON                "));
              } else {
                lcd.print(F("OFF               "));
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    iddont = checkampfb;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (iddont) {
                      EEPROM.writeFloat(checkampfbAddr, HIGH);
                    } else {
                      EEPROM.writeFloat(checkampfbAddr, LOW);
                    }
                    checkampfb = iddont;
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
                    iddont = checkrelayfb;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 2;
                    iddont = LOW;
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
                      automode = HIGH;
                      MainContact = HIGH;
                      Acerror = LOW;
                      Bcerror = LOW;
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
                    spos = 1;
                    iddont = checkampfb;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 0;
                    iddont = checkrelayfb;
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
          lcd.setCursor(0, 1);
          lcd.print(F("MENU->              "));
          lcd.setCursor(0, 2);
          lcd.print(F(" Ethernet Setting   "));
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
              setpos = 0;
              submenu = HIGH;
              iddont = dhcp;
              event = HIGH;
              break;
            case 2:
              eventin = 0;
              mpos = 6;
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
          lcd.print(F("MENU-> Ethernet     "));
          switch (spos) {
            //Almenübe lépés
            case 0:
              lcd.setCursor(0, 2);
              lcd.print(F("   DHCP Setting     "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (iddont) {
                lcd.print(F("ON "));
              } else {
                lcd.print(F("OFF"));
              }

              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    iddont = dhcp;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    dhcp = iddont;
                    EEPROM.writeFloat(dhcpAddr, iddont);
                    if (!dhcp) {
                      Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                      server.begin();
                    } else {
                      Ethernet.begin(mac);
                      server.begin();
                    }
                    lcd.setCursor(0, 3);
                    lcd.print(F("   Save Completed   "));
                    delay(2000);
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
                    spos = 6;
                    idszam = udpport;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    iddont = LOW;
                  } else {
                    spos = 1;
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
              lcd.print(F("   IPv4 Setting     "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (dhcp) {
                lcd.print(Ethernet.localIP());
              } else {
                lcd.print(ip1);
                lcd.print(F("."));
                lcd.print(ip2);
                lcd.print(F("."));
                lcd.print(ip3);
                lcd.print(F("."));
                lcd.print(ip4);
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    setpos = 0;
                    medit = LOW;
                    ip1 = EEPROM.readFloat(ip1Addr);
                    ip2 = EEPROM.readFloat(ip2Addr);
                    ip3 = EEPROM.readFloat(ip3Addr);
                    ip4 = EEPROM.readFloat(ip4Addr);
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 3:
                        if (ip1 > 255) {
                          ip1 = 255;
                        }
                        if (ip1 < 0) {
                          ip1 = 0;
                        }
                        if (ip2 > 255) {
                          ip2 = 255;
                        }
                        if (ip2 < 0) {
                          ip2 = 0;
                        }
                        if (ip3 > 255) {
                          ip3 = 255;
                        }
                        if (ip3 < 0) {
                          ip3 = 0;
                        }
                        if (ip4 > 255) {
                          ip4 = 255;
                        }
                        if (ip4 < 0) {
                          ip4 = 0;
                        }
                        //Dátum mentése.

                        EEPROM.writeFloat(ip1Addr, ip1);
                        EEPROM.writeFloat(ip2Addr, ip2);
                        EEPROM.writeFloat(ip3Addr, ip3);
                        EEPROM.writeFloat(ip4Addr, ip4);
                        if (!dhcp) {
                          Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                          server.begin();
                        }
                        lcd.setCursor(0, 3);
                        lcd.print(F("   Save Completed   "));
                        delay(2000);
                        medit = LOW;
                        setpos = 0;
                        break;
                      default:
                        setpos += 1;
                        break;
                    }

                  } else {
                    if (!dhcp) {
                      medit = HIGH;
                    }
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        ip1 += 1;
                        break;
                      case 1:
                        ip2 += 1;
                        break;
                      case 2:
                        ip3 += 1;
                        break;
                      case 3:
                        ip4 += 1;
                        break;
                    }
                    if (ip1 > 255) {
                      ip1 = 255;
                    }
                    if (ip1 < 0) {
                      ip1 = 0;
                    }
                    if (ip2 > 255) {
                      ip2 = 255;
                    }
                    if (ip2 < 0) {
                      ip2 = 0;
                    }
                    if (ip3 > 255) {
                      ip3 = 255;
                    }
                    if (ip3 < 0) {
                      ip3 = 0;
                    }
                    if (ip4 > 255) {
                      ip4 = 255;
                    }
                    if (ip4 < 0) {
                      ip4 = 0;
                    }
                  } else {
                    spos = 0;
                    iddont = dhcp;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        ip1 -= 1;
                        break;
                      case 1:
                        ip2 -= 1;
                        break;
                      case 2:
                        ip3 -= 1;
                        break;
                      case 3:
                        ip4 -= 1;
                        break;
                    }
                    if (ip1 > 255) {
                      ip1 = 255;
                    }
                    if (ip1 < 0) {
                      ip1 = 0;
                    }
                    if (ip2 > 255) {
                      ip2 = 255;
                    }
                    if (ip2 < 0) {
                      ip2 = 0;
                    }
                    if (ip3 > 255) {
                      ip3 = 255;
                    }
                    if (ip3 < 0) {
                      ip3 = 0;
                    }
                    if (ip4 > 255) {
                      ip4 = 255;
                    }
                    if (ip4 < 0) {
                      ip4 = 0;
                    }

                  } else {
                    spos = 2;
                    setpos = 0;
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
              lcd.print(F("  Subnet Setting    "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (dhcp) {
                lcd.print(Ethernet.subnetMask());
              } else {
                lcd.print(subnet1);
                lcd.print(F("."));
                lcd.print(subnet2);
                lcd.print(F("."));
                lcd.print(subnet3);
                lcd.print(F("."));
                lcd.print(subnet4);
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    setpos = 0;
                    medit = LOW;
                    subnet1 = EEPROM.readFloat(subnet1Addr);
                    subnet2 = EEPROM.readFloat(subnet2Addr);
                    subnet3 = EEPROM.readFloat(subnet3Addr);
                    subnet4 = EEPROM.readFloat(subnet4Addr);
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 3:
                        if (subnet1 > 255) {
                          subnet1 = 255;
                        }
                        if (subnet1 < 0) {
                          subnet1 = 0;
                        }
                        if (subnet2 > 255) {
                          subnet2 = 255;
                        }
                        if (subnet2 < 0) {
                          subnet2 = 0;
                        }
                        if (subnet3 > 255) {
                          subnet3 = 255;
                        }
                        if (subnet3 < 0) {
                          subnet3 = 0;
                        }
                        if (subnet4 > 255) {
                          subnet4 = 255;
                        }
                        if (subnet4 < 0) {
                          subnet4 = 0;
                        }


                        EEPROM.writeFloat(subnet1Addr, subnet1);
                        EEPROM.writeFloat(subnet2Addr, subnet2);
                        EEPROM.writeFloat(subnet3Addr, subnet3);
                        EEPROM.writeFloat(subnet4Addr, subnet4);
                        if (!dhcp) {
                          Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                          server.begin();
                        }
                        lcd.setCursor(0, 3);
                        lcd.print(F("   Save Completed   "));
                        delay(2000);
                        medit = LOW;
                        setpos = 0;
                        break;
                      default:
                        setpos += 1;
                        break;
                    }

                  } else {
                    if (!dhcp) {
                      medit = HIGH;
                    }
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        subnet1 += 1;
                        break;
                      case 1:
                        subnet2 += 1;
                        break;
                      case 2:
                        subnet3 += 1;
                        break;
                      case 3:
                        subnet4 += 1;
                        break;
                    }
                    if (subnet1 > 255) {
                      subnet1 = 255;
                    }
                    if (subnet1 < 0) {
                      subnet1 = 0;
                    }
                    if (subnet2 > 255) {
                      subnet2 = 255;
                    }
                    if (subnet2 < 0) {
                      subnet2 = 0;
                    }
                    if (subnet3 > 255) {
                      subnet3 = 255;
                    }
                    if (subnet3 < 0) {
                      subnet3 = 0;
                    }
                    if (subnet4 > 255) {
                      subnet4 = 255;
                    }
                    if (subnet4 < 0) {
                      subnet4 = 0;
                    }
                  } else {
                    spos = 1;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        subnet1 -= 1;
                        break;
                      case 1:
                        subnet2 -= 1;
                        break;
                      case 2:
                        subnet3 -= 1;
                        break;
                      case 3:
                        subnet4 -= 1;
                        break;
                    }
                    if (subnet1 > 255) {
                      subnet1 = 255;
                    }
                    if (subnet1 < 0) {
                      subnet1 = 0;
                    }
                    if (subnet2 > 255) {
                      subnet2 = 255;
                    }
                    if (subnet2 < 0) {
                      subnet2 = 0;
                    }
                    if (subnet3 > 255) {
                      subnet3 = 255;
                    }
                    if (subnet3 < 0) {
                      subnet3 = 0;
                    }
                    if (subnet4 > 255) {
                      subnet4 = 255;
                    }
                    if (subnet4 < 0) {
                      subnet4 = 0;
                    }

                  } else {
                    spos = 3;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;



            case 3:
              lcd.setCursor(0, 2);
              lcd.print(F("  Gateway Setting   "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (dhcp) {
                lcd.print(Ethernet.gatewayIP());
              } else {
                lcd.print(gateway1);
                lcd.print(F("."));
                lcd.print(gateway2);
                lcd.print(F("."));
                lcd.print(gateway3);
                lcd.print(F("."));
                lcd.print(gateway4);
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    setpos = 0;
                    medit = LOW;
                    gateway1 = EEPROM.readFloat(gateway1Addr);
                    gateway2 = EEPROM.readFloat(gateway2Addr);
                    gateway3 = EEPROM.readFloat(gateway3Addr);
                    gateway4 = EEPROM.readFloat(gateway4Addr);
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 3:
                        if (gateway1 > 255) {
                          gateway1 = 255;
                        }
                        if (gateway1 < 0) {
                          gateway1 = 0;
                        }
                        if (gateway2 > 255) {
                          gateway2 = 255;
                        }
                        if (gateway2 < 0) {
                          gateway2 = 0;
                        }
                        if (gateway3 > 255) {
                          gateway3 = 255;
                        }
                        if (gateway3 < 0) {
                          gateway3 = 0;
                        }
                        if (gateway4 > 255) {
                          gateway4 = 255;
                        }
                        if (gateway4 < 0) {
                          gateway4 = 0;
                        }


                        EEPROM.writeFloat(gateway1Addr, gateway1);
                        EEPROM.writeFloat(gateway2Addr, gateway2);
                        EEPROM.writeFloat(gateway3Addr, gateway3);
                        EEPROM.writeFloat(gateway4Addr, gateway4);
                        if (!dhcp) {
                          Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                          server.begin();
                        }
                        lcd.setCursor(0, 3);
                        lcd.print(F("   Save Completed   "));
                        delay(2000);
                        medit = LOW;
                        setpos = 0;
                        break;
                      default:
                        setpos += 1;
                        break;
                    }

                  } else {
                    if (!dhcp) {
                      medit = HIGH;
                    }
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        gateway1 += 1;
                        break;
                      case 1:
                        gateway2 += 1;
                        break;
                      case 2:
                        gateway3 += 1;
                        break;
                      case 3:
                        gateway4 += 1;
                        break;
                    }
                    if (gateway1 > 255) {
                      gateway1 = 255;
                    }
                    if (gateway1 < 0) {
                      gateway1 = 0;
                    }
                    if (gateway2 > 255) {
                      gateway2 = 255;
                    }
                    if (gateway2 < 0) {
                      gateway2 = 0;
                    }
                    if (gateway3 > 255) {
                      gateway3 = 255;
                    }
                    if (gateway3 < 0) {
                      gateway3 = 0;
                    }
                    if (gateway4 > 255) {
                      gateway4 = 255;
                    }
                    if (gateway4 < 0) {
                      gateway4 = 0;
                    }
                  } else {
                    spos = 2;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        gateway1 -= 1;
                        break;
                      case 1:
                        gateway2 -= 1;
                        break;
                      case 2:
                        gateway3 -= 1;
                        break;
                      case 3:
                        gateway4 -= 1;
                        break;
                    }
                    if (gateway1 > 255) {
                      gateway1 = 255;
                    }
                    if (gateway1 < 0) {
                      gateway1 = 0;
                    }
                    if (gateway2 > 255) {
                      gateway2 = 255;
                    }
                    if (gateway2 < 0) {
                      gateway2 = 0;
                    }
                    if (gateway3 > 255) {
                      gateway3 = 255;
                    }
                    if (gateway3 < 0) {
                      gateway3 = 0;
                    }
                    if (gateway4 > 255) {
                      gateway4 = 255;
                    }
                    if (gateway4 < 0) {
                      gateway4 = 0;
                    }

                  } else {
                    spos = 4;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;



            case 4:
              lcd.setCursor(0, 2);
              lcd.print(F("    DNS Setting     "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              if (dhcp) {
                lcd.print(Ethernet.dnsServerIP());
              } else {
                lcd.print(dns1);
                lcd.print(F("."));
                lcd.print(dns2);
                lcd.print(F("."));
                lcd.print(dns3);
                lcd.print(F("."));
                lcd.print(dns4);
              }
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    setpos = 0;
                    medit = LOW;
                    dns1 = EEPROM.readFloat(dns1Addr);
                    dns2 = EEPROM.readFloat(dns2Addr);
                    dns3 = EEPROM.readFloat(dns3Addr);
                    dns4 = EEPROM.readFloat(dns4Addr);
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 3:
                        if (dns1 > 255) {
                          dns1 = 255;
                        }
                        if (dns1 < 0) {
                          dns1 = 0;
                        }
                        if (dns2 > 255) {
                          dns2 = 255;
                        }
                        if (dns2 < 0) {
                          dns2 = 0;
                        }
                        if (dns3 > 255) {
                          dns3 = 255;
                        }
                        if (dns3 < 0) {
                          dns3 = 0;
                        }
                        if (dns4 > 255) {
                          dns4 = 255;
                        }
                        if (dns4 < 0) {
                          dns4 = 0;
                        }


                        EEPROM.writeFloat(dns1Addr, dns1);
                        EEPROM.writeFloat(dns2Addr, dns2);
                        EEPROM.writeFloat(dns3Addr, dns3);
                        EEPROM.writeFloat(dns4Addr, dns4);
                        if (!dhcp) {
                          Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                          server.begin();
                        }
                        lcd.setCursor(0, 3);
                        lcd.print(F("   Save Completed   "));
                        delay(2000);
                        medit = LOW;
                        setpos = 0;
                        break;
                      default:
                        setpos += 1;
                        break;
                    }

                  } else {
                    if (!dhcp) {
                      medit = HIGH;
                    }
                  }
                  event = HIGH;
                  break;
                case 2:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        dns1 += 1;
                        break;
                      case 1:
                        dns2 += 1;
                        break;
                      case 2:
                        dns3 += 1;
                        break;
                      case 3:
                        dns4 += 1;
                        break;
                    }
                    if (dns1 > 255) {
                      dns1 = 255;
                    }
                    if (dns1 < 0) {
                      dns1 = 0;
                    }
                    if (dns2 > 255) {
                      dns2 = 255;
                    }
                    if (dns2 < 0) {
                      dns2 = 0;
                    }
                    if (dns3 > 255) {
                      dns3 = 255;
                    }
                    if (dns3 < 0) {
                      dns3 = 0;
                    }
                    if (dns4 > 255) {
                      dns4 = 255;
                    }
                    if (dns4 < 0) {
                      dns4 = 0;
                    }
                  } else {
                    spos = 3;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 0;
                  if (medit) {
                    switch (setpos) {
                      case 0:
                        dns1 -= 1;
                        break;
                      case 1:
                        dns2 -= 1;
                        break;
                      case 2:
                        dns3 -= 1;
                        break;
                      case 3:
                        dns4 -= 1;
                        break;
                    }
                    if (dns1 > 255) {
                      dns1 = 255;
                    }
                    if (dns1 < 0) {
                      dns1 = 0;
                    }
                    if (dns2 > 255) {
                      dns2 = 255;
                    }
                    if (dns2 < 0) {
                      dns2 = 0;
                    }
                    if (dns3 > 255) {
                      dns3 = 255;
                    }
                    if (dns3 < 0) {
                      dns3 = 0;
                    }
                    if (dns4 > 255) {
                      dns4 = 255;
                    }
                    if (dns4 < 0) {
                      dns4 = 0;
                    }

                  } else {
                    spos = 5;
                    idszam = webport;
                    setpos = 0;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;



            case 5:
              lcd.setCursor(0, 2);
              lcd.print(F("  Webport Setting   "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = webport;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 65534) {
                      idszam = 65534;
                    }
                    if (idszam < 1) {
                      idszam = 1;
                    }
                    webport = idszam;
                    EEPROM.writeFloat(webportAddr, webport);
                    if (!dhcp) {
                      Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                      server.begin();
                    } else {
                      Ethernet.begin(mac);
                      server.begin();
                    }
                    lcd.setCursor(0, 3);
                    lcd.print(F("   Save Completed   "));
                    delay(2000);
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
                    if (idszam > 65534) {
                      idszam = 65534;
                    }
                    if (idszam < 1) {
                      idszam = 1;
                    }

                  } else {
                    spos = 4;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 6;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 65534) {
                      idszam = 65534;
                    }
                    if (idszam < 1) {
                      idszam = 1;
                    }
                  } else {
                    spos = 6;
                    idszam = udpport;
                  }
                  event = HIGH;
                  break;
                default:
                  event = LOW;
                  break;
              }
              break;


            case 6:
              lcd.setCursor(0, 2);
              lcd.print(F("  UDPport Setting   "));
              lcd.setCursor(0, 3);
              lcd.print(F("                    "));
              lcd.setCursor(0, 3);
              if (medit) {
                lcd.print(F("* "));
              } else {
                lcd.print(F("  "));
              }
              lcd.print(idszam, 0);
              switch (eventin) {
                case 4:
                  eventin = 0;
                  if (medit) {
                    medit = LOW;
                    idszam = udpport;
                  } else {
                    submenu = LOW;
                  }
                  event = HIGH;
                  break;
                case 1:
                  eventin = 0;
                  if (medit) {
                    if (idszam > 65534) {
                      idszam = 65534;
                    }
                    if (idszam < 1) {
                      idszam = 1;
                    }
                    udpport = idszam;
                    EEPROM.writeFloat(udpportAddr, udpport);
                    if (!dhcp) {
                      Ethernet.begin(mac, ip, dnServer, gateway, subnet);
                      server.begin();
                    } else {
                      Ethernet.begin(mac);
                      server.begin();
                    }
                    lcd.setCursor(0, 3);
                    lcd.print(F("   Save Completed   "));
                    delay(2000);
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
                    if (idszam > 65534) {
                      idszam = 65534;
                    }
                    if (idszam < 1) {
                      idszam = 1;
                    }

                  } else {
                    spos = 5;
                    idszam = webport;
                  }
                  event = HIGH;
                  break;
                case 3:
                  eventin = 6;
                  if (medit) {
                    idszam -= 1;
                    if (idszam > 65534) {
                      idszam = 65534;
                    }
                    if (idszam < 1) {
                      idszam = 1;
                    }
                  } else {
                    spos = 0;
                    iddont = dhcp;
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
     g változó: 1: A channel; 2: B channel; 3: ALL channel
     h változó: HIGH: Indítás; LOW: Leállítás
  */
  if (g == 1) {
    if (h) {
      if (!AcRun) {
        ATchannel = HIGH;
        relayupdate();
        delay(1500);
        ATchannel = LOW;
        Achannel = HIGH;
        relayupdate();
        AcRun = HIGH;
        delay(500);
        //checkerror();
      }
    } else {
      if (AcRun) {
        Achannel = LOW;
        ATchannel = HIGH;
        relayupdate();
        delay(1500);
        ATchannel = LOW;
        relayupdate();
        AcRun = LOW;
        delay(500);
        //checkerror();
      }
    }
  } else if (g == 2) {
    if (h) {
      if (!BcRun) {
        BTchannel = HIGH;
        relayupdate();
        delay(1500);
        BTchannel = LOW;
        Bchannel = HIGH;
        relayupdate();
        BcRun = HIGH;
        delay(500);
        //   checkerror();
      }
    } else {
      if (BcRun) {
        Bchannel = LOW;
        BTchannel = HIGH;
        relayupdate();
        delay(1500);
        BTchannel = LOW;
        relayupdate();
        BcRun = LOW;
        delay(500);
        //    checkerror();
      }
    }

  } else if (g == 3) {
    if (h) {
      if (!BcRun) {
        BTchannel = HIGH;
      }
      if (!AcRun) {
        ATchannel = HIGH;
      }
      relayupdate();
      delay(1500);
      if (!BcRun) {
        BTchannel = LOW;
      }
      if (!AcRun) {
        ATchannel = LOW;
      }
      relayupdate();
      if (!AcRun) {
        Achannel = HIGH;
        relayupdate();
        AcRun = HIGH;
        delay(1500);
      }
      if (!BcRun) {
        Bchannel = HIGH;
        relayupdate();
        BcRun = HIGH;
      }
      delay(500);
      //       checkerror();
    } else {
      if (AcRun || BcRun) {
        Achannel = LOW;
        Bchannel = LOW;
        ATchannel = HIGH;
        BTchannel = HIGH;
        relayupdate();
        AcRun = LOW;
        BcRun = LOW;
        delay(1500);
        ATchannel = LOW;
        BTchannel = LOW;
        relayupdate();
        delay(500);
        //   checkerror();
      }

    }
  }



}

void relayupdate() {
  digitalWrite(Achannelpin, !Achannel);
  digitalWrite(Bchannelpin, !Bchannel);
  digitalWrite(ATchannelpin, !ATchannel);
  digitalWrite(BTchannelpin, !BTchannel);
  digitalWrite(Acerrorpin, !Acerror);
  digitalWrite(Bcerrorpin, !Bcerror);
  digitalWrite(MainContactpin, !MainContact);
}

void pressuremeter() {
  float mert1 = 0;
  float mert2 = 0;
  for (int i = 0; i < minta; i++) {
    mert1 += analogRead(sensorA);
    mert2 += analogRead(sensorB);
  }
  mert1 /= minta; //Átlag
  //Serial.print("Atlag:");
  //Serial.println(mert1,1);
  mert2 /= minta;
  if (AcCalib >= mert1) {
    mert1 = 0;
  } else {
    mert1 -= AcCalib; //Eltolás kivonása
    // Serial.print("ACCalib:");
    // Serial.println(mert1,1);
    mert1 *= felbontas; //Feszültségben
    //Serial.print("Fesz:");
    //Serial.println(mert1,1);
    mert1 -= Acrangemin; //A méréshatár alsó részét kivonni.
    mert1 /= Acsensorrange; //PSI ben mérve
    // Serial.print("PSI:");
    //  Serial.println(mert1,1);
    if (!Acpsibar) {
      mert1 *= barvalto; //Bar ban megadva
    }
    //  Serial.print("BAR:");
    //   Serial.println(mert1,1);
  }

  if (BcCalib > mert2) {
    mert2 = 0;
  } else {
    mert2 -= BcCalib;
    mert2 *= felbontas;
    mert2 -= Bcrangemin;
    mert2 /= Bcsensorrange;
    if (!Bcpsibar) {
      mert2 *= barvalto;
    }
  }
  AchAct = mert1;
  BchAct = mert2;
}

void dataupdate() {
  Acrange = EEPROM.readFloat(AcrangeAddr);
  Acrangemax  = EEPROM.readFloat(AcrangemaxAddr);
  Acrangemin = EEPROM.readFloat(AcrangeminAddr);
  Bcrange = EEPROM.readFloat(BcrangeAddr);
  Bcrangemax = EEPROM.readFloat(BcrangemaxAddr);
  Bcrangemin = EEPROM.readFloat(BcrangeminAddr);
  Acsensorrange = (Acrangemax - Acrangemin) / Acrange;
  Bcsensorrange = (Bcrangemax - Bcrangemin) / Bcrange;
  EEPROM.writeFloat(AcsensorrangeAddr, Acsensorrange);
  EEPROM.writeFloat(BcsensorrangeAddr, Bcsensorrange);
}

void pressure() {
  int i = 1;
  Serial.println("Pressure Read Test 20X");
  Serial.println("-----------------START----------------");
  Serial.println("Channel - Bar - Time - Num");
  for (i = 1; i <= 20; i++) {
    pressuremeter();
    Serial.print("A - ");
    Serial.print(AchAct, 2);
    Serial.print(" - ");
    if (RTC.read(tm)) {
      print2digits(tm.Hour);
      Serial.write(':');
      print2digits(tm.Minute);
      Serial.write(':');
      print2digits(tm.Second);
    }
    Serial.print(" - ");
    Serial.println(i);

    Serial.print("B - ");
    Serial.print(BchAct, 2);
    Serial.print(" - ");
    if (RTC.read(tm)) {
      print2digits(tm.Hour);
      Serial.print(':');
      print2digits(tm.Minute);
      Serial.print(':');
      print2digits(tm.Second);
    }
    Serial.print(" - ");
    Serial.println(i);
    delay(200);
  }
  Serial.println("-----------------END----------------");
}

void configure() {
  Serial.println("Actual configuration");
  Serial.println("-----------------START----------------");
  Serial.println("Data Name - Value - Unit");
  Serial.println("---------Master---------");
  Serial.print("Master Set Point  - ");
  Serial.print(masterSP, 1);
  Serial.println("  - bar");
  Serial.print("Master Hysteresis   - ");
  Serial.print(masterHys, 1);
  Serial.println("  - bar");
  Serial.println("---------Slave---------");
  Serial.print("Slave Set Point  - ");
  Serial.print(slaveSP, 1);
  Serial.println("  - bar");
  Serial.print("Slave Hysteresis   - ");
  Serial.print(slaveHys, 1);
  Serial.println("  - bar");
  Serial.println("---------A channel---------");
  Serial.print("'A' Channel Calibration   - ");
  Serial.print(AcCalib);
  Serial.println("  - num");
  Serial.print("'A' Channel type - ");
  if (Acpsibar) {
    Serial.print("Bar");
  } else {
    Serial.print("Psi");
  }
  Serial.println("");
  Serial.print("'A' Channel range   - ");
  Serial.print(Acrange);
  Serial.println("  - psi/bar");
  Serial.print("'A' Channel range MIN   - ");
  Serial.print(Acrangemin);
  Serial.println("  - Volt");
  Serial.print("'A' Channel range MAX   - ");
  Serial.print(Acrangemax);
  Serial.println("  - Volt");
  Serial.println("---------B channel---------");
  Serial.print("'B' Channel Calibration   - ");
  Serial.print(BcCalib);
  Serial.println("  - num");
  Serial.print("'B' Channel type - ");
  if (Bcpsibar) {
    Serial.print("Bar");
  } else {
    Serial.print("Psi");
  }
  Serial.println("");
  Serial.print("'B' Channel range   - ");
  Serial.print(Bcrange);
  Serial.println("  - psi/bar");
  Serial.print("'B' Channel range MIN   - ");
  Serial.print(Bcrangemin);
  Serial.println("  - Volt");
  Serial.print("'B' Channel range MAX   - ");
  Serial.print(Bcrangemax);
  Serial.println("  - Volt");
  Serial.println("---------Common---------");
  Serial.print("Update Time   - ");
  Serial.print(updatetime);
  Serial.println("  - msec");
  Serial.print("Sample number   - ");
  Serial.print(minta);
  Serial.println("  - sample/round");
  Serial.println("-----------------END----------------");
}

void actstatus() {
  pressuremeter();
  Serial.println("Actual Status");
  Serial.println("-----------------START----------------");
  Serial.println("Data Name - Value - Unit");
  Serial.print("A ch actual value  - ");
  Serial.print(AchAct, 1);
  Serial.println("  - bar");
  Serial.print("B ch actual value   - ");
  Serial.print(BchAct, 1);
  Serial.println("  - bar");
  Serial.print("Priority channel  - ");
  if (Acprio) {
    Serial.println("A channel");
  } else if (Bcprio) {
    Serial.println("B channel");
  } else {
    Serial.println("Boost mode");
  }
  relaystatus();
  runstatus();
  Serial.println("-----------------END----------------");
}

void runstatus() {
  Serial.println("--------RUN Status----------");
  Serial.print("A channel: ");
  if (AcRun) {
    Serial.println("RUN");
  } else {
    Serial.println("OFF");
  }
  Serial.print("B channel: ");
  if (BcRun) {
    Serial.println("RUN");
  } else {
    Serial.println("OFF");
  }
}

void relaystatus() {
  Serial.print("'A' channel relay   - ");
  if (Achannel) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
  Serial.print("'B' channel relay   - ");
  if (Bchannel) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
  Serial.print("'AT' channel relay   - ");
  if (ATchannel) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
  Serial.print("'BT' channel relay   - ");
  if (BTchannel) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
}

void calibration() {
  int mert1 = 0;
  int mert2 = 0;
  for (int i = 0; i < 100; i++) {
    mert1 += analogRead(sensorA);
    mert2 += analogRead(sensorB);
  }
  AcCalib = mert1 / 100;
  BcCalib = mert2 / 100;
  EEPROM.writeFloat(AcCalibAddr, AcCalib);
  EEPROM.writeFloat(BcCalibAddr, BcCalib);

}

void calibrationReset() {
  AcCalib = 0;
  BcCalib = 0;
  EEPROM.writeFloat(AcCalibAddr, AcCalib);
  EEPROM.writeFloat(BcCalibAddr, BcCalib);
}


void terminal() {
  int param1 = 0;
  float param2 = 0;

  if (inputString == "pressure\n") {
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
    pressure();
  } else if (inputString == "calibration\n") {
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
    calibration();
    Serial.println(F("Calibration Done ALL channel!"));
  } else if (inputString.startsWith("config")) {
    Serial.println(inputString);
    if (inputString == "config\n") {
      configure();
    } else {
      switch (inputString.substring(7, 9).toInt())
      {
        case 1:
          Serial.print(F("Master Set Point SET: "));
          param2 = inputString.substring(9, 13).toFloat();
          if (param2 >= 1 && param2 <= 10) {
            EEPROM.writeFloat(masterSPAddr, param2);
            masterSP = param2;
            Serial.println(param2, 1);
            param2 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:1, Max:10"));
          }
          break;
        case 2:
          Serial.print(F("Master Hysteresis SET: "));
          param2 = inputString.substring(9, 13).toFloat();
          if (param2 >= 1 && param2 <= 10) {
            EEPROM.writeFloat(masterHysAddr, param2);
            masterSP = param2;
            Serial.println(param2, 1);
            param2 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:1, Max:10"));
          }
          break;
        case 3:
          Serial.print(F("Slave Set Point SET: "));
          param2 = inputString.substring(9, 13).toFloat();
          if (param2 >= 1 && param2 <= 10) {
            EEPROM.writeFloat(slaveSPAddr, param2);
            masterSP = param2;
            Serial.println(param2, 1);
            param2 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:1, Max:10"));
          }
          break;
        case 4:
          Serial.print(F("Slave Hysteresis SET: "));
          param2 = inputString.substring(9, 13).toFloat();
          if (param2 >= 1 && param2 <= 10) {
            EEPROM.writeFloat(slaveHysAddr, param2);
            masterSP = param2;
            Serial.println(param2, 1);
            param2 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:1, Max:10"));
          }
          break;
        case 5:
          Serial.print(F("A channel PSI/BAR SET: "));
          param1 = inputString.substring(9, 10).toInt();
          if (param1 == 1) {
            EEPROM.writeFloat(AcpsibarAddr, HIGH);
            Acpsibar = HIGH;
            Serial.println(F("BAR"));
            param1 = 0;
          } else if (param1 == 0) {
            EEPROM.writeFloat(AcpsibarAddr, LOW);
            Acpsibar = LOW;
            Serial.println(F("PSI"));
            param1 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: 1=BAR, 0=PSI"));
          }
          break;
        case 6:
          Serial.print(F("A channel range SET: "));
          param1 = inputString.substring(9, 13).toInt();
          if (param1 >= 1 && param1 <= 9000) {
            EEPROM.writeFloat(AcrangeAddr, param1);
            Acrange = param1;
            Serial.println(Acrange);
            param1 = 0;
            dataupdate();
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:1, Max:9000"));
          }
          break;
        case 7:
          Serial.print(F("A channel range MIN SET: "));
          param2 = inputString.substring(9, 12).toFloat();
          if (param2 >= 0 && param2 <= 5) {
            EEPROM.writeFloat(AcrangeminAddr, param2);
            Acrangemin = param2;
            Serial.println(Acrangemin, 1);
            param2 = 0;
            dataupdate();
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:0, Max:5"));
          }
          break;
        case 8:
          Serial.print(F("A channel range MAX SET: "));
          param2 = inputString.substring(9, 12).toFloat();
          if (param2 >= 0 && param2 <= 5) {
            EEPROM.writeFloat(AcrangemaxAddr, param2);
            Acrangemax = param2;
            Serial.println(Acrangemax, 1);
            param2 = 0;
            dataupdate();
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:0, Max:5"));
          }
          break;
        case 9:
          Serial.print(F("B channel PSI/BAR SET: "));
          param1 = inputString.substring(9, 10).toInt();
          if (param1 == 1) {
            EEPROM.writeFloat(BcpsibarAddr, HIGH);
            Bcpsibar = HIGH;
            Serial.println(F("BAR"));
            param1 = 0;
          } else if (param1 == 0) {
            EEPROM.writeFloat(BcpsibarAddr, LOW);
            Acpsibar = LOW;
            Serial.println(F("PSI"));
            param1 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: 1=BAR, 0=PSI"));
          }
          break;
        case 10:
          Serial.print(F("B channel range SET: "));
          param1 = inputString.substring(10, 14).toInt();
          if (param1 >= 1 && param1 <= 9000) {
            EEPROM.writeFloat(BcrangeAddr, param1);
            Bcrange = param1;
            Serial.println(Bcrange);
            param1 = 0;
            dataupdate();
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:1, Max:9000"));
          }
          break;
        case 11:
          Serial.print(F("B channel range MIN SET: "));
          param2 = inputString.substring(10, 13).toFloat();
          if (param2 >= 0 && param2 <= 5) {
            EEPROM.writeFloat(BcrangeminAddr, param2);
            Bcrangemin = param2;
            Serial.println(Bcrangemin, 1);
            param2 = 0;
            dataupdate();
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:0, Max:5"));
          }
          break;
        case 12:
          Serial.print(F("B channel range MAX SET: "));
          param2 = inputString.substring(10, 13).toFloat();
          if (param2 >= 0 && param2 <= 5) {
            EEPROM.writeFloat(BcrangemaxAddr, param2);
            Bcrangemax = param2;
            Serial.println(Bcrangemax, 1);
            param2 = 0;
            dataupdate();
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:0, Max:5"));
          }
          break;
        case 13:
          Serial.print(F("Updatetime SET: "));
          param1 = inputString.substring(10, 14).toInt();
          if (param1 >= 500 && param1 <= 5000) {
            EEPROM.writeFloat(updatetimeAddr, param1);
            updatetime = param1;
            Serial.println(updatetime);
            param1 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:500, Max:5000"));
          }
          break;
        case 14:
          Serial.print(F("Sample number SET: "));
          param1 = inputString.substring(10, 13).toInt();
          if (param1 >= 10 && param1 <= 100) {
            EEPROM.writeFloat(mintaAddr, param1);
            minta = param1;
            Serial.println(minta);
            param1 = 0;
          } else {
            Serial.println(F("error"));
            Serial.println(F("Param-2: Min:10, Max:100"));
          }
          break;
        default:
          Serial.println(F("Error parameter. Param-1"));
          break;
      }
    }
    inputString = "";
    stringComplete = false;
  } else if (inputString == "status\n") {
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
    actstatus();
  } else if (inputString == "help\n") {
    Serial.println(inputString);
    Serial.println(F("-help: Help!"));
    Serial.println(F("-pressure: Ask pressure 20x"));
    Serial.println(F("-config X Y: Actual configuration:"));
    Serial.println(F("            X:"));
    Serial.println(F("               1, Master SP SET; Y: 00.0"));
    Serial.println(F("               2, Master Hysteresis SET; Y: 00.0"));
    Serial.println(F("               3, Slave SP SET; Y: 00.0"));
    Serial.println(F("               4, Slave Hysteresis SET; Y: 00.0"));
    Serial.println(F("               5, A channel PSI/BAR SET; Y: 1=BAR, 0=PSI"));
    Serial.println(F("               6, A channel range SET; Y: 0000"));
    Serial.println(F("               7, A channel range MIN SET; Y: 0.0"));
    Serial.println(F("               8, A channel range MAX SET; Y: 0.0"));
    Serial.println(F("               9, B channel PSI/BAR SET; Y: 1=BAR, 0=PSI"));
    Serial.println(F("               10, B channel range SET; Y: 0000"));
    Serial.println(F("               11, B channel range MIN SET; Y: 0.0"));
    Serial.println(F("               12, B channel range MAX SET; Y: 0.0"));
    Serial.println(F("               13, Update time SET; Y: 0000"));
    Serial.println(F("               14, Sample number SET; Y: 000"));
    Serial.println(F("-calibration: ALL channel calibration. PLEASE take off all pressure sensor from air load! (Before use this function!)"));
    Serial.println(F("-auto X: Auto mode x: 0=OFF,1=ON"));
    Serial.println(F("-status: Actual pressure and relay state."));
    Serial.println(F("-manipulator X Y: Channel START/STOP; X: 1,2,3 (1: A channel; 2: B channel; 3: ALL channel) Y:1,2 (1: START; 2: STOP) "));
    Serial.println(F("-relay X Y: Change relay state: X Y: X:0,1,2,3(0=Achannel,1=Bchannel,2=A Tehermentesites,3=B Tehermentesites Y:0,1(Ki=0 vagy Be=1)"));
    Serial.println(F("-error X Y: error state-config: X Y: X:0,1,2(0= Reset error status,1=Enable or disable channel contactor check,2=Enabel or disable Over current contactor) Y:0,1(Kikapcsol=0,Bekapcsol=1)"));
    inputString = "";
    stringComplete = false;
  } else if (inputString.startsWith("auto")) {
    Serial.println(inputString);
    switch (inputString.substring(5, 6).toInt())
    {
      case 1:
        automode = HIGH;
        Serial.println(F("AUTO mode - ON"));
        break;
      case 0:
        automode = LOW;
        Serial.println(F("AUTO mode - OFF"));
        Achannel = LOW;
        Bchannel = LOW;
        ATchannel = LOW;
        BTchannel = LOW;
        relayupdate();
        break;
      case 3:
        if (automode) {
          Serial.println(F("AUTO mode - ON"));
        } else {
          Serial.println(F("AUTO mode - OFF"));
        }
        break;
      default:
        Serial.println(F("Error parameter. Param-1"));
        break;
    }
    inputString = "";
    stringComplete = false;
  } else if (inputString.startsWith("manipulator")) {
    Serial.println(inputString);
    if (inputString == "manipulator\n") {
      runstatus();
    } else {
      switch (inputString.substring(12, 13).toInt())
      {
        case 1:
          switch (inputString.substring(14, 15).toInt())
          {
            case 1:
              manipulator(1, HIGH);
              Serial.println(F("A channel - RUN"));
              break;
            case 0:
              manipulator(1, LOW);
              Serial.println(F("A channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        case 2:
          switch (inputString.substring(14, 15).toInt())
          {
            case 1:
              manipulator(2, HIGH);
              Serial.println(F("B channel - RUN"));
              break;
            case 0:
              manipulator(2, LOW);
              Serial.println(F("B channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        case 3:
          switch (inputString.substring(14, 15).toInt())
          {
            case 1:
              manipulator(3, HIGH);
              Serial.println(F("ALL channel - RUN"));
              break;
            case 0:
              manipulator(3, LOW);
              Serial.println(F("ALL channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        default:
          Serial.println(F("Error parameter. Param-1"));
          break;
      }
    }
    inputString = "";
    stringComplete = false;
  } else if (inputString.startsWith("error")) {
    Serial.println(inputString);
    if (inputString == "error\n") {
      Serial.print(F("Actual error status: "));
      Serial.println(ErrString(errorcode));
    } else {
      switch (inputString.substring(6, 7).toInt())
      {
        case 0:
          errorcode = 0;
          Serial.println(F("Error status reset!"));
          Serial.print(F("Actual error status: "));
          Serial.println(ErrString(errorcode));
          break;
        case 1:
          switch (inputString.substring(8, 9).toInt())
          {
            case 1:
              checkrelayfb = HIGH;
              EEPROM.writeFloat(checkrelayfbAddr, checkrelayfb);
              Serial.println(F("Channel contactor check - ON"));
              break;
            case 0:
              checkrelayfb = LOW;
              EEPROM.writeFloat(checkrelayfbAddr, checkrelayfb);
              Serial.println(F("Channel contactor check - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        case 2:
          switch (inputString.substring(8, 9).toInt())
          {
            case 1:
              checkampfb = HIGH;
              EEPROM.writeFloat(checkampfbAddr, checkampfb);
              Serial.println(F("Over amp check - ON"));
              break;
            case 0:
              checkampfb = LOW;
              EEPROM.writeFloat(checkampfbAddr, checkampfb);
              Serial.println(F("Over amp check - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        default:
          Serial.println(F("Error parameter. Param-1"));
          break;
      }
    }
    inputString = "";
    stringComplete = false;
  } else if (inputString.startsWith("relay")) {
    Serial.println(inputString);
    if (inputString == "relay\n") {
      relaystatus();
    } else {
      switch (inputString.substring(6, 7).toInt())
      {
        case 0:
          switch (inputString.substring(8, 9).toInt())
          {
            case 1:
              Achannel = HIGH;
              Serial.println(F("A channel - ON"));
              break;
            case 0:
              Achannel = LOW;
              Serial.println(F("A channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        case 1:
          switch (inputString.substring(8, 9).toInt())
          {
            case 1:
              Bchannel = HIGH;
              Serial.println(F("B channel - ON"));
              break;
            case 0:
              Bchannel = LOW;
              Serial.println(F("B channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        case 2:
          switch (inputString.substring(8, 9).toInt())
          {
            case 1:
              ATchannel = HIGH;
              Serial.println(F("AT channel - ON"));
              break;
            case 0:
              ATchannel = LOW;
              Serial.println(F("AT channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        case 3:
          switch (inputString.substring(8, 9).toInt())
          {
            case 1:
              BTchannel = HIGH;
              Serial.println(F("BT channel - ON"));
              break;
            case 0:
              BTchannel = LOW;
              Serial.println(F("BT channel - OFF"));
              break;
            default:
              Serial.println(F("Error parameter. Param-2"));
              break;
          }
          break;
        default:
          Serial.println(F("Error parameter. Param-1"));
          break;
      }
      if (!automode) {
        relayupdate();
      }
    }
    inputString = "";
    stringComplete = false;
  } else {
    Serial.println(inputString);
    Serial.println(F("Nem ismert parancs! >help"));
    inputString = "";
    stringComplete = false;
  }
}


bool getTime(const char *str) {
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str) {
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
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

void print2digitslcd2(int number) {
  if (number >= 0 && number < 10) {
    lcd2.print("0");
  }
  lcd2.print(number);
}

void print2digitslcd2ures(int number) {
  if (number >= 0 && number < 10) {
    lcd2.print(" ");
  }
  lcd2.print(number);
}

void print2digitsEth(int number, EthernetClient client) {
  if (number >= 0 && number < 10) {
    client.print("0");
  }
  client.print(number);
}