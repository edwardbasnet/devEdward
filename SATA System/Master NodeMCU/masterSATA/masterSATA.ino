
/*
 * MeroTV automation Firmware V2.0: masterSATA.ino
 * June 2017
 * (c)edward
 */
 
#include <ESP8266WiFi.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <QuickStats.h>
QuickStats stats;


//Define I/O pins
const int P_A_PIN = 5;
const int P_B_PIN = 4;
const int P_C_PIN = 0;

#define SIREN1 2 
#define SIREN2 14
#define BEEP 12
#define MUTE 13

//temperature and smoke range
#define TEMP_THRS_MIN 30
#define TEMP_THRS_MAX 40

#define SMOKE_THERS 500

//Authentication variables
char* SATAssid; //server wifi name
char* SATApassword; //server password
//float temperature;
//int lpg, co, smoke;


const uint8_t MAXC = 1;  //maximum number of clients
const int SAMPLE = 50;

WiFiServer SATAServer(9001);  //the server and the port number
WiFiClient SATAClient[MAXC];

//AC detection
boolean A_D_FLAG, B_D_FLAG, C_D_FLAG;
float P_A_STATUS[] = {0}, P_B_STATUS[SAMPLE] = {0}, P_C_STATUS[SAMPLE] = {0}, P_STATUS[] = {0};
unsigned long A_D_COUNT = 0, B_D_COUNT = 0, C_D_COUNT = 0;


void setup() {
  
  //I2C lcd inicialize
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  
  Serial.begin(9600);

  pinMode(P_A_PIN, INPUT);
  pinMode(P_B_PIN, INPUT);
  pinMode(P_C_PIN, INPUT);
  pinMode(BEEP, OUTPUT);
  pinMode(SIREN1, OUTPUT);
  pinMode(SIREN2, OUTPUT);
  pinMode(MUTE, INPUT_PULLUP);
  
/******************** WiFi Access point creation ************************/
  SetWifi("SATAServer", "password@SATA");
  }

void loop() {
  char Message[] = {0};
  String info = "";
  if (AvailableClients()) {
    Serial.print("Message : ");
    info = AvailableMessage();
    Serial.println(info);
    parseMessage(info);
    }
    
    
    delay(1000);
  }
  
void parseMessage(String that) {
  String T = "", L = "", C = "", S = "", temp = "";
  float temperature = 0.00;
  int lpg = 0, co = 0, smoke = 0;
  
  if((that.startsWith("#")) && that.endsWith("!")) {
    T = that.substring(1, that.indexOf(","));
    temperature = T.toInt() / 1000.00;
    Serial.println(temperature);
    
    temp = that.substring(that.indexOf(",") + 1);
    L = temp.substring(0, temp.indexOf(","));
    lpg = L.toInt();
    Serial.println(lpg);
    
    temp = temp.substring(temp.indexOf(",") + 1);
    C = temp.substring(0, temp.indexOf(","));
    co = C.toInt();
    Serial.println(co);
    
    temp = temp.substring(temp.indexOf(",")+1);
    S = temp.substring(0, temp.indexOf("!"));
    smoke = S.toInt();
    Serial.println(smoke);
    }
  
  }

void SetWifi(char* Name, char* Password) {
    // Stop Any Previous WIFI
    WiFi.disconnect();

    // Setting The Wifi Mode
    WiFi.mode(WIFI_AP_STA);
    Serial.println("WIFI Mode : AccessPoint Station");
    
    // Setting The AccessPoint Name & Password
    SATAssid      = Name;
    SATApassword  = Password;
    
    // Starting The Access Point
    WiFi.softAP(SATAssid, SATApassword);
    Serial.println("WIFI < " + String(SATAssid) + " > ... Started");
    
    // Wait For Few Seconds
    delay(1000);
    
    // Getting Server IP
    IPAddress IP = WiFi.softAPIP();
    
    // Printing The Server IP Address
    Serial.print("AccessPoint IP : ");
    Serial.println(IP);

    // Printing MAC Address
    Serial.print("AccessPoint MC : ");
    Serial.println(String(WiFi.softAPmacAddress()));

    // Starting Server
    SATAServer.begin();
    SATAServer.setNoDelay(true);
    Serial.println("Server Started");
  }

boolean AvailableClients() {
  if (SATAServer.hasClient()) {
    if (SATAClient[0] = SATAServer.available()) {
      Serial.println("New Client at index 0 ");
      return true;
      }
    } else return false;
  }


String AvailableMessage() {
  String Message = "";
  if (SATAClient[0] && SATAClient[0].connected() && SATAClient[0].available()) {
    while(SATAClient[0].available()) {
      String Message = SATAClient[0].readStringUntil('\r');
      SATAClient[0].flush();
//      Serial.println("Client No " + String(0) + " - " + Message);
      return Message;
      }
    }
 }

//====================================================================================

void ACStatus() {
  
  //adding <SAMPLE> number of data to the respective array 
  for(int i=0; i<SAMPLE; i++) {
    P_A_STATUS[i] = digitalRead(P_A_PIN);
    P_B_STATUS[i] = digitalRead(P_B_PIN);
    P_C_STATUS[i] = digitalRead(P_C_PIN);
    }
    //calculating mode
    int P_A_ = stats.mode(P_A_STATUS, SAMPLE, 0.00001);
    int P_B_ = stats.mode(P_B_STATUS, SAMPLE, 0.00001);
    int P_C_ = stats.mode(P_C_STATUS, SAMPLE, 0.00001);

    
  }

