#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
//#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27, 16, 2);

 
const char* ssid = ".__LAB__";
const char* password = "lab@devil";

// Time to sleep (in seconds):
const int sleepTimeS = 10;

int ledPin = 12; 
int lcdPin = 14;
WiFiServer server(80);
unsigned long onTime = 0;
 
void setup() {
  Serial.begin(115200);
  delay(10);
//  lcd.init();
//  lcd.backlight();
//  lcd.setCursor(1,0);
//  lcd.print("LIGHT CONTROL");
 
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(lcdPin, OUTPUT);
  digitalWrite(lcdPin, LOW);
 
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
     WiFi.begin(ssid, password);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");
//
//  ArduinoOTA.onStart([]() {
//    Serial.println("Start");
//  });
//  ArduinoOTA.onEnd([]() {
//    Serial.println("End");
//  });
//  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
//  });
//  ArduinoOTA.onError([](ota_error_t error) {
//    Serial.printf("Error[%u]: ", error);
//    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//    else if (error == OTA_END_ERROR) Serial.println("End Failed");
//  });
//  ArduinoOTA.begin();
//  Serial.println("Ready");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
//  lcd.setCursor(1,1);
//  lcd.print("SERVER STARTED");
// 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  Serial.println(WiFi.macAddress());

//  // Sleep
//  Serial.println("ESP8266 in sleep mode");
//  ESP.deepSleep(sleepTimeS * 1000000);

 
}

boolean ledStatus = LOW, lcdStatus = LOW;
 
void loop() {
//  ArduinoOTA.handle();

  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  // Wait until the client sends some data
  //Serial.println("new client");
  while(!client.available()){
    delay(1);
  }
 
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();
 
  if (request.indexOf("/LED=ON") != -1)  {
    Serial.println("LED on");
    digitalWrite(ledPin, HIGH);
    ledStatus = HIGH;
//    lcd.backlight();
  }
  if (request.indexOf("/LED=OFF") != -1)  {
    Serial.println("LED off");
    digitalWrite(ledPin, LOW);
    ledStatus = LOW;
//    lcd.noBacklight();
  }else if (request.indexOf("/LCD=ON") != -1)  {
    Serial.println("LCD on");
    digitalWrite(lcdPin, HIGH);
    lcdStatus = HIGH;
//    lcd.backlight();
  }else if (request.indexOf("/LCD=OFF") != -1)  {
    Serial.println("LCD off");
    digitalWrite(lcdPin, LOW);
    lcdStatus = LOW;
//    lcd.noBacklight();
//  }
  
  
  // Return the response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  
  client.println("=============================");
  client.println("<br><br>");
  
  client.print("<h1>Home Automation</h1>");
  client.println("<br><br>");
  client.println("=============================");
  client.println("<br><br>");
  if(ledStatus) {
    client.println("LED : On  ");
  } else if(!ledStatus){
    client.println("LED : Off  ");
  }
  client.println("<a href=\"/LED=ON\"\" ><button>Turn On </button></a>");
  client.println("<a href=\"/LED=OFF\"\"><button>Turn Off </button></a>");
  
  client.println("<br><br>");
  client.println("________________________");
  client.println("<br><br>");
  
  if(lcdStatus) {
    client.print("LCD : On  ");
  } else if(!lcdStatus){
    client.print("LCD : Off  ");
  }
  client.println("<a href=\"/LCD=ON\"\"><button>Turn On </button></a>");
  client.println("<a href=\"/LCD=OFF\"\"><button>Turn Off </button></a><br />");
  
  client.println("________________________");
  client.println("<br><br>");  
  
  client.println("</html>");
 
  delay(1);
  Serial.println("Client disonnected");
  Serial.println("");
  //pLcd();
 
}

//void pLcd() {
//  lcd.clear();
//  lcd.setCursor(1,0);
//  lcd.print("LIGHT CONTROL");
//  lcd.setCursor(0,1);
//  if(ledStatus){
//  lcd.print("LED:ON");
//  }else lcd.print("LED:OFF");
//  lcd.setCursor(9,1);
//  if(lcdStatus){
//  lcd.print("LCD:ON");
//  }else lcd.print("LCD:OFF");
}
