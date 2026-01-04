/*
 * MeroTV automation Firmware V1.0
 * October 2016
 * (c)edward basnet
 */
//  ----------------- Library Initialization  -----------------

//  capacitiveTouchPad 
//  Define the digital pins used for the clock and data
#define SCL_PIN 4
#define SDO_PIN 5

/* Used to store the key state */
byte Key;

//  K-type Thermocouple
#include <max6675.h>

int thermo_so_pin  = 10;
int thermo_cs_pin  = 11;
int thermo_sck_pin = 12;

MAX6675 thermocouple(thermo_sck_pin, thermo_cs_pin, thermo_so_pin);

//  MQ-2 smoke sensor

//  LCD
/* * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3) 
*/

#define LCD_RS 18
#define LCD_EN 15
#define LCD_D4 3
#define LCD_D5 6
#define LCD_D6 7
#define LCD_D7 8

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
/************************************************************************************/

//  Voltage level
#define VOL_LEVEL A6
float initVol, finalVol;
//  Charge Activation
#define CHARGE 19

/*********** AC phase detection **************/
#define PHASE_A A2
#define PHASE_B A3
#define PHASE_C A7

/********* Alarm **********/
#define SIREN 9
#define stopSIREN 2
#define BEEP 13

//Detect interrupt; if happen then activate touchpad and store the inputted value as Threshold

/*********** Smoke **********/
#define SMOKE A0
/****************************/

//  THRESHOLD voltage value for activate/deactivate the alerting mechanism
#define volUpperThreshold 12  //in volt
#define volLowerThreshold 7
#define tempUpperThreshold 29 //in 'C
#define tempLowerThreshold 23
#define smokeUpperThreshold 100  //analog value
#define smokeLowerThreshold 50
#define checkINTERVAL 120000  //in millisecond i.e. 1000ms = 1sec

boolean flag = LOW; 

boolean A_flag = LOW, B_flag = LOW, C_flag = LOW;
unsigned long A_down_count = 0, B_down_count = 0, C_down_count = 0;
unsigned long A_down_time = 0, B_down_time = 0, C_down_time = 0;


/****************** Start of setup() *****************/
void setup() {
  pinMode(SIREN, OUTPUT);
  digitalWrite(SIREN, HIGH);  //initiatinging SIREN in off 

// Initialise the serial interface 
  Serial.begin(9600);

//Initialize LCD
  lcd.begin(20,4);
    
// Battery Voltage and activate/deactivate charging mechanism
  pinMode(VOL_LEVEL, INPUT);
  pinMode(CHARGE, OUTPUT);
  initVol = readVoltage();
  
//Initialize Smoke Sensor
  pinMode(SMOKE, INPUT);


//Initialize Temperature Sensor and Callibrate


//Initialize TouchPad
  /* Configure the clock and data pins */
  //pinMode(SCL_PIN, OUTPUT);  
  //pinMode(SDO_PIN, INPUT);

//Detect AC Three Phase
  pinMode(PHASE_A, INPUT);
  pinMode(PHASE_B, INPUT);
  pinMode(PHASE_C, INPUT);
  
//Activate/Deactivate Alarm
  pinMode(BEEP, OUTPUT);
  pinMode(stopSIREN, INPUT);  //panic Switch used to turn off external siren
  pinMode(SIREN, OUTPUT);
  digitalWrite(SIREN, HIGH);  //initiatinging SIREN in off mode
}
/************ End of setup() ************/

/************************** main() Program *************************/


void loop() {
  Serial.println(digitalRead(stopSIREN));
  lcd.clear();
  lcd.begin(20,4);
  lcdPrint(1,0,"MERO-TV"); //arguments = setcursor(x, y), char
  lcdPrint(9,3,"(c)PETER-X");
     
//  AC phase online/offline status
// AC phase detection
  phaseA();
  if(millis() - A_down_time > checkINTERVAL){
    if(A_flag){
      lcdPrint(0,1,"A-DOWN");
      alert("A-DOWN",0,3000);
      }
    }else{
      lcdPrint(0,1,"        ");
      lcdPrint(0,1,"A-UP");
      }
  phaseB();
  if(millis() - B_down_time > checkINTERVAL){
    if(B_flag){
      lcdPrint(0,2,"B-DOWN");
      alert("B-DOWN",0,3000);
      }
    }else{
      lcdPrint(0,2,"        ");
      lcdPrint(0,2,"B-UP");
      }
  phaseC();
  if(millis() - C_down_time > checkINTERVAL){
    if(C_flag){
      lcdPrint(0,3,"C-DOWN");
      alert("C-DOWN",0,3000);
      }
    }else{
      lcdPrint(0,3,"        ");
      lcdPrint(0,3,"C-UP");
      }

//  Battery voltage level status
    //Serial.print("initVol : ");
    //Serial.println(initVol);
  float vol = readVoltage();
  lcd.setCursor(14,0);
  lcd.print(vol);
  lcdPrint(19,0,"v");
//  Battery Charging/Discharging
  if(vol <= volUpperThreshold && vol >= volLowerThreshold){
    digitalWrite(CHARGE, HIGH);
  }else digitalWrite(CHARGE, LOW);
    

//  Smoke Level 
  int z = smokeSense();
  //Serial.print("Smoke : ");
  lcdPrint(9,2,"S : ");
  //Serial.println(smokeSense());
  lcd.setCursor(13,2);
  lcd.print(z);
  lcdPrint(16,2,"ppm");
  if(z >= smokeUpperThreshold){
      if(!digitalRead(stopSIREN)){
        for(int g = 0; g<=5; g++){
          lcd.setCursor(7, 3);
          lcd.print("             ");
          lcd.setCursor(7, 3);
          lcd.print("Alert>>");
          lcd.setCursor(14,3);
          lcd.print("SMOKE");
          digitalWrite(BEEP, HIGH);
          delay(50);
          digitalWrite(BEEP, LOW);
          delay(100);
       }
      }else{
          lcd.setCursor(7, 3);
          lcd.print("             ");
          lcd.setCursor(7, 3);
          lcd.print("Alert>>");
          lcd.setCursor(14,3);
          lcd.print("SMOKE");
        }
  alert("SMOKE",0,2000);
  }
    //Serial.println("Smoke detected");
    
  //}
   
//  Temperature Level
  float total = 0, temp;
  for(int n=1; n<=20; n++){
    temp = thermocouple.readCelsius();
    total += temp;
    delay(2);
  }
  total = total / 20.00;
  
  lcdPrint(9,1,"T : ");
  lcd.setCursor(13,1);
  lcd.print(total);
  lcdPrint(18,1,"'C");
  
  if(total >= tempUpperThreshold){
      if(!digitalRead(stopSIREN)){
        for(int g = 0; g<=5; g++){
          lcd.setCursor(7, 3);
          lcd.print("             ");
          lcd.setCursor(7, 3);
          lcd.print("Alert>>");
          lcd.setCursor(14,3);
          lcd.print("T-RISE");
          digitalWrite(BEEP, HIGH);
          delay(50);
          digitalWrite(BEEP, LOW);
          delay(100);
       }
      }else{
          lcd.setCursor(7, 3);
          lcd.print("             ");
          lcd.setCursor(7, 3);
          lcd.print("Alert>>");
          lcd.setCursor(14,3);
          lcd.print("T-RISE");
        }
    alert("T-RISE",0,2000);
    delay(500);
  }

    lcdPrint(9,3,"(c)PETER-X");
    delay(500);
}
/*******************End of main program*******************/

/***************** Phase Sense ******************/
void phaseA(){
    if(digitalRead(PHASE_A)){
      A_down_count++;
      if(A_down_count==1){
        A_down_time = millis();
        A_flag = HIGH;
        }
    }else{
      A_down_count=0;
      A_down_time = millis();
      A_flag = LOW;
      }
}
    
void phaseB(){
  if(digitalRead(PHASE_B)){
      B_down_count++;
      if(B_down_count==1){
        B_down_time = millis();
        B_flag = HIGH;
        }
    }else{
      B_down_count=0;
      B_down_time = millis();
      B_flag = LOW;
      }
}

void phaseC(){
  if(analogRead(PHASE_C)>=50){
      C_down_count++;
      if(C_down_count==1){
        C_down_time = millis();
        C_flag = HIGH;
        }
    }else{
      C_down_count=0;
      C_down_time = millis();
      C_flag = LOW;
      }
}

/************** Voltage Sense***********/
 float readVoltage()
{
  float vout = 0.0, vin = 0.0, R1 = 22000.0, R2 = 10000.0; 
  int sensorValue = 0;
  //for(int k = 1; k<=20; k++){
  sensorValue = analogRead(VOL_LEVEL); //read the A0 pin value
  //Serial.println(sensorValue);
  //delay(20);
  //}
  //sensorValue = sensorValue / 19; //taking average value
  //Serial.println(sensorValue);
  vout = (sensorValue * 4.50) / 1024.0;   //(sensorValue * 5)/1023
  vin = vout / (R2 / (R1+R2));
  if(vin<0.09){vin = 0.0; //undesired reading
  }
  
  //Serial.print("Battery = ");
  //Serial.print(vin); //print the voltage to LCD
  //Serial.print(" v");
  //Serial.println();
 return vin;
}
/******* End of voltage level **********/

/*********** smoke **************/
int smokeSense(){
  int val, smokeValue = 0;
  for(int k = 0; k<=40; k++){
  smokeValue += analogRead(SMOKE); 
  delay(10);
  }
  val = smokeValue / 40;
  val = map(val, 0, 1023, 0, 255);
  Serial.println(val);
 return val;
  }

/*********** print something on lcd ***********/
void lcdPrint(int x, int y, char *z){  //x for column no. , y for row no. , z for character
  lcd.setCursor(x, y);
  lcd.print(z);
  }

/*************** Alert ***************/
void alert(char* k, boolean l, int m){  //k for string, l for boolean Siren Activate or deactivate, m for delaTime
  Serial.println(digitalRead(stopSIREN));
  if(digitalRead(stopSIREN)) {
    lcd.setCursor(7, 3);
    lcd.print("Alert>>");
    lcd.setCursor(14,3);
    lcd.print(k);
    delay(350);
    Serial.println("I am inside stopSiren funtion");
    for(int g = 0; g<=5; g++){
      digitalWrite(BEEP, HIGH);
      delay(50);
      digitalWrite(BEEP, LOW);
      delay(100);
    }
    lcd.setCursor(7, 3);
    lcd.print("             ");
  }
  if(!digitalRead(stopSIREN)){
    lcd.setCursor(7, 3);
    lcd.print("Alert>>");
    lcd.setCursor(14,3);
    lcd.print(k);
    delay(50);
    digitalWrite(SIREN, l);   
    delay(m);
    digitalWrite(SIREN, HIGH);
    lcd.setCursor(7, 3);
    lcd.print("             ");
    }
}



/* Read the state of the keypad */
/* byte Read_Keypad(void)
{
  byte Count;
  byte Key_State = 0;
  // Pulse the clock pin 16 times (one for each key of the keypad) 
  //   and read the state of the data pin on each pulse 
  for(Count = 1; Count <= 16; Count++)
  {
    digitalWrite(SCL_PIN, LOW); 
    // If the data pin is low (active low mode) then store the 
    //   current key number 
    if (!digitalRead(SDO_PIN))
      Key_State = Count; 
      digitalWrite(SCL_PIN, HIGH);
  }  
  return Key_State; 
}

//  End of Keypad
*/
