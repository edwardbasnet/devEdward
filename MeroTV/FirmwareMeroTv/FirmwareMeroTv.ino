/*
 * MeroTV automation Firmware V1.0
 * October 2016
 * (c)edward
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
#define beepLowerINTERVAL 1200000 //20min
#define beepUpperINTERVAL 1320000 //22min
#define alertINTERVAL 3600000 //60min

//Detect interrupt; if happen then activate touchpad and store the inputted value as Threshold

//  MQ-2 smoke sensor
 
/************************Hardware Related Macros For Smoke************************************/
#define         MQ_PIN                       (A0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
 
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
 
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
/****************************/

//  THRESHOLD voltage value for activate/deactivate the alerting mechanism
#define volUpperThreshold 12  //in volt
#define volLowerThreshold 7
#define tempUpperThreshold 29 //in 'C
#define tempLowerThreshold 23
#define smokeUpperThreshold 130  //analog value
#define smokeLowerThreshold 50

#define checkINTERVAL 60000  //5min

boolean flag = LOW; 

boolean A_flag = LOW, B_flag = LOW, C_flag = LOW, A_alert_flag = LOW, B_alert_flag = LOW, C_alert_flag = LOW, Smoke_alert_flag = LOW, Temp_alert_flag = LOW;
unsigned long A_down_count = 0, B_down_count = 0, C_down_count = 0, A_up_count = 0, B_up_count = 0, C_up_count = 0;
unsigned long A_down_time = 0, B_down_time = 0, C_down_time = 0, A_alert_time = 0, B_alert_time = 0, C_alert_time = 0, Smoke_alert_time = 0, Temp_alert_time = 0;


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
  
//  Callibrating Smoke Sensor
  pinMode(MQ_PIN, INPUT);
  lcd.setCursor(1,1);
  lcd.print("Callibrating...");  
  lcd.setCursor(4,2);
  lcd.print("Smoke Sensor");
  lcd.setCursor(4,3);
  lcd.print("Temperature");
  Ro = MQCalibration(MQ_PIN);
  
//  Callibrating Temperature Sensor
  float tot=0, tem;
  for(int n=0; n<=100; n++){
    tem = thermocouple.readCelsius();
    tot += tem;
    delay(2);
  }
  tot = tot / 100.00;
  
//  Initialize TouchPad
  /* Configure the clock and data pins */
  //pinMode(SCL_PIN, OUTPUT);  
  //pinMode(SDO_PIN, INPUT);

//  Detect AC Three Phase
  pinMode(PHASE_A, INPUT);
  pinMode(PHASE_B, INPUT);
  pinMode(PHASE_C, INPUT);
  
//  Activate/Deactivate Alarm
  pinMode(BEEP, OUTPUT);
  pinMode(stopSIREN, INPUT);  //panic Switch used to turn off external siren
  pinMode(SIREN, OUTPUT);
  digitalWrite(SIREN, HIGH);  //initiatinging SIREN in off mode
}
/************ End of setup() ************/

/************************** main() Program *************************/


void loop() {
  lcd.clear();
  lcdPrint(1,0,"MERO-TV"); //arguments = setcursor(x, y), char
  lcdPrint(9,3,"(c)PETER-X");
     
//  AC phase online/offline status
// phase-A detection
  phaseA();
  if(millis() - A_down_time > checkINTERVAL){
    if(A_flag){
      lcdPrint(0,1,"A-DOWN");
      lcd.setCursor(7, 3);
      lcd.print("             ");
      lcd.setCursor(7, 3);
      lcd.print("Alert>>A-DOWN");
      alertADown(3000);
      if((millis() - A_alert_time > beepLowerINTERVAL) && A_alert_flag && (millis() - A_alert_time < beepUpperINTERVAL)){
        beepAlert();
        }
      if((millis() - A_alert_time > alertINTERVAL) && A_alert_flag){
        A_alert_flag = LOW;    
        }
     }
    //A_up_count = 0;
    }else {
      lcdPrint(0,1,"        ");
      lcdPrint(0,1,"A-UP");      
      }
// phase-B detection
  phaseB();
  if(millis() - B_down_time > checkINTERVAL){
    if(B_flag){
      lcdPrint(0,2,"B-DOWN");
      lcd.setCursor(7, 3);
      lcd.print("             ");
      lcd.setCursor(7, 3);
      lcd.print("Alert>>B-DOWN");
      alertBDown(3000);
      if((millis() - B_alert_time > beepLowerINTERVAL) && B_alert_flag && (millis() - B_alert_time < beepUpperINTERVAL)){
        beepAlert();
        }
      if((millis() - B_alert_time > alertINTERVAL) && B_alert_flag){
        B_alert_flag = LOW;
        }
      }
    //B_up_count = 0;
    }else{
      lcdPrint(0,2,"        ");
      lcdPrint(0,2,"B-UP");      
      }
// phase-C detection      
  phaseC();
  if(millis() - C_down_time > checkINTERVAL){
    if(C_flag){
      lcdPrint(0,3,"C-DOWN");
      lcd.setCursor(7, 3);
      lcd.print("             ");
      lcd.setCursor(7, 3);
      lcd.print("Alert>>C-DOWN");
      alertCDown(3000);
      if((millis() - C_alert_time > beepLowerINTERVAL) && C_alert_flag && (millis() - C_alert_time < beepUpperINTERVAL)){
        beepAlert();
        }
      if((millis() - C_alert_time > alertINTERVAL) && C_alert_flag){
        C_alert_flag = LOW;
        }
      }
    //C_up_count = 0;
    }else{
      lcdPrint(0,3,"        ");
      lcdPrint(0,3,"C-UP");      
      }

//  voltage level
  lcd.setCursor(14,0);
  lcd.print(readVoltage());
  //Serial.print(readVoltage());
  lcdPrint(19,0,"v");
  
//  Battery Charge/!Charge
  if(readVoltage() <= volUpperThreshold){
    digitalWrite(CHARGE, HIGH);
  }else digitalWrite(CHARGE, LOW);
    

//  Smoke Level 
  lcdPrint(9,2,"S : ");
  lcd.setCursor(13,2);
  lcd.print(smokeSense());
  lcdPrint(16,2,"ppm");
  if(smokeSense() > smokeUpperThreshold){
    lcdPrint(7,3, "             ");
    lcdPrint(7,3, "Alert>>SMOKE");
    alertSmoke(2000);
    if((millis() - Smoke_alert_time > beepLowerINTERVAL) && Smoke_alert_flag && (millis() - Smoke_alert_time < beepUpperINTERVAL)){
        beepAlert();
        }
    if((millis() - Smoke_alert_time > alertINTERVAL) && Smoke_alert_flag){
        Smoke_alert_flag = LOW;
       }
    }
   
//  Temperature Level
  float total, temp=0;
  for(int n=0; n<=40; n++){
    temp += thermocouple.readCelsius();
    //Serial.println(temp);
    delay(2);
  }
  total = temp / 40;
  lcdPrint(9,1,"T : ");
  lcd.setCursor(13,1);
  lcd.print(total);
  lcdPrint(18,1,"'C");
  if(total > tempUpperThreshold){
    lcdPrint(7,3, "             ");
    lcdPrint(7,3, "Alert>>T-RISE");
    alertTemp(3000);
    if((millis() - Temp_alert_time > beepLowerINTERVAL) && Temp_alert_flag && (millis() - Temp_alert_time < beepUpperINTERVAL)){
        beepAlert();
        }
    if((millis() - Temp_alert_time > alertINTERVAL) && Temp_alert_flag){
       Temp_alert_flag = LOW;
       }
    delay(500);
  }
  delay(1000);
    lcdPrint(9,3,"(c)PETER-X");
    delay(500);
}
/*******************End of main program*******************/

/***************** Phase Sense ******************/
void phaseA(){
    if(digitalRead(PHASE_A)){
      Serial.print("A- down");
      A_down_count++;
      if(A_down_count==1){
        A_down_time = millis();
        A_flag = HIGH;
        }
    }else{
      Serial.print("A- up");
      //A_up_count++;
      A_down_count=0;
      A_down_time = millis();
      A_flag = LOW;
      }
}
    
void phaseB(){
  if(digitalRead(PHASE_B)){
      Serial.print("B- down");
      B_down_count++;
      if(B_down_count==1){
        B_down_time = millis();
        B_flag = HIGH;
        }
    }else{
      Serial.print("B- up");
      //B_up_count++;
      B_down_count=0;
      B_down_time = millis();
      B_flag = LOW;
      }
}

void phaseC(){
  if(analogRead(PHASE_C)>=50){
      Serial.print("C- down");
      C_down_count++;
      if(C_down_count==1){
        C_down_time = millis();
        C_flag = HIGH;
        }
    }else{
      Serial.print("C-up");
      //C_up_count++;
      C_down_count=0;
      C_down_time = millis();
      C_flag = LOW;
      }
}

/************** Voltage Sense***********/
 float readVoltage()
{
  float vout = 0.0, vin = 0.0, R1 = 22000.0, R2 = 10000.0; 
  long int sensorValue=0, temp;
  for(int k = 0; k<=20; k++){
  sensorValue += analogRead(VOL_LEVEL); //read the A0 pin value
  //Serial.println(sensorValue);
  delay(2);
  }
  temp = sensorValue / 20; //taking average value
  //Serial.println(sensorValue);
  vout = (temp * 4.50) / 1024.00;   //(sensorValue * 5)/1023
  vin = vout * ((R1+R2) / R2);
  if(vin<0.09){vin = 0.0; //undesired reading
  }

 return vin;
}
/******* End of voltage level **********/

/*********** smoke **************/
int smokeSense(){
  int val, smokeValue = 0;
  for(int k = 0; k<=40; k++){
  smokeValue += analogRead(MQ_PIN);
  delay(2);
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

void beepAlert(){
  for(int g = 0; g<=5; g++){
      digitalWrite(BEEP, HIGH);
      delay(50);
      digitalWrite(BEEP, LOW);
      delay(100);
    }
  }

void alertADown(int m){  //k for string, l for boolean Siren Activate or deactivate, m for delay Time
 if(!A_alert_flag){
  Serial.println(digitalRead(stopSIREN));
  if(digitalRead(stopSIREN)){
    lcd.setCursor(7, 3);
    lcd.print("Alert>>A-DOWN");
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
    lcd.print("Alert>>A-DOWN");
    delay(350);
    digitalWrite(SIREN, LOW);   
    delay(m);
    digitalWrite(SIREN, HIGH);
    delay(1000);
    lcd.setCursor(7, 3);
    lcd.print("             ");
    }
  A_alert_time = millis();
  A_alert_flag = HIGH;
}
}

void alertBDown(int m){  //k for string, l for boolean Siren Activate or deactivate, m for delaTime
 if(!B_alert_flag){
  Serial.println(digitalRead(stopSIREN));
  if(digitalRead(stopSIREN)) {
    lcd.setCursor(7, 3);
    lcd.print("Alert>>B-DOWN");
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
    lcd.print("Alert>>B-DOWN");
    delay(350);
    digitalWrite(SIREN, LOW);   
    delay(m);
    digitalWrite(SIREN, HIGH);
    delay(1000);
    lcd.setCursor(7, 3);
    lcd.print("             ");
    }
  B_alert_time = millis();
  B_alert_flag = HIGH;
 }
}

void alertCDown(int m){  //k for string, l for boolean Siren Activate or deactivate, m for delaTime
 if(!C_alert_flag){ 
  Serial.println(digitalRead(stopSIREN));
  if(digitalRead(stopSIREN)) {
    lcd.setCursor(7, 3);
    lcd.print("Alert>>C-DOWN");
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
    lcd.print("Alert>>C-DOWN");
    delay(350);
    digitalWrite(SIREN, LOW);   
    delay(m);
    digitalWrite(SIREN, HIGH);
    delay(1000);
    lcd.setCursor(7, 3);
    lcd.print("             ");
    }
  C_alert_time = millis();
  C_alert_flag = HIGH;
 }
}


void alertSmoke(int m){  //k for string, l for boolean Siren Activate or deactivate, m for delaTime
 if(!Smoke_alert_flag){
  Serial.println(digitalRead(stopSIREN));
  if(digitalRead(stopSIREN)) {
    lcd.setCursor(7, 3);
    lcd.print("Alert>>SMOKE");
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
    lcd.print("Alert>>SMOKE");
    delay(350);
    digitalWrite(SIREN, LOW);   
    delay(m);
    digitalWrite(SIREN, HIGH);
    delay(1000);
    lcd.setCursor(7, 3);
    lcd.print("             ");
    }
  Smoke_alert_time = millis();
  Smoke_alert_flag = HIGH;
 }
}

void alertTemp(int m){  //k for string, l for boolean Siren Activate or deactivate, m for delaTime
 if(!Temp_alert_flag){ 
  Serial.println(digitalRead(stopSIREN));
  if(digitalRead(stopSIREN)) {
    lcd.setCursor(7, 3);
    lcd.print("Alert>>T-RISE");
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
    lcd.print("Alert>>T-RISE");
    delay(350);
    digitalWrite(SIREN, LOW);   
    delay(m);
    digitalWrite(SIREN, HIGH);
    delay(1000);
    lcd.setCursor(7, 3);
    lcd.print("             ");
    }
  Temp_alert_time = millis();
  Temp_alert_flag = HIGH;
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

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
