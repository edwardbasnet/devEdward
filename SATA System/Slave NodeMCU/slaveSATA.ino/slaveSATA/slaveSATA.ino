/*
 * MeroTV automation Firmware V2.0: slaveSATA.ino
 * June 2017
 * (c)edward
 */
 //Libraries needed for esp8266
 #include <ESP8266WiFi.h>
 
 //library include for temperature DS18B20
 #include <OneWire.h>
 #include <DallasTemperature.h>

 // Data wire is plugged into pin 2 on the NodeMCU 
 #define ONE_WIRE_BUS 2 
 
 // Setup a oneWire instance to communicate with any 'OneWire devices'  
 OneWire oneWire(ONE_WIRE_BUS);

 // Pass oneWire reference to Dallas Temperature. 
 DallasTemperature sensors(&oneWire);
 
 //Defining I/O pins

 //wifi name and password 
 const char* ssid = "SATAServer";
 const char* password = "password@SATA";

 int SATAServerPort = 9001;
 IPAddress SATAServer(192,168,4,1);
 WiFiClient SATAClient;
 String dataStream;
 float temperature;
 int L, C, S;    // l=LPG gas, C=CO gas, S=Smoke

 /************************Hardware Related Macros************************************/
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
/***********************************************************************************************************************************/

 void setup() {
  
  Serial.begin(9600);
  pinMode(MQ_PIN, INPUT);
  
/******************** Connecting to WiFi Access point ************************/
  //stop previous wifi
  if(WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(50);
  }
  //to avoid broadcasting an ssid
  WiFi.mode(WIFI_STA);
  
  //starting wifi connection  
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.println("waiting for connection ...");
    }Serial.println("WiFi connected!");

  Serial.print("Device IP : "); Serial.println(WiFi.localIP());
  Serial.print("Device MAC Address : "); Serial.println(String(WiFi.macAddress()));
/*****************************************************************************/ 
  
  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

  //temperature sensor initialization
  sensors.begin();

  }

 void loop() {
  //temperature readings
  sensors.requestTemperatures(); 
  temperature = sensors.getTempCByIndex(0); //for more than one DS18B20 on the same bus value of argument is increased
  int temp = temperature * 1000;  //encoding
  //Serial.println(temp);
  
  //smoke sensor reading
  L = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  C = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  S = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  //Serial.println(S);

  dataStream = String("#") + String(temp) + String(",") + String(L) + String(",") + String(C) + String(",") + String(S) + String("!");
  Serial.print("collected data : ");
  Serial.println(dataStream);
  if(sendInfo(dataStream)) {
    Serial.println("data send succesfully");
    }else SATAClient.connect(SATAServer, SATAServerPort);
  }


//Function made to send data to server
 boolean sendInfo(String info) {
  //make sure to disconnect
//  SATAClient.stop();

  if(SATAClient.connect(SATAServer, SATAServerPort)) {
    SATAClient.println(info);
    Serial.print("sending... ");
    Serial.print(info);
    Serial.println(" ...done");
    return true;
    }else {
      Serial.println("retry connecting");
      SATAClient.flush();
      return false;
    }
  }


/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc) {
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
float MQCalibration(int mq_pin) {
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
float MQRead(int mq_pin) {
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
int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
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
int  MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
