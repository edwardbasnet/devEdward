/*
 * dashingresham@gmail.com
 * newpal22@gmail.com
 * 
 * https://www.bluetin.io/guides/l298n-h-bridge-dc-motor-driver-guide/
 */
#include<Servo.h>
#include<SoftwareSerial.h>
SoftwareSerial mySerial(2,4); //RX, TX

Servo myServo;  //creating servo object

#define L_F 8 //front left motor pin
#define L_B 9 //front right motor pin
#define R_F 10 //back left motor pin
#define R_B 11 //back right motor pin
#define speed_L 3 //left motor's speed
#define speed_R 5 //right motor's speed
#define KICK 6 // ball kick attached servo
volatile int SPEED; //motor speed

int getByte;
void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  myServo.attach(KICK);
  
  pinMode(L_F, OUTPUT);
  pinMode(L_B, OUTPUT);
  pinMode(R_F, OUTPUT);
  pinMode(R_B, OUTPUT);
  pinMode(KICK, OUTPUT);
  pinMode(speed_R, OUTPUT);
  pinMode(speed_L, OUTPUT);
  }

void loop() {
if(mySerial.available() > 0){
  getByte = mySerial.read() - '0';
  switch(getByte){
    case 1:
      motorF();
      Serial.println("f");
      break;
    case 2:
      motorR();
      Serial.println("r");
      break;
    case 3:
      motorL();
      Serial.println("l");
      break;
    case 4:
      motorB();
      Serial.println("b");
      break;
    case 5:
      SPEED = 50;
      Serial.println("set speed: ");
      Serial.println(SPEED);
      break;
    case 6:
      SPEED = 100;
      Serial.print("set speed: ");
      Serial.println(SPEED);
      break;
    case 7:
      SPEED = 255;
      Serial.println("set speed: ");
      Serial.println(SPEED);
      break;
    case 8:
      servo();
      Serial.println("k");
      break;
    case 9:
      motorH();
      Serial.println("h");
      break;
    default:
      motorH();
      break;
    }
  
  }
}

void motorF(){
  analogWrite(speed_L, SPEED);
  analogWrite(speed_R, SPEED);
  digitalWrite(L_F, HIGH);
  digitalWrite(L_B, LOW);
  digitalWrite(R_F, HIGH);
  digitalWrite(R_B, LOW);
  }

void motorL(){
  analogWrite(speed_L, SPEED);
  analogWrite(speed_R, SPEED);
  digitalWrite(L_F, LOW);
  digitalWrite(L_B, HIGH);
  digitalWrite(R_F, HIGH);
  digitalWrite(R_B, LOW);
  }

void motorR(){
  analogWrite(speed_L, SPEED);
  analogWrite(speed_R, SPEED);
  digitalWrite(L_F, HIGH);
  digitalWrite(L_B, LOW);
  digitalWrite(R_F, LOW);
  digitalWrite(R_B, HIGH);
  }

void motorB(){
  analogWrite(speed_L, SPEED);
  analogWrite(speed_R, SPEED);
  digitalWrite(L_F, LOW);
  digitalWrite(L_B, HIGH);
  digitalWrite(R_F, LOW);
  digitalWrite(R_B, HIGH);
  }
void motorH(){
  digitalWrite(L_F, HIGH);
  digitalWrite(L_B, HIGH);
  digitalWrite(R_F, HIGH);
  digitalWrite(R_B, HIGH);
  }
  
void servo(){
  myServo.write(180);
  delay(200);
  myServo.write(0);
  }
