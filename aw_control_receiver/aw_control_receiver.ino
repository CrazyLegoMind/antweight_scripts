#include <SPI.h>
#include "RF24.h"
#include <PWMServo.h>

//motori
const int AIN1 = A1;//PHASE
const int AIN2 = A0;//PHASE
const int PWMA = 6;
const int BIN1 = A3;//PHASE
const int BIN2 = A2;//PHASE
const int PWMB = 5;

//arma
int servoPin = 10;
PWMServo servoarma;

bool reverseM1 = false;
bool reverseM2 = false;

bool radioNumber = 1;
const uint64_t READ_ADDR = 0xe197c02702;//0xe197c02702


RF24 radio(7, 8);

typedef struct {
  int speedmotorLeft;
  int speedmotorRight;
  int speedmotorWeapon;
  bool Fire;
  int Angle;
}
A_t;

A_t sentData;
unsigned long current_time;
unsigned long Last_Data_Time;
unsigned long FailSafe_Time = 200;
bool FailSafe;
int PWM;

void setup() {

  //esc motori
  //Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  digitalWrite(AIN1, LOW);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN2, LOW);
  pinMode(PWMA, OUTPUT);
  digitalWrite(PWMA, LOW);


  pinMode(BIN1, OUTPUT);
  digitalWrite(BIN1, LOW);
  pinMode(BIN2, OUTPUT);
  digitalWrite(BIN2, LOW);
  pinMode(PWMB, OUTPUT);
  digitalWrite(PWMB, LOW);
  //esc arma
  servoarma.attach(servoPin);
  sentData.Angle = 90;
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, READ_ADDR);
  radio.startListening();
}

void loop() {
  if (radio.available()){
    //Serial.print("connected \t");
    while (radio.available()){
      radio.read( &sentData, sizeof(sentData) );
      Last_Data_Time = millis();
      FailSafe = false;
    }
    setMAspeed(sentData.speedmotorLeft);
    setMBspeed(sentData.speedmotorRight);
    servoarma.write(sentData.Angle);

  }else{
    //Serial.print("not connected \t");
  }
  
  if (!radio.available()){
    current_time = millis();
    Serial.print(current_time - Last_Data_Time);
    Serial.print(" ");
    if (current_time - Last_Data_Time > FailSafe_Time){
      FailSafe = true;
    }
  }
  
  if (FailSafe){
    Serial.print("\t Failsafe");
    failsafe();
  }

  /*
    Serial.print("FailSafe: ");
    Serial.print(FailSafe);
    Serial.print("\t millis: ");
    time = millis();
    Serial.print(time);
    Serial.print("\t radio: ");
    Serial.print(radio.available());
    Serial.println(sentData.speedmotor1);

    Serial.print("in1: ");
    Serial.print(sentData.speedmotor1);
    Serial.print("in12: ");
    Serial.println(sentData.speedmotor2);
    //*/
  Serial.println();
  delay(10);
}

void setMAspeed(int pwm1) {
  if (pwm1 == 0) {
    analogWrite(PWMA, pwm1);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    return;
  }
  if (pwm1 < 0) {
    analogWrite(PWMA, -pwm1);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    return;
  }
  if (pwm1 > 0) {
    analogWrite(PWMA, pwm1);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
}

void setMBspeed(int pwm2) {
  if (pwm2 == 0) {
    analogWrite(PWMB, pwm2);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    return;
  }
  if (pwm2 < 0) {
    analogWrite(PWMB, -pwm2);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    return;
  }
  if (pwm2 > 0) {
    analogWrite(PWMB, pwm2);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
}

void failsafe() {
  setMAspeed(0);
  setMBspeed(0);
  servoarma.write(90);
}
