#include <SPI.h>
#include "RF24.h"

//motori
const int M1A = 5;//PHASE
const int M1B = 10;
const int M2A = 9;//PHASE
const int M2B = 6;

//arma
const int M3rpm = 3;
int WeaponAngle = 0;

bool radioNumber = 1;
const uint8_t channel =85;

RF24 radio(7, 8);

typedef struct {
  int speedmotor1;
  int speedmotor2;
  int speedmotor3;
  bool Fire;
  int Angle;
}
A_t;

volatile A_t sentData;
unsigned long Current_Time;
unsigned long Last_Data_Time;
unsigned long FailSafe_Time = 800;
bool FailSafe;


byte addresses[][6] = {"3Node", "4Node"};

void setup() {
  //Serial.begin(9600);
  //Serial.println("Serial Ready");

  //esc motori
  pinMode(M1A, OUTPUT);
  digitalWrite(M1A, LOW);
  pinMode(M1B, OUTPUT);
  digitalWrite(M1B, LOW);
  pinMode(M2A, OUTPUT);
  digitalWrite(M2A, LOW);
  pinMode(M2B, OUTPUT);
  digitalWrite(M2B, LOW);

  //mosfet arma
  pinMode(M3rpm, OUTPUT);
  digitalWrite(M3rpm, LOW);


  radio.begin();
  radio.setChannel(channel);
  radio.setPALevel(RF24_PA_LOW);
  if (radioNumber) {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  } else {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }
  radio.startListening();
}

void loop() {
  
  if (radio.available()) {
    while (radio.available()) {
      radio.read( &sentData, sizeof(sentData) );
      Last_Data_Time = millis();
      FailSafe = false;
    }
    setM1speed(sentData.speedmotor1);
    setM2speed(sentData.speedmotor2);
    setM3speed(sentData.speedmotor3);
  }
  
  while (!radio.available()){
    Current_Time = millis();
    if (Current_Time - Last_Data_Time > FailSafe_Time){
      FailSafe = true;
      break;
    }
  }
  
  if (FailSafe){
    failsafe();
  }

  /*
    Serial.print("FailSafe: ");
    Serial.print(FailSafe);
    //Serial.print("\t millis: ");
    //time = millis();
    //Serial.print(time);
    Serial.print("\t radio: ");
    Serial.print(radio.available());

    Serial.print("PWM1: ");
    Serial.print(sentData.speedmotor1);
    Serial.print("PWM2: ");
    Serial.println(sentData.speedmotor2);
  //*/

}

int setM1speed(int rpm) {
  if (rpm < 0) {
    digitalWrite(M1A, LOW);
    analogWrite(M1B, -rpm);
  } else if (rpm > 0) {
    digitalWrite(M1B, LOW);
    analogWrite(M1A, rpm);
  } else if (rpm == 0) {
    digitalWrite(M1B, HIGH);
    digitalWrite(M1A, HIGH);
  }
  return 0;
}

int setM2speed(int rpm) {
  if (rpm < 0) {
    digitalWrite(M2A, LOW);
    analogWrite(M2B, -rpm);
  } else if (rpm > 0) {
    digitalWrite(M2B, LOW);
    analogWrite(M2A, rpm);
  } else if (rpm == 0) {
    digitalWrite(M2B, HIGH);
    digitalWrite(M2A, HIGH);
  }
  return 0;
}

int setM3speed(int rpm) {
  if (rpm < 0) return 0;
  if (rpm > 240) {
    digitalWrite(M3rpm, HIGH);
    return 0;
  }
  if (rpm == 0) {
    digitalWrite(M3rpm, LOW);
    return 0;
  }
  analogWrite(M3rpm, rpm);
  return 1;
}

void failsafe(){
  setM1speed(0);
  setM2speed(0);
  setM3speed(0);
}
