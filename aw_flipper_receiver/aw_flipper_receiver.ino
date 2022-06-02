#include <SPI.h>
#include "RF24.h"

//motori
const int M1A = 5;//PHASE
const int M1B = 6;
const int M2A = 9;//PHASE
const int M2B = 10;

//arma
const int M3rpm = 3;
int WeaponAngle = 0;

const int M3stop = 4;
bool loaded = false;
bool failsafe_released = false;

//time based motor stop
unsigned long overloadCurrentTime = 0;
unsigned long activeStartTime = 0;
unsigned long activeMaxMillis = 1000;
unsigned long cooldownMillis = 1500;
bool activeInitMillis = false;
bool overloaded = false;

typedef struct {
  int16_t speedmotorLeft;
  int16_t speedmotorRight;
  int16_t weaponStrenght;
  int16_t weaponArg;
  int16_t weaponAccel;
  int8_t Fire;
}
A_t;

volatile A_t sentData;
unsigned long Reconnect_Time;
unsigned long Current_Time;
unsigned long Last_Data_Time;
unsigned long FailSafe_Time = 200;
unsigned long Recconect_Delay_Time = 190;
bool FailSafe;
bool reconnect = true;

//RADIO

const uint8_t channel = 85;
const uint64_t READ_ADDR = 0xF71993500B07;

void setup() {
  //Serial.begin(115200);
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
  pinMode(M3stop, INPUT_PULLUP);

  radio.begin();
  radio.setChannel(channel);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, READ_ADDR);
  radio.startListening();
}

void loop() {

  loaded = !digitalRead(M3stop);

  if (radio.available()) {
    while (radio.available()) {
      radio.read( &sentData, sizeof(sentData) );
      Last_Data_Time = millis();
      Reconnect_Time = millis();
      reconnect = true;
      FailSafe = false;
      failsafe_released = false;
      setM1speed(sentData.speedmotorLeft);
      setM2speed(sentData.speedmotorRight);
      handle_flipper(sentData.Fire);
    }

  }

  while (!radio.available()) {
    Current_Time = millis();
    if(!reconnect){
      Reconnect_Time = Current_Time;
      reconnect = true;
    }
    if (Current_Time - Last_Data_Time >= FailSafe_Time) {
      failsafe();
      //Serial.println("f");
      if (Current_Time - Reconnect_Time >= Recconect_Delay_Time) {
        radio.stopListening();
        radio.begin();
        radio.setChannel(channel);
        radio.setPALevel(RF24_PA_LOW);
        radio.openReadingPipe(1, READ_ADDR);
        radio.startListening();
        //Serial.println("reconnected");
        reconnect = false;
      }

      break;
    }
  }

  //DEBUG RECEIVED
  /*
    Serial.print("LPWM: ");
    Serial.print(sentData.speedmotorLeft);
    Serial.print("\t");
    Serial.print("R PWM: ");
    Serial.print(sentData.speedmotorRight);
    Serial.print("\t");
    Serial.print("WPN: ");
    Serial.print(sentData.weaponArg);
    Serial.print("\t");
    Serial.print("FIRE: ");
    Serial.println((int)sentData.Fire);
    //*/
  //DEBUG COOLDOWN
  ///*

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



void handle_flipper(bool fire) {
  //controlla se devo sparare
  overloadCurrentTime = millis();
  bool activate = false;
  if (loaded) {
    if (fire) {
      activate = true;
    }
  } else {
    activate = true;
  }

  //azzera il cooldown se non devi sparare
  //spara solo se sei entro il cooldown
  //e segnati quando comincia sparare
  if (activate) {//active
    if (!overloaded) {//not overloadeded
      if (!activeInitMillis) {
        activeStartTime = overloadCurrentTime;
        activeInitMillis  = true;
      }

      if (overloadCurrentTime - activeStartTime >= activeMaxMillis) {
        overloaded = true;
        setM3speed(0);
      } else {
        setM3speed(255);
      }
    } else { //overloaded
      setM3speed(0);
      if (overloadCurrentTime - activeStartTime >= activeMaxMillis + cooldownMillis) {
        overloaded = false;
        activeInitMillis  = false;
      }

    }
  } else {//inactive
    overloaded = false;
    activeInitMillis  = false;
    setM3speed(0);
  }
  /*
    Serial.print(Current_Time - activeStartTime >= activeMaxMillis);
    Serial.print(Current_Time - activeStartTime >= activeMaxMillis + cooldownMillis);
    Serial.print(" activate: ");
    Serial.print(activate);
    Serial.print("loaded: ");
    Serial.print(loaded);
    Serial.print("\t");
    Serial.print("over: ");
    Serial.print(overloaded);
    Serial.print("\t");
    Serial.print("start: ");
    Serial.print(activeStartTime);
    Serial.print("\t");
    Serial.print("elapsed: ");
    Serial.println(Current_Time - activeStartTime);
    //*/
}


void failsafe() {
  //Serial.println("failsafing");
  setM1speed(0);
  setM2speed(0);
  if (!loaded) {
    failsafe_released = true;
  }
  if (!failsafe_released) {
    handle_flipper(true);
  } else {
    setM3speed(0);
  }
}
