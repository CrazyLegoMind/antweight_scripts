#include <SPI.h>
#include "RF24.h"
#include <EEPROM.h>
#include <math.h>
bool radioNumber = 0;

RF24 radio(7, 8);


const uint64_t WRITE_ADDR = 0x47E197009488;

// stick and trims potentiometers input pins
const int steerPot = A1;
const int accPot = A2;
const int thumbBtn = 4;
const int TrimDown = A3;
const int TrimUp = A0;
const int button1 = 3;
const int button2 = 9;
const int modePot = A4;


//datas that will be sent to the receiver
typedef struct {
  int speedmotorLeft;
  int speedmotorRight;
  int weaponStrenght;
  bool Fire;
  int Angle;
  int weaponAccel;
}
A_t;


A_t sentData;



//various stick/potentiometers limits
//this is useful if:

//tx potentiometers don't travel from 1023 to 0
//-if their center positon is not 512
//-their center position has some play so it is a range of values
// instead of a single vaue

const int potStrRightEnd = 100; //default 0, reversed 1024
const int potStrLeftEnd = 1008; //default 1024, reversed 0

const int potAccForwardEnd = 71; // default 0, reversed 1024
const int potAccBackEnd = 541; //default 1024, reversed 0


const int potStrRightStart = 578; //default 512
const int potStrLeftStart = 590; //default 512

const int potAccForwardStart = 338; //default 512
const int potAccBackStart = 360; //default 512


//var that will be stored and retreived from
//eprom mem
int restAngle= 0;
int activeAngle= 0;
int strExpoalpha = 0;
int accExpoalpha = 0;
int wpnAccel = 0;


//customisable vars
int PWMmax = 255;
int expoMax = 190; //exp will be this/100

//variables for the sketch
int right= 0;
int left= 0;
int forward= 0;
int back= 0;
int strPWMmax = PWMmax;
int accPWMmax = PWMmax;
int modeValue= 0;
int address = 0;
bool memSetted = true;
bool wpnSafetyCeck = false;

void setup() {
  pinMode(thumbBtn, INPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);

  //Serial.begin(115200);

  //LOADING FROM MEM
  //active angle for the weapon
  int activeAng_low = (int) EEPROM.read(address);
  delay(5);
  int activeAng_high = (int) EEPROM.read(address + 1);
  delay(5);
  activeAngle = activeAng_high << 8 | activeAng_low ;
  //exponential coefficients for drive
  strExpoalpha = (int) EEPROM.read(address + 2);
  delay(5);
  accExpoalpha = (int) EEPROM.read(address + 3);
  delay(5);
  //active angle for the weapon
  int restAng_low = (int) EEPROM.read(address + 4);
  delay(5);
  int restAng_high = (int) EEPROM.read(address + 5);
  delay(5);
  restAngle = restAng_high << 8 | restAng_low ;
  //weapon acceleration coefficient
  wpnAccel = (int) EEPROM.read(address+6);
  
  Serial.println("mem READ");
  Serial.print("angle_act: ");
  Serial.print(activeAngle);
  Serial.print(" -- angle_rest: ");
  Serial.print(restAngle);
  Serial.print(" -- wpn_acc: ");
  Serial.print(wpnAccel);
  Serial.print(" -- str_exp: ");
  Serial.print(strExpoalpha);
  Serial.print(" -- acc_exp: ");
  Serial.println(accExpoalpha);
  sentData.weaponStrenght = 0;
  // Open a writing and reading pipe on each radio, with opposite addresses
  //from the rf24 tutorial, need to be reworked
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(WRITE_ADDR);

}



void loop() {
  //read pots values
  int strValue = analogRead(steerPot);
  int accValue = analogRead(accPot);
  int TrimDownValue = analogRead(TrimDown);
  int TrimUpValue = analogRead(TrimUp);
  modeValue = analogRead(modePot);

  //read btns values
  bool setMode = digitalRead(button1);
  bool storeValue = digitalRead(button2);
  bool thumbValue = digitalRead(thumbBtn);


  bool drSet = false;
  bool expoSet = false;
  bool safetySet = false;
  bool wpnSet = false;
  bool wpn2Set = false;

  int mode_n = modeValue / 205;

  if (setMode) {
    switch (mode_n) {
      case 0:
        drSet = true;
        break;
      case 1:
        expoSet = true;
        break;
      case 2:
        safetySet = true;
        break;
      case 3:
        wpnSet = true;
        break;
      case 4:
        wpn2Set = true;
        break;
      default:
        break;
    }
  }

  //----------------------------------------------------WEAPON CODE
  if (safetySet) {
    if(TrimUpValue <= 20) wpnSafetyCeck = true;
    if(wpnSafetyCeck){
      sentData.weaponStrenght = map(TrimUpValue, 0, 1010, 0, PWMmax);
    }
  }

  if (wpnSet) {
    activeAngle = analogRead(TrimUp);
    restAngle = analogRead(TrimDown);
  }
  if (wpn2Set) {
    sentData.weaponStrenght = map(TrimUpValue, 0, 1010, 0, PWMmax);
    wpnAccel = map(TrimDownValue, 0, 1022, 0, 100);
  }

  if (thumbValue) {
    sentData.Angle = activeAngle;
  } else {
    sentData.Angle = restAngle;
  }
  
  sentData.weaponAccel = wpnAccel;


  //----------------------------------------------------MOTOR CODE
  //setting dual rates

  if (drSet) {
    strPWMmax = map(TrimUpValue, 0, 1010, 0, PWMmax);
    accPWMmax = map(TrimDownValue, 0, 1010, 0, PWMmax);
  }

  //map the value to useful pwm-friendly ones
  right = map(strValue, potStrRightStart, potStrRightEnd, 0, strPWMmax);
  left = map(strValue, potStrLeftStart, potStrLeftEnd, 0, strPWMmax);
  forward = map(accValue, potAccForwardStart, potAccForwardEnd, 0, accPWMmax);
  back = map(accValue, potAccBackStart, potAccBackEnd, 0, -accPWMmax);

  //correct the values in case of wrong pot limits on initialization and
  //so all of them will have the same weight in next computations
  right = constrain(right, 0, strPWMmax);
  left = constrain(left, 0, strPWMmax);
  forward = constrain(forward, 0, accPWMmax);
  back = constrain(back, -accPWMmax, 0);

  //exponential curve setup
  bool strExpo = false;
  bool accExpo = false;
  if (expoSet) {
    strExpoalpha = map(TrimUpValue, 10, 1010, 100, expoMax);
    accExpoalpha = map(TrimDownValue, 10, 1010, 100, expoMax);
  }
  if (strExpoalpha > 100) {
    strExpo = true;
  }
  if (accExpoalpha > 100) {
    accExpo = true;
  }

  if (strExpo) {
    int leftx = map(left, 0, strPWMmax, 0, 1000);
    left = pow(leftx / 1000.0f, strExpoalpha / 100.0f) * strPWMmax;

    int rightx = map(right, 0, strPWMmax, 0, 1000);
    right = pow(rightx / 1000.0f, strExpoalpha / 100.0f) * strPWMmax;
  }
  if (accExpo) {
    int forwardx = map(forward, 0, accPWMmax, 0, 1000);
    forward = pow(forwardx / 1000.0f, accExpoalpha / 100.0f) * accPWMmax;

    int backx = map(-back, 0, accPWMmax, 0, 1000);
    back = -pow(backx / 1000.0f, accExpoalpha / 100.0f) * accPWMmax;
  }

  /* comment here if you want car-like steering direction
    if (back < 0){ //correct the turning direction while go backward but not while pivot turning
    sentData.speedmotorLeft = back - right + left;
    sentData.speedmotorRight = back + right - left;
    }else if (forward >= 0) { // "else" could be called but "else if" prevent some strange moving if forward is negative due to wrong initialization
    sentData.speedmotorLeft = forward  + right - left;
    sentData.speedmotorRight = forward - right + left;
    }
    //*/

  ///* uncomment here if you want car-like steering direction
  sentData.speedmotorLeft = forward + back + right - left;
  sentData.speedmotorRight = forward + back - right + left;
  //*/

  //recorrect the data not to have more than max +PWM while non pivot-steering
  sentData.speedmotorLeft = constrain(sentData.speedmotorLeft, -PWMmax, PWMmax);
  sentData.speedmotorRight = constrain(sentData.speedmotorRight, -PWMmax, PWMmax);


  if (!storeValue) {
    memSetted = false;
  } else {
    if (!memSetted) {
      memSetted = true;
      EEPROM.update(address, activeAngle);
      delay(5);
      EEPROM.update(address + 1, activeAngle >> 8);
      delay(5);

      EEPROM.update(address + 2, strExpoalpha);
      delay(5);
      EEPROM.update(address + 3, accExpoalpha);
      delay(5);

      EEPROM.update(address + 4, restAngle);
      delay(5);
      EEPROM.update(address + 5, restAngle >> 8);
      delay(5);

      EEPROM.update(address + 6, wpnAccel);
      delay(5);
      
      Serial.println("mem SETTED!");
    }
  }

  if (!radio.write( &sentData, sizeof(sentData) )) {
    //Serial.println(F("failed"));
  }

  /*
    Serial.print("B1 (modeValue): ");
    Serial.print(setMode);
    Serial.print("\t");
    Serial.print("B2 (mem): ");
    Serial.println(storeValue);

    Serial.print("Left motor PWM: ");
    Serial.print(sentData.speedmotorLeft);
    Serial.print("\t");
    Serial.print("Right motor PWM: ");
    Serial.println(sentData.speedmotorRight);

    Serial.print(accValue);
    Serial.print("\t");
    Serial.println(strValue);


    Serial.print("Up: ");
    Serial.print(TrimUpValue);
    Serial.print("\t");
    Serial.print("Down: ");
    Serial.println(TrimDownValue);

    Serial.println(sentData.Angle);

    Serial.print("B1: ");
    Serial.print(setMode);
    Serial.print("\t");
    Serial.print("B2: ");
    Serial.print(storeValue);
    Serial.print("\t");
    Serial.print("modePot: ");
    Serial.println(modeValue);
    //*/
}
