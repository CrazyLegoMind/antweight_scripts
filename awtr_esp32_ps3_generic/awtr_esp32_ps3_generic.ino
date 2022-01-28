#include <Ps3Controller.h>


const int potStrRightStart = 2; //default 0
const int potStrRightEnd = 128; //default 128, reversed 


const int potStrLeftStart = -2; //default 0
const int potStrLeftEnd = -127; //default -127, reversed 0

const int potAccForwardStart = -2; //default 0
const int potAccForwardEnd = -127; // default -127, reversed 1024

const int potAccBackStart = 2; //default 0
const int potAccBackEnd = 128; //default 128, reversed 0

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



//customisable vars
const int PWMmax = 255;
int expoMax = 190; //exp will be this/100

//variables for the sketch
int right = 0;
int left = 0;
int forward = 0;
int back = 0;
int strPWMmax = 255;
int accPWMmax = 255;
int strValue = 0;
int accValue  = 0;

void setup(){
  Serial.begin(115200);
  Ps3.begin("00:12:34:56:78:9b");
  Serial.println("Ready.");
}

void loop(){
  if (Ps3.isConnected()) {
    strValue = Ps3.data.analog.stick.rx;
    accValue = Ps3.data.analog.stick.ry;
    //Serial.print(Ps3.data.analog.stick.rx, DEC);
    //Serial.print("\t");
    //Serial.println(Ps3.data.analog.stick.ry, DEC);
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

  sentData.speedmotorLeft = forward + back + right - left;
  sentData.speedmotorRight = forward + back - right + left;

  sentData.speedmotorLeft = constrain(sentData.speedmotorLeft, -PWMmax, PWMmax);
  sentData.speedmotorRight = constrain(sentData.speedmotorRight, -PWMmax, PWMmax);

  Serial.print(sentData.speedmotorLeft);
  Serial.print("\t");
  Serial.println(sentData.speedmotorRight);

}
