#include <SPI.h>
#include "RF24.h"

RF24 radio(9, 10);
const uint64_t WRITE_ADDR = 0x47E197009488;

// stick and trims potentiometers input pins
const int weaponPot = A0;
const int steerPot = A1;
const int accPot = A2;
const int RTbtnPin = 2;

//datas that will be sent to the receiver
typedef struct {
  int speedmotorLeft;
  int speedmotorRight;
  int speedmotorWeapon;
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
const int potStrRightStart = 527; //default 512
const int potStrRightEnd = 1023; //default 0, reversed 1024
const int potStrLeftStart = 523; //default 512
const int potStrLeftEnd = 0; //default 1024, reversed 0

const int potAccForwardStart = 503; //default 512
const int potAccForwardEnd = 0; // default 0, reversed 1024
const int potAccBackStart = 507; //default 512
const int potAccBackEnd = 1023; //default 1024, reversed 0

const int potWpnLowStart = 515; //default 512
const int potWpnLowEnd = 965; //default 0, reversed 1024
const int potWpnHighStart = 510; //default 512
const int potWpnHighEnd = 0; //default 1024, reversed 0



int restAngle =  35;
int endAngle = 1023 ;
int startAngle = 0;

//variables for the sketch
int right;
int left;
int forward;
int back;
int tempAngle;
int PWMmax = 255;
int wpnSpeed = PWMmax;
int strPWMmax = PWMmax;
int accPWMmax = PWMmax;
int mode;
int address = 0;
int RTstate = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(RTbtnPin, INPUT);
  
  Serial.begin(115200);
  radio.begin();
  //radio.setChannel(channel);
  radio.setPALevel(RF24_PA_LOW);
  sentData.speedmotorWeapon = 0;
  // Open a writing and reading pipe on each radio, with opposite addresses
  //from the rf24 tutorial, need to be reworked
  radio.openWritingPipe(WRITE_ADDR);

}

void loop() {
  
  int wpnValue = analogRead(weaponPot);
  int strValue = analogRead(steerPot);
  int accValue = analogRead(accPot);
  RTstate = digitalRead(RTbtnPin);
  
  //weapon part first
  int tmpA1 = map(wpnValue, potWpnHighStart, potWpnHighEnd, 0, endAngle-restAngle);
  tmpA1 = constrain(tmpA1, 0, endAngle-restAngle);
  int tmpA2 = map(wpnValue, potWpnLowEnd, potWpnLowStart, startAngle, restAngle);
  tmpA2 = constrain(tmpA2, startAngle, restAngle);
  sentData.Angle = tmpA1+tmpA2;
  if(RTstate){
    sentData.Angle = endAngle;
  }
  //Serial.println(sentData.Angle);
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


  /*
  if (back < 0) //correct the turning direction while go backward but not while pivot turning
  {
    sentData.speedmotorLeft = back - right + left;
    sentData.speedmotorRight = back + right - left;
  }
  else if (forward >= 0) // "else" could be called but "else if" prevent some strange moving if forward is negative due to wrong initialization
  {
    sentData.speedmotorLeft = forward + right - left;
    sentData.speedmotorRight = forward - right + left;
  }
  */
  int M1 = forward + back + right - left;
  int M2 = forward + back - right + left;
  sentData.speedmotorWeapon = constrain(wpnSpeed, 0, PWMmax);
  sentData.weaponAccel = 100;
  //recorrect the data not to have more than 255PWM while non pivot-steering
  sentData.speedmotorRight = constrain(M2, -PWMmax, PWMmax);
  sentData.speedmotorLeft = constrain(M1, -PWMmax, PWMmax);

  if (!radio.write( &sentData, sizeof(sentData) )){
    Serial.println("failed");
  }
  Serial.println(sentData.Angle);
  /*
    Serial.print(wpnValue);
    Serial.print("\t");
    Serial.print(accValue);
    Serial.print("\t");
    Serial.println(strValue);
    Serial.print(sentData.speedmotorWeapon);
  
  Serial.print("Wapon: ");
  Serial.print(sentData.speedmotorWeapon);
  Serial.print("\t");
  Serial.print("Left: ");
  Serial.print(sentData.speedmotorLeft);
  Serial.print("\t");
  Serial.print("Right: ");
  Serial.print(sentData.speedmotorRight);
  Serial.print("RT: ");
  Serial.println(RTstate);
  //*/
  delay(20);
}
