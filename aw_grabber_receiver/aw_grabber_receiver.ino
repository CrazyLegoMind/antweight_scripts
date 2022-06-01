#include <SPI.h>
#include "RF24.h"

//motori
const int M1dir = 2;//PHASE
const int M1rpm = 3;
const int M2dir = 4;//PHASE
const int M2rpm = 5;
//arma
const int M3dir = 7;//PHASE
const int M3rpm = 6;
const int M3pot = A0;
int WeaponAngle = -1;

bool radioNumber = 1;

RF24 radio(9, 10);

typedef struct {
  int16_t speedmotor1;
  int16_t speedmotor2;
  int16_t speedmotor3;
  int16_t Angle;
  int16_t wpnAccel;
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

const uint64_t READ_ADDR = 0x47E197009488;

//script var
int wpn_prev_pos = 0;
unsigned long wpn_prev_time = 0;
float wpn_prev_speed = 0.0f;
int wpn_prev_pwm = 0;
float avg_speed = 0.0f;
int tot_step = 0;
unsigned long tot_time = 0;
int grad_cnt = 0;
float wpn_drop_speed = 0.0f;
int func[13] = {4, 10, 18, 24, 30, 36, 42, 48, 52, 57, 62, 68, 68};

void setup() {
  //Serial.begin(115200);
  //Serial.println("Serial Ready");
  //esc motori
  pinMode(M1dir, OUTPUT);
  digitalWrite(M1dir, LOW);
  pinMode(M1rpm, OUTPUT);
  digitalWrite(M1rpm, LOW);
  pinMode(M2dir, OUTPUT);
  digitalWrite(M2dir, LOW);
  pinMode(M2rpm, OUTPUT);
  digitalWrite(M2rpm, LOW);

  //esc arma
  pinMode(M3dir, OUTPUT);
  digitalWrite(M3dir, LOW);
  pinMode(M3rpm, OUTPUT);
  digitalWrite(M3rpm, LOW);


  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, READ_ADDR);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    while (radio.available()) {
      radio.read( &sentData, sizeof(sentData) );
      Last_Data_Time = millis();
      Reconnect_Time = millis();
      reconnect = true;
      FailSafe = false;
      setM1speed(sentData.speedmotor1);
      setM2speed(sentData.speedmotor2);
      seek_angle_smooth(sentData.Angle,sentData.wpnAccel);
      delay(10);
    }
  }
  while (!radio.available()){
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
        radio.setPALevel(RF24_PA_LOW);
        radio.openReadingPipe(1, READ_ADDR);
        radio.startListening();
        //Serial.println("reconnected");
        reconnect = false;
      }
      break;
    }
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
    digitalWrite(M1dir, HIGH);
    analogWrite(M1rpm, -rpm);
    return 0;
  }
  digitalWrite(M1dir, LOW);
  analogWrite(M1rpm, rpm);
  return 1;
}

int setM2speed(int rpm) {
  if (rpm < 0) {
    digitalWrite(M2dir, HIGH);
    analogWrite(M2rpm, -rpm);
    return 0;
  }
  digitalWrite(M2dir, LOW);
  analogWrite(M2rpm, rpm);
  return 1;
}

int setM3speed(int rpm) {
  if (rpm < 0) {
    digitalWrite(M3dir, HIGH);
    analogWrite(M3rpm, -rpm);
    return 0;
  }
  digitalWrite(M3dir, LOW);
  analogWrite(M3rpm, rpm);
  return 1;
}

int seek_angle_smooth(int target_pos,int accel) {
  int wpn_current_pos = analogRead(M3pot);
  
  unsigned long wpn_current_time = millis();
  float pos_delta = wpn_current_pos - wpn_prev_pos;
  unsigned long time_delta = wpn_current_time - wpn_prev_time;
  float wpn_speed = pos_delta/time_delta;
  int gap = target_pos - wpn_current_pos;
  int wpn_pwm = 0;
  int accel_treshold = abs(wpn_prev_pwm)+accel;
  int overshoot = 0;
  if (abs(gap) > 20) {
    overshoot = target_pos -(wpn_current_pos+ wpn_speed*14);
    wpn_pwm = overshoot;
  }
  
  if(accel> 0){
    wpn_pwm = constrain(wpn_pwm, -accel_treshold, accel_treshold);
  }
  wpn_pwm = constrain(wpn_pwm, -sentData.speedmotor3, sentData.speedmotor3);
  wpn_pwm = constrain(wpn_pwm, -255, 255);
  
  setM3speed(wpn_pwm);

  //prev sets for next iteration
  wpn_prev_pos = wpn_current_pos;
  wpn_prev_time = wpn_current_time;
  wpn_prev_speed = wpn_speed;
  wpn_prev_pwm = wpn_pwm;
  
  /*
  Serial.print("delta:");
  Serial.print(target_pos-wpn_current_pos);
  Serial.print(",speed:");
  Serial.print(abs(wpn_speed) * 100.0f);
  Serial.print(",pwm:");
  Serial.print(abs(wpn_pwm));
  Serial.print(",ovs:");
  Serial.println(overshoot);
  //*/
  return 0;
}

/*
int seek_angle_smooth(int target_pos,int accel) {
  if (target_pos < 0) return 1;

  unsigned long wpn_current_time = millis();
  int wpn_current_pos = analogRead(M3pot);
  float pos_delta = abs(wpn_current_pos - wpn_prev_pos);
  unsigned long time_delta = wpn_current_time - wpn_prev_time;
  float wpn_speed = pos_delta/time_delta;
  int gap = target_pos - wpn_current_pos;
  int wpn_pwm = 0;
  int safetylimit = sentData.speedmotor3;

  if (abs(gap) > 20) {
    wpn_pwm = gap;
    wpn_pwm = constrain(wpn_pwm, -sentData.speedmotor3, sentData.speedmotor3);
  }
  //int teo_speed = func[(abs(wpn_pwm)-20)/20]*0.1;
  int i = 0;
  if (wpn_speed < wpn_prev_speed) {
    grad_cnt++;
  } else{
    grad_cnt = 0;
  }

  if (grad_cnt > 2) {
    grad_cnt = 3;
    i = (wpn_speed - 400) / 600;
    safetylimit = (i+2)*20;
  }
  safetylimit = constrain(safetylimit, -(abs(wpn_prev_pwm)+accel), abs(wpn_prev_pwm)+accel);
  wpn_prev_pos = wpn_current_pos;
  wpn_prev_time = wpn_current_time;
  wpn_prev_speed = wpn_speed;
  wpn_pwm = constrain(wpn_pwm, -safetylimit, safetylimit);
  wpn_pwm = constrain(wpn_pwm, -sentData.speedmotor3, sentData.speedmotor3);
  wpn_prev_pwm = wpn_pwm;
  setM3speed(wpn_pwm);
    if (gap <= 20)return 1;
    Serial.print(target_pos);
    Serial.print("\t");
    Serial.print(wpn_current_pos);
    Serial.print("\t");
    if (wpn_speed > 1){
    tot_step += pos_delta;
    tot_time += time_delta;
    avg_speed = tot_step/(tot_time/1000.0f);
    }
    Serial.print(avg_speed/1000.0f);
    Serial.print("\t");

  //if(wpn_speed < 350) return 1;
  Serial.print(safetylimit);
  Serial.print("\t");
  Serial.print(target_pos);
  Serial.print("\t");
  Serial.print(wpn_current_pos);
  Serial.print("\t");
  Serial.print(wpn_speed * 100.0f);
  Serial.print("\t");
  Serial.print(abs(wpn_pwm));
  Serial.print("\t");
  Serial.println();
  return 0;
}

int seek_angle(int target){
  if (target <0) return 1;
  int current = analogRead(M3pot);
  int gap = target - current;
  int wpn_pwm = 0;
  if (abs(gap) > 20) {
    wpn_pwm = gap;
    wpn_pwm = constrain(wpn_pwm, -sentData.speedmotor3, sentData.speedmotor3);
  }
  setM3speed(wpn_pwm);
  return 0;
  }
//*/

void failsafe()
{
  setM1speed(0);
  setM2speed(0);
  setM3speed(0);
}
