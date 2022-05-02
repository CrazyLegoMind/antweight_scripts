#include <SPI.h>
#include "RF24.h"
#include <EEPROM.h>
#include <math.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

//------------ turn on generic serial printing
//#define DEBUG_PRINTS

//------------ specific cases of debug prints
//#define DEBUG_FAILS
//#define DEBUG_SENT
//#define DEBUG_READS
//#define DEBUG_EXPO
//#define DEBUG_NOISE

//------------ chose the hardware input pins model
#define CUSTOM_PIN_LAYOUT


//datas that will be sent to the receiver
typedef struct {
  int16_t speedmotorLeft;
  int16_t speedmotorRight;
  int16_t weaponStrenght;
  int16_t weaponArg;
  int16_t weaponAccel;
  int8_t Fire;
}
packet_t;


packet_t sentData;
packet_t recData;


//---------------------------------------RF24 Variables
RF24 radio(22, 21);
const uint64_t WRITE_ADDR_GRAB = 0x47E197009488;
const uint64_t WRITE_ADDR_FLIP = 0xF71993500B07;
const uint64_t WRITE_ADDR_WEDG = 0xe197c02702;
uint64_t current_addr = WRITE_ADDR_GRAB;
int rf_channel = 76;

//---------------------------------------ESP_NOW Variables
//MAC robot hinge  C8:C9:A3:CB:33:F8
//MAC robot grab    C8:C9:A3:CB:70:54
uint8_t hingeAddress[] = {0xC8, 0xC9, 0xA3, 0xCB, 0x33, 0xF8};
uint8_t egrabAddress[] = {0xC8, 0xC9, 0xA3, 0xCB, 0x70, 0x54};
String success;

esp_now_peer_info_t peerInfo;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&recData, incomingData, sizeof(recData));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  //salva dati in var globali
}




//---------------------------------------HARDWARE DEPENDANT Variables
// one ifdef case per hardware to speed up modularity of the code

#ifdef CUSTOM_PIN_LAYOUT //GRAY REMOTE

// stick and trims potentiometers input pins
const int steerPot = 33;
const int accPot = 35;
const int leverPot = 34;
const int trimPot = 39;

const int leftBtn = 25;
const int rightBtn = 32;
const int topBtn = 12;
const int lowSwitch = 4;
const int topSwitch = 0;
const int ledPin = 2;

//various stick/potentiometers limits
//this is useful if:

//tx potentiometers don't travel from 1023 to 0
//-if their center positon is not 512
//-their center position has some play so it is a range of values
// instead of a single value or their end position has ome play

const int potStrRightEnd = 1020; //default 0, reversed 1024
const int potStrLeftEnd = 3; //default 1024, reversed 0

const int potAccForwardEnd = 3; // default 0, reversed 1024
const int potAccBackEnd = 1020; //default 1024, reversed 0

const int potStrRightStart = 463; //default 512
const int potStrLeftStart = 444; //default 512

const int potAccForwardStart = 479; //default 512
const int potAccBackStart = 502; //default 512

const int potLevUpStart = 456; //default 512
const int potLevDownStart = 481; //default 512

const int potLevUpEnd = 280; //default 0, reversed 1024
const int potLevDownEnd = 644; //default 1024, reversed 0
#else // GREEN and DEV REMOTE
//standard remote
const int steerPot = 35;
const int accPot = 33;
const int leverPot = 34;
const int trimPot = 39;

const int rightBtn = 0;
const int leftBtn = 12;
const int topBtn = 4;
const int lowSwitch = 32;
const int topSwitch = 25;
const int ledPin = 2;

const int potStrRightEnd = 3; //default 0, reversed 1024
const int potStrLeftEnd = 1020; //default 1024, reversed 0

const int potAccForwardEnd = 1020; // default 0, reversed 1024
const int potAccBackEnd = 3; //default 1024, reversed 0

const int potStrRightStart = 461; //default 512
const int potStrLeftStart = 494; //default 512

const int potAccForwardStart = 475; //default 512
const int potAccBackStart = 446; //default 512

const int potLevUpStart = 460; //default 512
const int potLevDownStart = 485; //default 512

const int potLevUpEnd = 284; //default 0, reversed 1024
const int potLevDownEnd = 645; //default 1024, reversed 0
#endif


//customisable vars
int PWMmax = 255;
int expoMax = 210; //exp will be this/100
int analogReadMax = 1023;
int analogRes = 10;

//var that will be stored and retreived from
//eprom mem
int wpn_start = 0;
int wpn_default = 20;
int wpn_end = 1023;
int wpn_range = 1023;

int strExpoalpha = 100;
int accExpoalpha = 100;
int wpnAccel = 8;

//variables for the sketch
int right = 0;
int left = 0;
int forward = 0;
int back = 0;
int wpn = 0;
int strPWMmax = PWMmax;
int accPWMmax = PWMmax;
int trimValue = 0;
int address = 0;
bool memSetted = true;
int rev_str = false;
int debug_min = 1023;
int debug_max = 0;
float leverValue_f = 0.0;
int leverValue = 0;

enum bot_control {
  GRAB,
  FLIP,
  WEDG,
  HING,
  EGRAB
};

int current_bot = EGRAB;
bool wpnSafetyCeck = true;
bool wifi_remote = false;
bool firing = false;
unsigned long current_time = 0;
unsigned long animation_millis_1 = 0;
unsigned long animation_millis_2 = 0;
bool topHold = false;
bool leverMode = false;
int temp_value_debug = 0;
int temp_curve_debug = 0;

void setup() {
  //store_values(); // uncomment only to initialize mem
  analogSetWidth(analogRes);
  analogReadResolution(analogRes);
  analogSetAttenuation(ADC_11db);
  pinMode(rightBtn, INPUT_PULLUP);
  pinMode(leftBtn, INPUT_PULLUP);
  pinMode(topBtn, INPUT_PULLUP);
  pinMode(lowSwitch, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
#ifdef DEBUG_PRINTS
  Serial.begin(115200);
#endif
  //load_values()




  //TODO robot select on initalization
  int robot_n = analogRead(trimPot) / 205;

  switch (robot_n) {
    case 0:
      Serial.println("HINGE BOT");
      wifi_remote = true;
      current_bot = HING;
      wpn_range = 180;
      wpn_end = 10;
      wpn_default = 150;
      wpn_start = wpn_default;
      break;
    case 1:
      Serial.println("FLIPPER BOT");
      current_bot = FLIP;
      current_addr = WRITE_ADDR_FLIP;
      wpn_range = 0;
      rf_channel = 85;
      wpn_end = 0;
      wpn_default = 0;
      wpn_start = wpn_default;
      break;
    case 2:
      Serial.println("WEDGE BOT");
      current_bot = WEDG;
      current_addr = WRITE_ADDR_WEDG;
      wpn_range = 180;
      wpn_end = 180;
      wpn_default = 83;
      wpn_start = 30;
      break;
    case 3:
      Serial.println("EGRAB BOT");
      wifi_remote = true;
      current_bot = EGRAB;
      wpn_range = 1023;
      wpn_end = 1023;
      wpn_default = 100;
      wpn_start = wpn_default;
      break;
    case 4:
      Serial.println("ARDU GRAB BOT");
      current_bot = GRAB;
      current_addr = WRITE_ADDR_GRAB;
      wpn_start = wpn_default;
      break;
    default:
      break;
  }

  //---------------------------------------ESP NOW setup
  if (wifi_remote) {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
#ifdef DEBUG_FAILS
      Serial.println("Error initializing ESP-NOW");
#endif
      return;
    }
    esp_now_register_send_cb(OnDataSent);
    if (current_bot == HING) {
      memcpy(peerInfo.peer_addr, hingeAddress, 6);
      peerInfo.channel = 0;
    } else {
      memcpy(peerInfo.peer_addr, egrabAddress, 6);
      esp_wifi_set_channel(10, WIFI_SECOND_CHAN_NONE);
      peerInfo.channel = 10;
    }
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#ifdef DEBUG_FAILS
      Serial.println("Failed to add peer");
#endif
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    //---------------------------------------RF24 Setup
  } else {
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.openWritingPipe(current_addr);
    radio.setChannel(rf_channel);
  }
}



void loop() {
  //read pots values
  int strValue = analogRead(steerPot);
  delay(3);
  int accValue = analogRead(accPot);
  delay(3);
  int templeverValue = analogRead(leverPot);
  delay(3);
  trimValue = analogRead(trimPot);
  delay(3);
  current_time = millis();
  //leverValue_f = leverValue * 0.8 +  templeverValue * 0.2;
  //leverValue = (int)leverValue_f;
  leverValue = templeverValue;



  //*/
  //read btns values
  bool setMode = false;//!digitalRead(lowSwitch);
  bool storeValue = false;//!digitalRead(topSwitch);
  bool rightValue = !digitalRead(rightBtn);
  bool leftValue = !digitalRead(leftBtn);
  bool topValue = !digitalRead(topBtn);


  bool drSet = !digitalRead(topSwitch);
  bool expoSet = !digitalRead(lowSwitch);
  bool safetySet = false;
  bool wpnSet = false;
  bool mode5Set = false;
  
  if (topValue && !topHold && !drSet && !expoSet) {
    topHold = true;
    leverMode = !leverMode;
  }
  if (!topValue) {
    topHold = false;
  }
  


  //----------------------------------------------------WEAPON CODE
  sentData.weaponArg = wpn_default;
  sentData.Fire = false;



  if (leverMode) {
    wpn = map(leverValue, potLevUpEnd, potLevDownEnd, wpn_end, wpn_start);
    //Serial.print(wpn);
    //Serial.print(" ");
    wpn = constrain(wpn,
                    wpn_start < wpn_end ? wpn_start : wpn_end ,
                    wpn_start < wpn_end ? wpn_end : wpn_start );
    //Serial.println(wpn);
    sentData.weaponArg = wpn;
  }

  if (leftValue) {
    sentData.weaponArg = wpn_start;
  }
  if (rightValue) {
    sentData.weaponArg = wpn_end;
    sentData.Fire = true;
  }

  sentData.weaponStrenght = PWMmax;
  sentData.weaponAccel = 8;

  if (safetySet) {
    //FIX MSSING POT
    /*
      if(TrimUpValue <= 20) wpnSafetyCeck = true;
      if(wpnSafetyCeck){
      sentData.weaponStrenght = map(TrimUpValue, 0, 1022, 0, PWMmax);
      }
    */
  }

  if (wpnSet) {
    wpn_default = map(trimValue, 0, 1023, 0 ,wpn_range);
  }






  //----------------------------------------------------MOTOR CODE
  //setting dual rates

  if (drSet) {
    if(topValue){
      accPWMmax = map(trimValue, 0, 1022, 0, PWMmax);
    }else{
      strPWMmax = map(trimValue, 0, 1022, 0, PWMmax);
    }
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
    //FIX MSSING POT
    if(topValue){
      accExpoalpha = map(trimValue, 10, 1022, 100, expoMax);
    }else{
      strExpoalpha = map(trimValue, 10, 1022, 100, expoMax);
    }
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

  //recorrect the data not to have more than max +PWM while non pivot-steering
  int left_pwm = constrain(forward + back + right - left, -PWMmax, PWMmax);
  int right_pwm = constrain(forward + back - right + left, -PWMmax, PWMmax);

  //*/
  if (rev_str) {
    int tmp = left_pwm;
    left_pwm = right_pwm;
    right_pwm = tmp;
  }


  sentData.speedmotorLeft = left_pwm;
  sentData.speedmotorRight = right_pwm;



  if (!storeValue) {
    memSetted = false;
  } else {
    if (!memSetted) {
      memSetted = true;
      store_values();
    }
  }

  if (!wifi_remote) {
    if (!radio.write( &sentData, sizeof(sentData) )) {
#ifdef DEBUG_FAILS
      Serial.println(F("failed"));
#endif
    }
  } else {
    esp_err_t result = -1;
    if (current_bot == EGRAB) {
      result = esp_now_send(egrabAddress, (uint8_t *) &sentData, sizeof(sentData));
    } else {
      result = esp_now_send(hingeAddress, (uint8_t *) &sentData, sizeof(sentData));
    }
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    } else {
      //Serial.println("Error sending the data");
    }
  }


  //------------------- all debug prints


  //DEBUG NOISE POT
#ifdef DEBUG_NOISE
    int check_debug = leverValue;
    if (check_debug > debug_max) {
    debug_max = check_debug ;
    }
    if (check_debug < debug_min) {
    debug_min = check_debug;
    }
    Serial.print(check_debug );
    Serial.print(", ");
    Serial.print(debug_max);
    Serial.print(", ");
    Serial.print(debug_min);
    Serial.print(", ");
    Serial.println(debug_max-debug_min);
#endif

  //DEBUG FOR SENT VALUES
#ifdef DEBUG_SENT
    Serial.print("LPWM: ");
    Serial.print(sentData.speedmotorLeft);
    Serial.print("\t");
    Serial.print("R PWM: ");
    Serial.print(sentData.speedmotorRight);
    Serial.print("\t");
    int delta = 0;
  
    int sign_r = 1;
    if(sentData.speedmotorRight != 0)
     sign_r = sentData.speedmotorRight/sentData.speedmotorRight;
    int sign_l = 1;
    if(sentData.speedmotorLeft != 0)
      sign_l = sentData.speedmotorLeft/sentData.speedmotorLeft;

    int high_v = sentData.speedmotorRight*sign_r >= sentData.speedmotorLeft*sign_l ? sentData.speedmotorRight : sentData.speedmotorLeft;
    int low_v = sentData.speedmotorRight*sign_r <= sentData.speedmotorLeft*sign_l ? sentData.speedmotorRight : sentData.speedmotorLeft;

    if(sign_r == sign_l){
      delta = high_v - low_v;
    }else{
      delta = sentData.speedmotorRight*sign_r + sentData.speedmotorLeft*sign_l;
    }
    Serial.print("delta: ");
    Serial.print(delta);
    Serial.print("\t");
    
    Serial.print("WPN: ");
    Serial.print(sentData.weaponArg);
    Serial.print("\t");
    Serial.print("FIRE: ");
    Serial.println((int)sentData.Fire);
#endif

  //DEBUG FOR READ VALUES
#ifdef DEBUG_READS
    Serial.print("accpot: ");
    Serial.print(accValue);
    Serial.print("\t");

    Serial.print("strpot");
    Serial.print(strValue);
    Serial.print("\t");

    Serial.print("leverpot");
    Serial.print(leverValue);
    Serial.print("\t");

    Serial.print("BL: ");
    Serial.print(leftValue);
    Serial.print("\t");

    Serial.print("BR: ");
    Serial.print(rightValue);
    Serial.print("\t");

    Serial.print("BT: ");
    Serial.print(topValue);
    Serial.print("\t");


    Serial.print("TSW: ");
    Serial.print(storeValue);
    Serial.print("\t");

    Serial.print("BSW: ");
    Serial.print(setMode);
    Serial.print("\t");

    Serial.print("trimPot: ");
    Serial.println(trimValue);
#endif

  //DEBUG EXPO CURVE
#ifdef DEBUG_EXPO
    Serial.print("left:");
    Serial.print(left);
    Serial.print(",");
    Serial.print("expo:");
    Serial.print(strExpoalpha);
    Serial.print(",");
    temp_value_debug += 3;
    if (temp_value_debug > 255) {
    temp_value_debug = 0;
    }
    Serial.print("value:");
    Serial.print(temp_value_debug);
    Serial.print(",");
    int valuex = map(temp_value_debug, 0, strPWMmax, 0, 1000);
    temp_curve_debug = pow(valuex / 1000.0f, strExpoalpha / 100.0f) * strPWMmax;
    Serial.print("curve:");
    Serial.print(temp_curve_debug);
    Serial.println();
#endif
}

void load_values() {
  /*
    //LOADING FROM MEM
    //active angle for the weapon
    int activeAng_low = (int) EEPROM.read(address);
    delay(5);
    int activeAng_high = (int) EEPROM.read(address + 1);
    delay(5);
    wpn_end = activeAng_high << 8 | activeAng_low ;
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
    wpn_default = restAng_high << 8 | restAng_low ;
    //weapon acceleration coefficient
    wpnAccel = (int) EEPROM.read(address+6);
  */
}

void store_values() {
  /*
      EEPROM.update(address, wpn_end);
      delay(5);
      EEPROM.update(address + 1, wpn_end >> 8);
      delay(5);

      EEPROM.update(address + 2, strExpoalpha);
      delay(5);
      EEPROM.update(address + 3, accExpoalpha);
      delay(5);

      EEPROM.update(address + 4, wpn_default);
      delay(5);
      EEPROM.update(address + 5, wpn_default >> 8);
      delay(5);

      EEPROM.update(address + 6, wpnAccel);
      delay(5);

      Serial.println("mem SETTED!");
  */
}
