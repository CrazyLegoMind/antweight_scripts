#include <SPI.h>
#include "RF24.h"
#include <EEPROM.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

//datas that will be sent to the receiver
typedef struct {
  short speedmotorLeft;
  short speedmotorRight;
  short weaponStrenght;
  char Fire;
  short Angle;
  short weaponAccel;
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


//---------------------------------------ESP_NOW Variables
//MAC robot C8:C9:A3:CB:33:F8
uint8_t broadcastAddress[] = {0xC8, 0xC9, 0xA3, 0xCB, 0x33, 0xF8};
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





// stick and trims potentiometers input pins
const int steerPot = 35;
const int accPot = 33;
const int leverPot = 34;
const int modePot = 39;

const int rightBtn = 0;
const int leftBtn = 12;
const int topBtn = 4;
const int lowSwitch = 32;
const int topSwitch = 25;
const int ledPin = 2;

//various stick/potentiometers limits
//this is useful if:

//tx potentiometers don't travel from 1023 to 0
//-if their center positon is not 512
//-their center position has some play so it is a range of values
// instead of a single vaue

const int potStrRightEnd = 10; //default 0, reversed 1024
const int potStrLeftEnd = 4085; //default 1024, reversed 0

const int potAccForwardEnd = 4085; // default 0, reversed 1024
const int potAccBackEnd = 10; //default 1024, reversed 0


const int potStrRightStart = 1750; //default 512
const int potStrLeftStart = 1810; //default 512

const int potAccForwardStart = 1835; //default 512
const int potAccBackStart = 1735; //default 512

const int potLevUpStart = 1790; //default 512
const int potLevDownStart = 1900; //default 512

const int potLevUpEnd = 1030; //default 0, reversed 1024
const int potLevDownEnd = 2500; //default 1024, reversed 0


//var that will be stored and retreived from
//eprom mem
int restAngle = 20;
int activeAngle = 1023;
int strExpoalpha = 115;
int accExpoalpha = 115;
int wpnAccel = 2;


//customisable vars
int PWMmax = 255;
int expoMax = 190; //exp will be this/100
int analogReadMax = 4095;

//variables for the sketch
int right = 0;
int left = 0;
int forward = 0;
int back = 0;
int wpn = 0;
int wpn_range = 1023;
int strPWMmax = PWMmax;
int accPWMmax = PWMmax;
int modeValue = 0;
int address = 0;
bool memSetted = true;
int rev_str = false;
int debug_min = 4095;
int debug_max = 0;

enum bot_control {
  GRAB,
  FLIP,
  WEDG,
  ESPREC
};

int bot_current = ESPREC;
bool wpnSafetyCeck = true;

void setup() {
  //store_values(); //only to initialize
  pinMode(rightBtn, INPUT_PULLUP);
  pinMode(leftBtn, INPUT_PULLUP);
  pinMode(topBtn, INPUT_PULLUP);
  pinMode(lowSwitch, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin,HIGH);

  Serial.begin(115200);

  //load_values()

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



  //---------------------------------------RF24 Setup
  //radio.begin();
  //radio.setPALevel(RF24_PA_LOW);
  //radio.openWritingPipe(current_addr);

  //---------------------------------------ESP NOW setup
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}



void loop() {
  //read pots values
  int strValue = analogRead(steerPot);
  int accValue = analogRead(accPot);
  int leverValue = analogRead(leverPot);
  modeValue = analogRead(modePot);
  if (strValue < debug_min) {
    debug_min = strValue;
  }
  //read btns values
  bool setMode = !digitalRead(lowSwitch);


  bool storeValue = !digitalRead(topSwitch);
  bool radioSet = false;//!digitalRead(topSwitch);

  bool rightValue = !digitalRead(rightBtn);
  bool leftValue = !digitalRead(leftBtn);
  bool topValue = !digitalRead(topBtn);


  bool drSet = false;
  bool expoSet = false;
  bool safetySet = false;
  bool wpnSet = false;
  bool mode5Set = false;

  int mode_n = modeValue / 205;
  //Serial.println(mode_n);
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
        mode5Set = true;
        break;
      default:
        break;
    }
  }

  //----------------------------------------------------WEAPON CODE
  wpn = map(leverValue, potLevDownEnd, potLevUpEnd, 0, wpn_range);
  wpn = constrain(wpn, 0, wpn_range);

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
    //FIX MSSING POT
    //activeAngle = analogRead(TrimUp);
    //restAngle = analogRead(TrimDown);
  }


  if (radioSet) {
    //Serial.print("serring radio...");
    //Serial.println(bot_current);
    if (modeValue < 300 && bot_current != GRAB) {
      bot_current = GRAB;
      rev_str = false;
      current_addr = WRITE_ADDR_GRAB;
      radio.setChannel(76);
      radio.openWritingPipe(current_addr);
    }
    if (modeValue > 301 && modeValue < 600 && bot_current != FLIP) {
      bot_current = FLIP;
      rev_str = true;
      current_addr = WRITE_ADDR_FLIP;
      radio.setChannel(85);
      radio.openWritingPipe(current_addr);

    }
    if (modeValue > 601 && bot_current != WEDG) {
      bot_current = WEDG;
      rev_str = false;
      current_addr = WRITE_ADDR_WEDG;
      activeAngle = 166;
      restAngle = 74;
      radio.setChannel(76);
      radio.openWritingPipe(current_addr);

    }
  }

  if (rightValue) {
    sentData.Angle = activeAngle;
    sentData.Fire = true;
  } else {
    wpn = map(leverValue, potLevDownEnd, potLevDownStart,  activeAngle, restAngle);
    wpn = constrain(wpn, restAngle, activeAngle);
    sentData.Angle = wpn;
    sentData.Fire = false;
  }

  sentData.weaponStrenght = PWMmax;
  sentData.weaponAccel = 0;


  //----------------------------------------------------MOTOR CODE
  //setting dual rates

  if (drSet) {
    //strPWMmax = map(TrimUpValue, 0, 1022, 0, PWMmax);
    //accPWMmax = map(TrimDownValue, 0, 1022, 0, PWMmax);
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
    //strExpoalpha = map(TrimUpValue, 10, 1022, 100, expoMax);
    //accExpoalpha = map(TrimDownValue, 10, 1022, 100, expoMax);
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
  int left_pwm = constrain(forward + back + right - left, -PWMmax, PWMmax);
  int right_pwm = constrain(forward + back - right + left, -PWMmax, PWMmax);

  //*/
  if (rev_str) {
    int tmp = left_pwm;
    left_pwm = right_pwm;
    right_pwm = tmp;
  }

  //recorrect the data not to have more than max +PWM while non pivot-steering
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

  if (bot_current != ESPREC) {
    if (!radio.write( &sentData, sizeof(sentData) )) {
      //Serial.println(F("failed"));
    }
  } else {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sentData, sizeof(sentData));

    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    } else {
      //Serial.println("Error sending the data");
    }
  }



  /* DEBUG PACKET
    Serial.print("LPWM: ");
    Serial.print(sentData.speedmotorLeft);
    Serial.print("\t");
    Serial.print("R PWM: ");
    Serial.print(sentData.speedmotorRight);
    Serial.print("\t");
    Serial.print("WPN: ");
    Serial.println(sentData.Angle);

    //*/

  ///*DEBUG INPUT
  Serial.print("accpot: ");
  Serial.print(accValue);
  Serial.print("\t");

  Serial.print("strpot");
  Serial.print(strValue);
  Serial.print("\t");

  Serial.print("leverpot");
  Serial.print(leverValue);
  Serial.print("\t");

  Serial.print("BR: ");
  Serial.print(rightValue);
  Serial.print("\t");

  Serial.print("BL: ");
  Serial.print(leftValue);
  Serial.print("\t");

  Serial.print("BT: ");
  Serial.print(topValue);
  Serial.print("\t");


  Serial.print("Sw1: ");
  Serial.print(radioSet);
  Serial.print("\t");

  Serial.print("sW2: ");
  Serial.print(setMode);
  Serial.print("\t");

  Serial.print("modePot: ");
  Serial.println(modeValue);
  //*/
}

void load_values() {
  /*
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
  */
}

void store_values() {
  /*
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
  */
}
