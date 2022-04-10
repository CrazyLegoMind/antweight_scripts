#include <esp_now.h>
#include <WiFi.h>


#define PWM1_ch 12
#define PWM2_ch 13
#define PWM3_ch 14
#define PWM4_ch 15
#define PWM5_ch 10
#define PWM6_ch 11

#define PWM_res 8
#define PWM_freq 400

#define IN1_gpio 21
#define IN2_gpio 22
#define IN3_gpio 17
#define IN4_gpio 16
#define IN3w_gpio 0
#define IN4w_gpio 4

#define weapPot 33

//MAC robot grab    C8:C9:A3:CB:70:54
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
  int16_t speedmotorLeft;
  int16_t speedmotorRight;
  int16_t weaponStrenght;
  int16_t Angle;
  int16_t packetArg1;
  int8_t Fire;
}
packet_t;


packet_t recData;

bool failsafe = false;
unsigned long failsafeMaxMillis = 200;
unsigned long lastPacketMillis = 0;
unsigned long currentWaitMillis = 0;
int wpn_prev_pos = 0;
int wpn_prev_time = 0;
int wpn_prev_speed = 0;
int wpn_prev_pwm = 0;

int recLpwm = 0;
int recRpwm = 0;
int recWpnStr = 0;
bool recFire = false;
int recAngle = 0;
int recpckArg1 = 0;

// Variable to store if sending data was successful
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
  recLpwm = recData.speedmotorLeft;
  recRpwm = recData.speedmotorRight;
  recWpnStr = recData.weaponStrenght;
  recFire = recData.Fire;
  recAngle = recData.Angle;
  recpckArg1 = recData.packetArg1;
  lastPacketMillis = millis();
}
void setup() {
//  Serial.begin(115200);
//  Serial.println("Ready.");
  analogReadResolution(10);
  ledcAttachPin(IN1_gpio, PWM1_ch);
  ledcAttachPin(IN2_gpio, PWM2_ch);
  ledcAttachPin(IN3_gpio, PWM3_ch);
  ledcAttachPin(IN4_gpio, PWM4_ch);
  ledcAttachPin(IN3w_gpio, PWM5_ch);
  ledcAttachPin(IN4w_gpio, PWM6_ch);

  ledcSetup(PWM1_ch, PWM_freq, PWM_res);
  ledcSetup(PWM2_ch, PWM_freq, PWM_res);
  ledcSetup(PWM3_ch, PWM_freq, PWM_res);
  ledcSetup(PWM4_ch, PWM_freq, PWM_res);
  ledcSetup(PWM5_ch, PWM_freq, PWM_res);
  ledcSetup(PWM6_ch, PWM_freq, PWM_res);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  recWpnStr =0;
}

void loop() {
  failsafe = false;
  unsigned long current_time = millis();
  if(current_time - lastPacketMillis > failsafeMaxMillis){
    failsafe = true;
    //Serial.println(current_time - lastPacketMillis);
  }
  if(failsafe){
    setM2speed(0);
    setM1speed(0);
//    setM3speed(0);
    seek_angle_smooth(512, 8);
  }else{
    setM2speed(recRpwm);
    setM1speed(recLpwm);
    seek_angle_smooth(recAngle,recpckArg1);
  }
  delay(10);
}


int setM1speed(int rpm) {
  if (rpm < 0) {
    ledcWrite(PWM1_ch, 0);
    ledcWrite(PWM2_ch, -rpm);
  } else if (rpm > 0) {
    ledcWrite(PWM1_ch, rpm);
    ledcWrite(PWM2_ch, 0);
  } else if (rpm == 0) {
    ledcWrite(PWM1_ch, 255);
    ledcWrite(PWM2_ch, 255);
  }
  return 0;
}

int setM2speed(int rpm) {
  if (rpm < 0) {
    ledcWrite(PWM3_ch, 0);
    ledcWrite(PWM4_ch, -rpm);
  } else if (rpm > 0) {
    ledcWrite(PWM3_ch, rpm);
    ledcWrite(PWM4_ch, 0);
  } else if (rpm == 0) {
    ledcWrite(PWM3_ch, 255);
    ledcWrite(PWM4_ch, 255);
  }
  return 0;
}

int setM3speed(int rpm) {
  if (rpm < 0) {
    ledcWrite(PWM5_ch, 0);
    ledcWrite(PWM6_ch, -rpm);
  } else if (rpm > 0) {
    ledcWrite(PWM5_ch, rpm);
    ledcWrite(PWM6_ch, 0);
  } else if (rpm == 0) {
    ledcWrite(PWM5_ch, 255);
    ledcWrite(PWM6_ch, 255);
  }
  return 0;
}

int seek_angle_smooth(int target_pos,int accel) {
  if (target_pos < 0 || target_pos > 830) return 1;
  int wpn_current_pos = analogRead(weapPot);
//  Serial.println("Pot: " + (String) wpn_current_pos);
  unsigned long wpn_current_time = millis();
  float pos_delta = wpn_current_pos - wpn_prev_pos;
  unsigned long time_delta = wpn_current_time - wpn_prev_time;
  float wpn_speed = pos_delta/time_delta;
  int gap = target_pos - wpn_current_pos;
  int wpn_pwm = 0;
  int accel_treshold = abs(wpn_prev_pwm)+accel;
  
  if (abs(gap) > 20) {
    wpn_pwm = target_pos -(wpn_current_pos+ wpn_speed*13);
  }
  
  if(accel> 0) wpn_pwm = constrain(wpn_pwm, -accel_treshold, accel_treshold);
  wpn_pwm = constrain(wpn_pwm, -recWpnStr, recWpnStr);
  setM3speed(wpn_pwm);

  //prev sets for next iteration
  wpn_prev_pos = wpn_current_pos;
  wpn_prev_time = wpn_current_time;
  wpn_prev_speed = wpn_speed;
  wpn_prev_pwm = wpn_pwm;
  
  /*
  //if(wpn_speed < 350) return 1;
  Serial.print(target_pos);
  Serial.print("\t");
  Serial.print(wpn_current_pos);
  Serial.print("\t");
  Serial.print(abs(wpn_speed) * 100.0f);
  Serial.print("\t");
  Serial.print(abs(wpn_pwm));
  Serial.print("\t");
  Serial.println(accel_treshold);
  //*/
  return 0;
}
