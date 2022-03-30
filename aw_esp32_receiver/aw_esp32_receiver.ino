#include <esp_now.h>
#include <WiFi.h>

#define PWM1_ch 0
#define PWM2_ch 1
#define PWM3_ch 2
#define PWM4_ch 3

#define PWM_res 8
#define PWM_freq 400


#define IN1_gpio 21
#define IN2_gpio 22
#define IN3_gpio 17
#define IN4_gpio 16

//MAC robot C8:C9:A3:CB:33:F8
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
  short speedmotorLeft;
  short speedmotorRight;
  short weaponStrenght;
  char Fire;
  short Angle;
  short packetArg1;
}
packet_t;


packet_t recData;

bool failsafe = false;
unsigned long failsafeMaxMillis = 200;
unsigned long lastPacketMillis = 0;
unsigned long currentWaitMillis = 0;

int recLpwm = 0;
int recRpwm = 0;
int recWpn = 0;
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
  recWpn = recData.weaponStrenght;
  recFire = recData.Fire;
  recAngle = recData.Angle;
  recpckArg1 = recData.packetArg1;
  lastPacketMillis = millis();
}
void setup() {
  //Serial.begin(115200);
  //Serial.println("Ready.");

  ledcAttachPin(IN1_gpio, PWM1_ch);
  ledcAttachPin(IN2_gpio, PWM2_ch);
  ledcAttachPin(IN3_gpio, PWM3_ch);
  ledcAttachPin(IN4_gpio, PWM4_ch);

  ledcSetup(PWM1_ch, PWM_freq, PWM_res);
  ledcSetup(PWM1_ch, PWM_freq, PWM_res);
  ledcSetup(PWM1_ch, PWM_freq, PWM_res);
  ledcSetup(PWM1_ch, PWM_freq, PWM_res);
  
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
  }else{
    setM2speed(recRpwm);
    setM1speed(recLpwm);
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
