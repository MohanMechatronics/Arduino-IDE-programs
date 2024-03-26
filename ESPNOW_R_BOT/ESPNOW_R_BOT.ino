#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 0

const int motor1A = 14;
const int motor1B = 27;
const int motor2A = 26;
const int motor2B = 25;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup() {
  Serial.begin(115200);

  // configure LED PWM functionalitites
  ledcSetup(0,2000,8); //ledcSetup(ledChannel, freq, resolution);
  ledcSetup(1,2000,8);
  ledcSetup(2,2000,8);
  ledcSetup(3,2000,8);

  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(14, 0); //ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(27, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);

  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
  if(*data == HIGH){
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
  }
  else{
    ledcWrite(0, 255);
    ledcWrite(1, 0);
    ledcWrite(2, 255);
    ledcWrite(3, 0);
  }
}

void loop() {
  // Chill
}