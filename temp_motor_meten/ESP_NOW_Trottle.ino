#include <esp_now.h>
#include <WiFi.h>

//#define DEBUG;

uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2,0x36, 0xB2, 0xD0};// REPLACE WITH THE MAC Address of your receiver 
bool trottlePermission;
bool motorConnected;

const uint32_t maxPermissionTime = 10000;

String success; // Variable to store if sending data was successful

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 #ifdef DEBUG
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
 #endif
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&motorConnected, incomingData, sizeof(motorConnected));
  #ifdef DEBUG
  Serial.print("Bytes received: ");
  Serial.println(len);
  #endif
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
/*if(millis() - permissionTime  > maxPermissionTime){
permissionTime = millis();

}
*/

  // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &trottlePermission, sizeof(trottlePermission));
   #ifdef DEBUG
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  #endif
  }
  


