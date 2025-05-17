
/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <esp_now.h>
#include <WiFi.h>

// gate 1 mac: 3c:8a:1f:77:d7:04
// gate 2 mac: 8c:4f:00:2e:6e:78
// gate 3 mac: 8c:4f:00:2e:c3:d8
// REPLACE WITH THE MAC Address of your receiver not broad casting anything in this case
//uint8_t broadcastAddress[] = {0x3c, 0x8a, 0x1f, 0x77, 0xd7, 0x04}; //if gate 1 is running fsm
uint8_t broadcastAddress[] = {0x8c, 0x4f, 0x00, 0x2e, 0x6e, 0x78};// if gate 2 is running fsm
//uint8_t broadcastAddress[] = {0x8c, 0x4f, 0x00, 0x2e, 0xc3, 0xd8};// if gate 3 is running fsm

//Pin for battery reading
const int readingPin = 4;
const int pin17 = 17;
const int ledPin = 13;
// Variable to store if sending data was successful
String success;
//timer variables
//unsigned long starttime;
//float elapsed_time;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int id; // must be unique for each sender board
  int Sen1;
  int Sen2;
} struct_message;

// Create a struct_message called myData
struct_message myData;


// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //uncomment this out if data is questionably sending
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init pushbutton pin as input
  pinMode(readingPin, INPUT);
  pinMode(ledPin, OUTPUT);

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
}

void loop() {
  //setup button state reading of button
  //read button state
  bool sensorValue = !digitalRead(readingPin); //read output from pin 4\
  //this should be a 1 when tripped and a 0 when not tripped in this way
  // Set values to send
  myData.id = 2;
  myData.Sen1 = sensorValue;
  myData.Sen2 = 0; // this is here for now need for the
  //ive already tested button outputs im sending one if the gate (button) is tripped (pushed)
  // always sending the Sensor value
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (sensorValue == 1) {
    Serial.println("Gate is tripped");
    digitalWrite(ledPin, HIGH);
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  else {
    Serial.println("Gate is closed");
    digitalWrite(ledPin, LOW);
  }
  //commented since the board sends sending
}
