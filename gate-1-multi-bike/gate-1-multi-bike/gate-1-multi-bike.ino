
/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <esp_now.h>
#include <WiFi.h>

// gate 1 mac: 94:b5:55:fe:7b:68
// gate 2 mac: 3c:8a:1f:5e:b1:cc
// gate 3 mac: 94:54:c5:f0:15:fc
// REPLACE WITH THE MAC Address of your receiver not broad casting anything in this case 
//uint8_t broadcastAddress[] = {0x94, 0xb5, 0x55, 0xfe, 0x7b, 0x68}; // if gate 1 is recieving
//uint8_t broadcastAddress[] = {0x3c, 0x8a, 0x1f, 0x5e, 0xb1, 0xcc}; // if gate 2 is recieving
uint8_t broadcastAddress[] = {0x94, 0x54, 0xc5, 0xf0, 0x15, 0xfc}; //if gate 3 is recieving

// Define variables to store BME280 readings to be sent
const int buttonPin = 4; //define button (maybe needs to be changed to whatever ) 
bool button_press; 
bool buttonState_old= false;

// Variable to store if sending data was successful
String success;
//timer variables 
unsigned long starttime; 
float elapsed_time; 

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
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init pushbutton pin as input 
  pinMode(buttonPin,INPUT_PULLUP); 
  
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
}
 
void loop() {
  //setup button state reading of button 
  //read button state
  bool buttonState = !digitalRead(buttonPin);
  button_press= buttonState && !buttonState_old && (elapsed_time>1000); //detect unique button press
  if (button_press) {
      starttime= millis();
  }
  else {
      elapsed_time = millis()-starttime;
  }

  // Set values to send
  myData.id = 1;
  myData.Sen1 = button_press;
  myData.Sen2= 0; // this is here for now need for the 
//check if buttonstate== button state old should only be true twice at rising edge and lowering edge
  if (buttonState != buttonState_old){
     //ive already tested button outputs im sending one if the gate (button) is tripped (pushed)
    Serial.print(buttonState); // will be 0 when pressed 1 when up
    Serial.print("\t");
    Serial.print(button_press);
    Serial.print("\t");
    Serial.println(elapsed_time); 
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
//  //commented since the board sends sending
//    if (result == ESP_OK) {
//      Serial.println("Sent with success");
//    }
//    else {
//      Serial.println("Error sending the data");
//    }
  }
  // loop is here to make sure unique button press worked 
//  if (button_press == 1) {
//    //ive already tested button outputs im sending one if the gate (button) is tripped (pushed)
//    Serial.print(buttonState); // will be 0 when pressed 1 when up
//    Serial.print("\t");
//    Serial.print(button_press);
//    Serial.print("\t");
//    Serial.println(elapsed_time);
//    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
////  //commented since the board sends sending
////    if (result == ESP_OK) {
////      Serial.println("Sent with success");
////    }
////    else {
////      Serial.println("Error sending the data");
////    }
//  } 
  buttonState_old=buttonState;
}
