/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>  // MQTT Client Library

// Replace with your network credentials
const char* ssid = "AlertLight";
const char* password = "MQTTBroker";

// Replace with your Mosquitto broker IP (use your PC or Raspberry Pi IP)
const char* mqtt_server = "192.168.4.1";  // Change this to your broker's IP
//
//
WiFiClient espClient;
PubSubClient client(espClient);

// gate 1 mac: 3c:8a:1f:77:d7:04
// gate 2 mac: 8c:4f:00:2e:6e:78
// gate 3 mac: 8c:4f:00:2e:c3:d8
//// REPLACE WITH THE MAC Address of your receiver not broad casting anything in this case
//uint8_t broadcastAddress[] = {0x94, 0x54, 0xc5, 0xf0, 0x15, 0xfc};
//not sending anything yet so dont need this
// FSM Variables
bool WS = true; //Waiting State
bool CS; //Crash detected state
bool SenS; //Sensing state
bool TWS; //Waiting to sensing transition
bool TSW; //Sensing to waiting state
bool TWW; //Waiting Latch
bool TCC; //Crash latch
bool TSC; //Sensing to Latch transition
bool TSS; //Sensing latch
bool TCW; //crash to waiting state

// Pins and Sensors
const int light_pin = 13; //out put for the led
float Seconds1;
float Seconds2;
bool veltime = false;

//stuff for ldr reading
const int readingPin= 4; //define button (maybe needs to be changed to whatever )
const int resetPin= 16; //reset the fsm pin
const int ledPin= 13; 
bool reset_press;
bool resetState_old = false;

// Timer variables
unsigned long startTime = 0;
float currentTime = 0;
bool enableTimer = false;
const int timerThreshold = 30000; // All bikes should be through the gate at this point

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int Sen1;
  int Sen2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
//struct_message board2; only using 2 sending boards

// Create an array with all the structures
struct_message boardsStruct[2] = {board1, board2};

//esp_now_peer_info_t peerInfo;
//variables for jump calculations
float total_time; //total time
float air_time; //air time for jump calculation
float straight_time; //
float theta= PI/180*30; //convert to radian 
float g= 9.81; //m/s^2 gravity 
float v0; // in m/s 
float first_dist= 1; // first distance from gate 1 and 2 
float second_dist= 20;//distance from gate 2 to gate 3 (this is used completely if there is no jump in calculation
float straight_dist= 4; // distance from landing of jump to 3rd gatet


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
//  Serial.print("Packet received from: ");
//  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//  Serial.println(macStr);
// dont want to print the mac addresses 
  memcpy(&myData, incomingData, sizeof(myData));
  //uncomment for data recivied testing
//  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].Sen1 = myData.Sen1;
  boardsStruct[myData.id - 1].Sen2 = myData.Sen2;
  // uncomment this if we are questioning the data coming to the board
//  Serial.printf("Sensor 1 value: %d \n", boardsStruct[myData.id - 1].Sen1);
//  Serial.printf("Sensor 2 value: %d \n", boardsStruct[myData.id - 1].Sen2);
//  Serial.println();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  WiFi.disconnect();
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  pinMode(readingPin, INPUT);  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA); 
  //Wifi stuff 
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");
  Serial.println("\nConnected to the WiFi network");

  while(WiFi.status() != WL_CONNECTED){
    Serial.print(WiFi.status());
    delay(1000);
}

  Serial.print("[+] ESP32 IP : ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, 1883);
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
void loop() {
  // put your main code here, to run repeatedly:
  // Reconnect to MQTT if not connected
  if (!client.connected()) {
    while (!client.connected()) {
      if (client.connect("esp32/Gate")) {
        client.subscribe("esp32/Gate");
      } else {
        delay(1000);
      }
    }
  }
  client.loop();  // Keep the connection alive
  //reset pin stuff 
  bool resetState = !digitalRead(resetPin);
  reset_press= resetState && !resetState_old; //detect unique reset button press with delay
  //now calculate the elapsed time when the sensing state is true during the bike run from 2nd to 3rd gate
  // If the timer is enabled, update elapsed time
  if (enableTimer) { 
    currentTime = (millis() - startTime);
  } else if (!enableTimer) {
    currentTime = 0; // Reset time when the timer is off
    //have a timer always calculating the time
    startTime=millis(); 
  }  
  //setup button state reading of button
  bool sensorValue = !digitalRead(readingPin); 
    if (sensorValue ==1) {
    //Serial.println("Gate is tripped");
    digitalWrite(ledPin, HIGH);
  }
  else {
    //Serial.println("Gate is closed");
    digitalWrite(ledPin, LOW);
  }
  // Access the variables for each board that we need which is Sen1 from board 1 and additionally Sen1 from board 2  
  bool Sensor1 = boardsStruct[0].Sen1;
  //since both boards are updating Sen1 variable in from their buttons
  bool Sensor2= sensorValue;
  bool Sensor3 = boardsStruct[1].Sen1;

  //FSM STUFF BELOW
  // Timer Code
  if (Sensor1) {
    Seconds1 = (millis() / 1000.0);
  }
  if (Sensor2) {
    Seconds2 = (millis() / 1000.0); // units of seconds 
  // Conversion from Velocity into time between sensor 2 and 3
  //calculate a velocity based on straight distance  
  v0=first_dist/(Seconds2-Seconds1); // find v0 from deltaX/deltaT of g1 - g2 m/s
  total_time= second_dist/v0;
  //v0 is the initial velocity at the jump location
  //if you are calculating based on a jump geometry use this

//  Xv0=v0*cos(theta);
//  Yv0=v0*sin(theta);
//  air_time=2*(v0*sin(theta)*g); //in seconds
//  straight_time=straight_dist/v0; //also in seconds
//  total_time= air_time+straight_time; //add times together for final calculation of timer
  //need time distance after jump 
  
  // vtime = (Dist. between gate 2-3 w factor of safety(in meters))/(Dist. between gate 1-2(meters))/(Time)
  }

  // Overwrites the timer so a crash can be detected
  if (total_time <= currentTime) {
    veltime = true;
  }
  else {
    veltime = false;
  }

  // Transition Code
  TWW = WS && (!Sensor2);
  TWS = WS && (Sensor2);
  TSS = SenS && (!Sensor3 && !veltime);
  TCC = CS && !resetState;
  TCW = CS && resetState;
  TSC = SenS && (veltime);
  TSW = SenS && ((Sensor3 && !veltime));


  // States
  WS = (TWW || TSW || TCW);
  SenS = (TWS || TSS);
  CS = (TSC || TCC);

  // Output (not testing fsm right now)
  Serial.print(Sensor1);
  Serial.print("\t");
  Serial.print(Sensor2);
  Serial.print("\t");
  Serial.print(Sensor3);
  Serial.print("\t");
  Serial.print(total_time);
  Serial.print("\t");
  Serial.print(v0); 
  Serial.print("\t");
  Serial.print(currentTime);
  Serial.print("\t");
  Serial.print(WS);
  Serial.print("\t");
  Serial.print(SenS);
  Serial.print("\t");
  Serial.print(CS);
  Serial.println();

  //set timer to start counting in the sensing state and off every other time
  // If SenS just became true, store the time
  if (SenS == true) { 
    enableTimer = true; // Start timer
  } 
  // Turn off the timer when exiting sensing state
  else if (!SenS) { 
    enableTimer = false; 
  }
  // this else if statement turns the crash light on or off
  if (CS == true) {
  // code here for turning on the light 
    // Publish a message to the topic "test/topic"
//    static unsigned long lastMsg = 0;
//    if (millis() - lastMsg > 10000) {  // Publish every 5 seconds
//      lastMsg = millis();
//      client.publish("esp32/Gate", "1"); //THIS IS CRASH
//     // Serial.println("True Crash");
//      delay(4000);
//  }
  
  }
  else if (!CS) {
//    client.publish("esp32/Gate", "0"); //THIS IS NO CRASH
  }
  resetState_old= resetState;
}
