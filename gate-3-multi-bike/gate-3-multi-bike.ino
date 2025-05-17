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
//// REPLACE WITH THE MAC Address of your receiver not broad casting anything in this case
//uint8_t broadcastAddress[] = {0x94, 0x54, 0xc5, 0xf0, 0x15, 0xfc};
//not sending anything yet so dont need this
const int max_bike=10;
unsigned long startTimes[max_bike];
unsigned long stopTimes[max_bike];
unsigned long expectedArrivalTime[max_bike];
bool hasStarted[max_bike];
bool hasVelocity[max_bike];
bool trip3Received[max_bike];

int dist12 = 36;
int dist23 = 50;


int timerCount = 0;

// FSM Variables
bool WS = true; //Waiting State
bool CS; //Crash detected state
bool SenS; //Sensing state
bool TrackS; //Tracking state when button 1 is pressed
bool TWS; //Waiting to sensing transition
bool TSW; //Sensing to waiting state
bool TWW; //Waiting Latch
bool TCW; //Crash to Waiting transition
bool TCC; //Crash latch
bool TSC; //Sensing to Latch transition
bool TSS; //Sensing latch
// Pins and Sensors
const int light_pin = 13; //out put for the led
//stuff for button readings
const int buttonPin = 4; //define button (maybe needs to be changed to whatever )
bool button_press;
bool buttonState_old= false;
//gate 1 and two timer decalation
//unsigned long first_old_time{max_bike}= {0}; //timer for time it takes bike to pass the first and second gate
//float first_current_time{max_bike}= {0}; // equally sized current time check
bool enable_first_timer = false; 

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

// Create an array with all the structures
struct_message boardsStruct[2] = {board1, board2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
//  Serial.print("Packet received from: ");
//  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//  Serial.println(macStr);
// dont want to print the mac addresses but can uncomment that if that changes
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
  pinMode(light_pin, OUTPUT);

  //Init pushbutton pin as input 
  pinMode(buttonPin,INPUT_PULLUP); 
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
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
    // initializes the sensors and their outpins
  // this can be easily changed if there is a change in which board is running the FSM (in testing I used the third hence sensor 3 being a button) 
  // Acess the variables for each board that we need which is Sen1 from board 1 and Sen2 from board 2 
  bool Sensor1 = boardsStruct[0].Sen1; 
  bool Sensor2 = boardsStruct[1].Sen2;
  //finally read a unique button press for sensor 3 
    //read button state
  bool buttonState = !digitalRead(buttonPin);
  button_press= buttonState && !buttonState_old; //detect unique button press
  bool Sensor3 = button_press;

  unsigned long currentTime = millis();
  
//  for (int i=0 ; i<max_bike; i++){ 
//    if (enable_first_timer){ 
//      first_current_time(i)=(millis()-first_old_time(i));
//    }
//    else if (!enable_first_timer){
//      first_current_time(i)=0;
//      first_old_time(i)=millis(); 
//    }
//  }
  // Transition Code
//  TWW = WS && (!Sensor2);
//  TWS = WS && (Sensor2);
//  TSS = SenS && (!Sensor3 && !veltime);
//  TCC = CS;
//  TSC = SenS && (veltime);
//  TSW = SenS && ((Sensor3 && !veltime));
//  
//  // States
//  WS = (TWW);
//  TrackingS = (TWS);
  //CS = (TSC || TCC);

    //set timer to start counting in the sensing state and off every other time
  // If SenS just became true, store the time


  if(Sensor1){
    int idx = getAvailableSlot();
    if (idx >= 0){
      startTimes[idx] = currentTime;
      hasStarted[idx] = true;
    }
  }
  if(Sensor2){
    for (int i = 0; i < max_bike; i++) {
      if (hasStarted[i] && !hasVelocity[i]){
        stopTimes[i] = currentTime;
        float elapse = (stopTimes[i] - startTimes[i]) / 1000.0;
        float velocity = dist12 / elapse;
        float expectedTime = dist23 / velocity;
        expectedArrivalTime[i] = stopTimes[i] + (unsigned long)(expectedTime * 1000);
        hasVelocity[i] = true;

        Serial.print("[Run ");
        Serial.print(i);
        Serial.print("] Velocity: ");
        Serial.print(velocity, 2);
        Serial.println(" in/s");

        Serial.print("[Run ");
        Serial.print(i);
        Serial.print("] Must reach Tripwire 3 in ");
        Serial.print(expectedTime, 2);
        Serial.println(" seconds");
      }
    }
  }
  
  if (Sensor3) { 
    for(int i = 0; i < max_bike; i++) {
      if(hasVelocity[i] && !trip3Received[i]) {
        trip3Received[i] = true;
        if (currentTime <= expectedArrivalTime[i]) {
          Serial.print("[Run ");
          Serial.print(i);
          Serial.println("] is Safe.");
        }
        else{
        Serial.print("[Run ");
        Serial.print(i);
        Serial.println("] Crashed.");
        digitalWrite(light_pin, HIGH);
        }
        resetRun(i);
      }
    }
  }
  for (int i = 0; i < max_bike; i++) {
    if (hasVelocity[i] && !trip3Received && currentTime > expectedArrivalTime[i]) {
      Serial.print("[Run ");
      Serial.print(i);
      Serial.println("] Crashed.");
      digitalWrite(light_pin, HIGH);   
      resetRun(i);   
    }
  }
//  // this else if statement turns the crash light on or off
//  if (CS == true) {
//    digitalWrite(light_pin, HIGH);
//  }
//  else if (!CS) {
//    digitalWrite(light_pin, LOW);
//  }
  // Output (not testing fsm right now)

}

void resetRun(int idx){
  startTimes[idx] = 0;
  stopTimes[idx] = 0;
  expectedArrivalTime[idx] = 0;
  hasStarted[idx] = false;
  hasVelocity[idx] = false;
  trip3Received[idx] = false;
}

int getAvailableSlot(){
  for (int i = 0; i < max_bike; i++) {
    if (!hasStarted[i]) return i;
  }
  return -1;
}

  
  
