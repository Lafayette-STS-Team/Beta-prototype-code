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
const int max_bike = 3;
// FSM Variables
//states
bool S1 = true; //start in waiting state
bool S2; bool S3; bool S4;
//transitions
bool v1; bool v2; bool v3; bool v4; bool v5; bool v6; bool v7; bool v8; bool v9;
//boolean check for any crash in i-th in any bike count
bool we_have_crash;

// Constant Pins and Sensors
const int light_pin = 13; //out put for the led
//stuff for gate 3 button reading
const int buttonPin = 4; //define button (maybe needs to be changed to whatever )
bool button_press;
bool buttonState_old = false;
//Timer for button delay
unsigned long startButtonTime;
float elapsedButtonTime;

//counter variable
int first_gate_counter = -1;
int second_gate_counter = -1;

//gate 1 and 2 timer decalation
unsigned long old_first_time[max_bike] = {0}; //timer for time it takes bike to pass the first and second gate
float first_current_time[max_bike] = {0}; // equally sized current time check
bool enable_first_timer[max_bike] = {false};

//gate 2 and 3 timer stuff below
unsigned long old_second_time[max_bike] = {0};
float second_current_time[max_bike] = {0};
bool enable_second_timer[max_bike] = {false};

//velocity timer
float  delta_t[max_bike] = {0};
float speed_time[max_bike] = {0};
float velocity[max_bike] = {0};
int delta_d_1 = 10; //units of meters (distance from g1 - g2)
int delta_d_2 = 20; //units of meters (distance from g2 - g3)

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

//Declare reset button
const int resetPin = 17;
bool reset_press;
bool resetState_old = false;
//need these due to compiling errors
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len);
void counter(int &value, bool increment, bool decrement, bool reset);
void counter2(int &value2, bool increment2, bool decrement2, bool reset2);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //set pinmodes
  pinMode(light_pin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  //pinMode(resetPin, INPUT_PULLUP);
  //esp now stuff
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // put your main code here, to run repeatedly:
  bool buttonState = !digitalRead(buttonPin);
  button_press = buttonState && !buttonState_old && (elapsedButtonTime > 1000); //detect unique button press with delay
  // declare reset button stuff 
//  bool resetState = !digitalRead(resetPin);
//  reset_press= resetState && !resetState_old && (elapsedButtonTime > 1000); //detect unique reset button press with delay
  //set timer delay
  if (button_press || reset_press) {
    startButtonTime = millis(); //start the self referenial timer on button press
  }
  else {
    elapsedButtonTime = millis() - startButtonTime;
  }
  // this can be easily changed if there is a change in which board is running the FSM (in testing I used the third hence sensor 3 being a button)
  // Access the variables for each board that we need which is Sen1 from board 1 and additionally Sen1 from board 2
  bool Sensor1 = boardsStruct[0].Sen1;
  //since both boards are updating Sen1 variable in from their buttons
  bool Sensor2 = boardsStruct[1].Sen1;
  bool Sensor3 = button_press;
  //  // initialize counter to increment at sensor 1 decrement on sensor 2 and reset during a button
  counter(first_gate_counter, Sensor1, Sensor2, 0);
  counter2(second_gate_counter, Sensor2, Sensor3, 0);
  //now need to set the first timer
  if (Sensor1 == 1) {
    enable_first_timer[first_gate_counter] = true;
  }
  if (Sensor2 == 1) {
    enable_second_timer[second_gate_counter] = true;
    enable_first_timer[second_gate_counter] = false;
  }
  else if (Sensor3 == 1) { // loop for reseting all counting variables
    if (second_gate_counter == 0) { //this is true for the last bike in the gate always will be left a i=1 this does not work right now since 
      enable_second_timer[1] = false; // the second bike is the last bike to exit the fsm if there is more then one in the fsm
      enable_second_timer[0] = false; // this covers the condition tha only one bike was ever in the FSM
    }
    else {
      enable_second_timer[second_gate_counter] = false;
      //shift everything down a i value as soon as first bike is cleared gate 3
      for (int i = 0; i < second_gate_counter; i++) {
        speed_time[i] = speed_time[i + 1];
        second_current_time[i] = second_current_time[i + 1];
        enable_second_timer[i] = enable_second_timer[i + 1];
      }
    }
  }
  //timer 1 starts at any tipping of gate 1 and resets for each individual bike at a tripping of gate 2
  //just prints and stuff for debugging
  for (int i = 0; i < max_bike; i++) {
    if (enable_first_timer[i]) {
      // Debugging output
      first_current_time[i] = millis() - old_first_time[i];
      Serial.print(first_current_time[i] / 1000);
      Serial.print("\t");
    }
    else {
      old_first_time[i] = millis();
    }
    //timer 2 starts at any tipping of gate 2 and resets for each individual bike at a tripping of gate 3
    //checks the if the timer set by the timer itself is < the timer calculated by the velocity
    if (enable_second_timer[i]) {
      second_current_time[i] = millis() - old_second_time[i];
      delta_t[i] = (first_current_time[i]) / 1000; // units of seconds
      velocity[i] = delta_d_1 / delta_t[i]; // units of m/s
      speed_time[i] = delta_d_2 / velocity[i]; //velocity calculated in m/s using distance from 2-3/velocity of gate 1-2
      Serial.print(second_current_time[i] / 1000); //print the running time in seconds
      Serial.print("\t clear time: ");
      Serial.print(speed_time[i]);
      Serial.print("\t gate counter:");
      Serial.print(second_gate_counter);
      Serial.print("\t loop #:");
      Serial.print(i);
      Serial.print("\t");
    }
    else {
      old_second_time[i] = millis();
    }
    if (second_current_time[i] / 1000 > speed_time[i]) {
      we_have_crash = true;
      for (int j = 0; j < max_bike; j++) {
        enable_first_timer[j] = false;
        enable_second_timer[j] = false;
      }
      Serial.print("Crash bike #:");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(second_current_time[i]/1000);
      Serial.print(" > ");
      Serial.print(speed_time[i]);
      break; //exit the larger loop
    }
  }
  Serial.println();


  // Transition Code
  v1 = !Sensor1;
  v2 = Sensor1;
  v3 = !Sensor2;
  v4 = Sensor2;
  //might want to just have a boolean set for this calculation to make sure that if any bike is not through in time then we have a crash
  v5 = !Sensor3 || !(we_have_crash);
  v6 = Sensor3 && !(we_have_crash);
  v7 = !Sensor3 && (we_have_crash);
  v8 = !resetPin;
  v9 = resetPin;

  // States
  S1 = v1 || v6 || v9; //waiting state
  S2 = v2 || v3; //tracking state between gates 1 and 2
  S3 = v4 || v5; //tracking state between gates 2 and 3
  S4 = v7 || v8; // Crashed state

  // Outputs
  // this else if statement turns the crash light on or off
  if (S4) {
    //send out the error signal to the light module somehow (not sure yet).
    digitalWrite(light_pin, HIGH);
  }
  else if (!S4) {
    digitalWrite(light_pin, LOW);
  }
  buttonState_old = buttonState;
//  resetState_old = resetState; 
}
