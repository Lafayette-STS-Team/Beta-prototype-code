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
