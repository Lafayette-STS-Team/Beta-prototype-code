//decalare variables for rising edge counter 
bool old_increment2;
bool old_decrement2;
bool old_reset2;
void counter2(int &value2, bool increment2, bool decrement2, bool reset2) {
  //need to add unique button press logic to make it a rising edge counter basically 
  // done with the counter reading issues so commented out the printing part
//  Serial.print("counter inputs: ");
//  Serial.print(increment);
//  Serial.print(decrement);
//  Serial.print(reset);
//  Serial.println();
  if (increment2 && !old_increment2) {
    value2++;
    
  }
  else if (decrement2 && !old_decrement2) {
    value2--;
  }
  else if (reset2 && !old_reset2) {
    value2 = 0;
  }
  old_increment2=increment2; 
  old_decrement2= decrement2; 
  old_reset2= reset2;
}
