//decalare variables for rising edge counter 
bool old_increment;
bool old_decrement;
bool old_reset;
void counter(int &value, bool increment, bool decrement, bool reset) {
  //need to add unique button press logic to make it a rising edge counter basically 
  // done with the counter reading issues so commented out the printing part
//  Serial.print("counter inputs: ");
//  Serial.print(increment);
//  Serial.print(decrement);
//  Serial.print(reset);
//  Serial.println();
  if (increment && !old_increment) {
    value++;
    
  }
  else if (decrement && !old_decrement) {
    value--;
  }
  else if (reset && !old_reset) {
    value = 0;
  }
  old_increment=increment; 
  old_decrement= decrement; 
  old_reset= reset;
}
