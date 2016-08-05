

int reads[15];
float volts[15];

void setup(){

  Serial.begin(9600); //initializes the serial port with the pc
  
  for(byte i=0;i<15;i++){ //initializes the array to 0s
    reads[i]=0; 
  }
}


void loop(){
  
  for(byte i=0;i<15;i++){ //reads from pin 0 to pin 14
    reads[i]=analogRead(i);
  }
  
  //String buffer;
  //for(byte i=0;i<15;i++){ //converts values to string
  // String theRead=String( reads[0]);
  // buffer.concat(theRead);
  // buffer.concat(" ");
  //}
  //Serial.println(buffer);
  //Serial.println(theRead);
  
  for(byte i=0;i<15;i++){ //converts integers into volts
    volts[i]=reads[i]*(5.0 / 1023.0);
  }
  
  for(byte i=0;i<15;i++){ //sends numbers as a line of text
    Serial.print(volts[i]);
    Serial.print(" ");
  }
  Serial.println();
  //Serial.println(volts[0]);
}
