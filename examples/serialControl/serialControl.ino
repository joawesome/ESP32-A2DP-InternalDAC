#include <btAudioInternal.h>

btAudio audio = btAudio("Jo awesome - serial");// set your own name here
String command;

void setup() { 
 // streams audio data to the ESP32   
 audio.begin();
 audio.InternalDAC();//turns dac on
 Serial.begin(115200);
}


void loop() {
 delay(300);
 // check if data is available 
 if(Serial.available()){
  //read until a terminator. after this point there should only be numbers
  command = Serial.readStringUntil('#');
  if(command.equals("vol")){
   //read and set volume	  
   float vol =Serial.parseFloat();
   Serial.println("Changing Volume");
   audio.volume(vol/100);
  }
 }
}