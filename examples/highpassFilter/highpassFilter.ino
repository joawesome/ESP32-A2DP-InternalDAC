#include <btAudioInternal.h>

btAudio audio = btAudio("ESP_Speaker");
String command;

void setup() { 
 audio.begin();
 int bck = 26; 
 int ws = 27;
 int dout = 25;
 audio.I2S(bck, dout, ws);
 Serial.begin(115200);
}


int fo=3;
float fc=30;

void loop() {
 delay(300);
 if(Serial.available()){
  command = Serial.readStringUntil('#');
  if(command.equals("hp")){
   Serial.println("Setting a High-Pass Filter...");
   fc =Serial.parseFloat();
   audio.createFilter(fo, fc, highpass);
  }
  else if(command.equals("stopFilt")){
   audio.stopFilter();
   Serial.println("Stopping Filter...");
  }   
 }
}
