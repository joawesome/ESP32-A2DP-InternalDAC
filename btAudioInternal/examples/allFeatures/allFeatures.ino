#include <btAudioInternal.h>

btAudio audio = btAudio("JO awesome - allfeatures");
String command;

void setup() { 
 // streams audio data to the ESP32   
 audio.begin();
 audio.InternalDAC();//turns dac on
 Serial.begin(115200);
}

float thresh=40;
float attack=0.1;
float release= 0.2;
float ratio = 10;
float kneeWidth= 1;
float gain=12;
int fo=3;
float fc;


void loop() {
 delay(300);
 if(Serial.available()){
  command = Serial.readStringUntil('#');
   if(command.equals("compress")){
    Serial.println("Applying Compression");
    audio.compress(thresh,attack,release,ratio,kneeWidth,gain);
   }
   else if(command.equals("decompress")){
    Serial.println("Releasing Compression");
    audio.decompress();
   }
   else if(command.equals("gain")){
    gain =Serial.parseInt();
    Serial.println((String)"Compressing with a gain of "+gain);
    audio.compress(thresh,attack,release,ratio,kneeWidth,gain);
   }
   else if(command.equals("thresh")){
    thresh =Serial.parseFloat();
    Serial.println(thresh);
    Serial.println((String)"Compressing with a threshold of " + thresh + "dB");
    audio.compress(thresh,attack,release,ratio,kneeWidth,gain);
   }
   else if(command.equals("attack")){
    attack =Serial.parseFloat();
    Serial.println((String)"Compressing with an attack time of " + attack*1000 + "ms");
   audio.compress(thresh,attack,release,ratio,kneeWidth,gain); 
   }
   else if(command.equals("release")){
    release =Serial.parseFloat();
    Serial.println((String)"Compressing with an release time of " + release*1000 + "ms");
    audio.compress(thresh,attack,release,ratio,kneeWidth,gain);
   }
   else if(command.equals("ratio")){
    ratio =Serial.parseFloat();
    Serial.println((String)"Compressing with a Ratio of " + ratio);
    audio.compress(thresh,attack,release,ratio,kneeWidth,gain);
   }
   else if(command.equals("width")){
    kneeWidth =Serial.parseInt();
    Serial.println((String)"Compressing with a knee width of " + kneeWidth + "dB");
    audio.compress(thresh,attack,release,ratio,kneeWidth,gain);
   }
   else if(command.equals("hp")){
   Serial.println("Setting a High-Pass Filter...");
   fc =Serial.parseFloat();
   audio.createFilter(fo, fc, highpass);
  }
  else if(command.equals("stopFilt")){
   audio.stopFilter();
   Serial.println("Stopping Filter...");
  }
  else if(command.equals("order")){
   fo =Serial.parseInt();
   Serial.println((String)"Filter order changed to " + fo + "for next filter ");
  }  
  else if(command.equals("lp")){
   Serial.println("Setting a Low-Pass Filter...");
   fc =Serial.parseFloat();
   audio.createFilter(fo, fc, lowpass);
  }else{
   Serial.println("Command not recognised");
  }
 }
}