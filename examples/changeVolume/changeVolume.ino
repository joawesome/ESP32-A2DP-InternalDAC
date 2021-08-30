#include <btAudioInternal.h>

// Sets the name of the audio device
btAudio audio = btAudio("jo awesome volume");

void setup() {
 
 // streams audio data to the ESP32   
 audio.begin();
 audio.InternalDAC();//turns dac 
}

void loop() {
delay(3000);
audio.volume(0.1);
delay(3000);
audio.volume(1.0);
}#include <btAudioInternal.h>

// Sets the name of the audio device
btAudio audio = btAudio("ESP_Speaker");

void setup() {
 
 // streams audio data to the ESP32   
 audio.begin();
 audio.InternalDAC();//turns dac 
}

void loop() {
delay(3000);
audio.volume(0.1);
delay(3000);
audio.volume(1.0);
}