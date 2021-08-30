#include <btAudioInternal.h>

// Minimum code to stream audio
btAudio audio = btAudio("jo awesome esp bt"); // change friendly name here

void setup() {
 
 // streams audio data to the ESP32   
 audio.begin();
 audio.InternalDAC();//turns dac on
}

void loop() {

}

