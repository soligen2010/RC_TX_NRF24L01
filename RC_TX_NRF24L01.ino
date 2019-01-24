
#include "Config.h" 
#include "TX.h" 
#include "Model_Config.h"


void setup(void) {
  Serial.begin(74880);
  Serial.println(); Serial.println(F("Starting"));
 
   setupTransmitter(); 

 Serial.println(F("Starting main loop ")); 

}


void loop() {  

   processTransmitter(); 
}






