#include <EEPROM.h>

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  
  for(int i=0; i < 512; i++){
    EEPROM.write(i, 0);
    Serial.print(EEPROM.read(i));
    Serial.print(" ");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
