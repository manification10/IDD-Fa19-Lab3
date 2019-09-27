// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
const int EEPROMSIZE=1024;
int state,lastState = 0;

void setup(void) {
  EEPROM.write(0, 0);

#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  Serial.println(lis.begin(0x18));
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");
}

void loop() {
  lis.read();      // get X Y and Z data at once
  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);

  // set states
  /*
  States: 
  0. up-left: (y>0, x>0) Serial.print("/^v^\\");  Serial.print("    \t");
  1. up-right: (y>0, x<0) Serial.print("    \t"); Serial.print("/^v^\\");
  2. down-left: Serial.print("\\^v^/"); Serial.print("    \t");
  3. down-right: Serial.print("    \t"); Serial.print("\\^v^/"); 
  */
  
  if (event.acceleration.y > 0 && event.acceleration.x > 0) { state = 0; }
  if (event.acceleration.y > 0 && event.acceleration.x < 0) { state = 1; }
  if (event.acceleration.y < 0 && event.acceleration.x > 0) { state = 2; }
  if (event.acceleration.y < 0 && event.acceleration.x < 0) { state = 3; }
  
  // write to EEPROM
  if (state != lastState)
    { EEPROM.write(0, state); }
  
  // read state from EEPROM
  lastState = EEPROM.read(0);
  
  // call function
  Serial.println();
  Serial.print("    \t");
  switch (lastState) {
    case 0:    
      Serial.print("/^v^\\");  Serial.print("    \t");
      break;
    case 1:    
      Serial.print("    \t"); Serial.print("/^v^\\");
      break;
    case 2:    
      Serial.print("\\^v^/"); Serial.print("    \t");
      break;
    case 3:
      Serial.print("    \t"); Serial.print("\\^v^/");
      break;
    } 
 
  delay(100); 
}
