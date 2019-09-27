# Data Logger (and using cool sensors!)

*A lab report by Ananya Paul.*

## In The Report

Include your responses to the bold questions on your own fork of [this lab report template](https://github.com/FAR-Lab/IDD-Fa18-Lab2). Include snippets of code that explain what you did. Deliverables are due next Tuesday. Post your lab reports as README.md pages on your GitHub, and post a link to that on your main class hub page.

For this lab, we will be experimenting with a variety of sensors, sending the data to the Arduino serial monitor, writing data to the EEPROM of the Arduino, and then playing the data back.

## Part A.  Writing to the Serial Monitor
 
**a. Based on the readings from the serial monitor, what is the range of the analog values being read?**

0 to 1023
 
**b. How many bits of resolution does the analog to digital converter (ADC) on the Arduino have?**

10 bits, 2^10

## Part B. RGB LED

**How might you use this with only the parts in your kit? Show us your solution.**
![Ohm resistors](./ohm_resistors.jpeg)

I used the Ohm resistors in the kit.

## Part C. Voltage Varying Sensors 
 
### 1. FSR, Flex Sensor, Photo cell, Softpot

**a. What voltage values do you see from your force sensor?**
From 0 to 1023

**b. What kind of relationship does the voltage have as a function of the force applied? (e.g., linear?)**

FSR and Softpot are Logorithmic.
Flex Sensor and PhotoCell are linear.

**c. Can you change the LED fading code values so that you get the full range of output voltages from the LED when using your FSR?**

Connected the common node of the anode to A0, where the voltage is controlled by the pressure resistor. 

**d. What resistance do you need to have in series to get a reasonable range of voltages from each sensor?**


**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**
FSR and Flex Sensor are Logorithmic.
Photo Cell and Softspot are Linear.

### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**
![LED reading input from X,Y,Z](https://youtu.be/Orq72ozvCmg)

```
// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup(void) {

#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
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
  
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  int redLED = 1;      
  int blueLED = 2;  
  int greenLED = 3; 
  analogWrite(redLED, event.acceleration.x);
  analogWrite(greenLED, event.acceleration.y);
  analogWrite(blueLED, event.acceleration.z);

  Serial.println(" m/s^2 ");

  Serial.println();
 
  delay(200); 
}
```
![Source](https://youtu.be/Orq72ozvCmg)

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**
Yes. 
![State Change](./2021.png)
A Read action should come right after the Write action to read values from the EEPROM. 

**b. Why is the code here all in the setup() functions and not in the loop() functions?**
To setup states everytime there is a switching of state. The setup is different for different states. We also want to take actions only when the state is changed and not on a loop constantly. 

**c. How many byte-sized data samples can you store on the Atmega328?**

1024 byte-sized data samples

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**
With int to byte char in Wire Library
```byte hiByte = highByte(wordVal);
byte loByte = lowByte(wordVal);
Serial.println(hiByte, HEX);
Serial.println(loByte,HEX);
```
[Source](https://forum.arduino.cc/index.php?topic=72993.0)

**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**

### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**
