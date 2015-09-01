/*--------------------------------------------------------
CAN Bus Sequential Shiftlight                            
Author: Chris Crumpacker                               
Date: September 2015

Copyright (c) 2015 Chris Crumpacker.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
                                                           
Sketch Notes: 

The idea for this comes from Jon over at Chippernut.com
The wiring is basically the same and the fuctionality is simalar but 
the code has been reimagined from the groud up. I removed the 
7seg display and menu system here and hard coded the values. 
http://www.chippernut.com/blog/welcome

This uses a slightly modified version of the CAN bus library 
from sparkfun that I have included in this repo. There's will 
work for this sketch but I added other PID values for the other 
examples. Essentially this example just uses ODB PID requests 
over the CAN bus to get the RPM signal.

I'm using the Adafruit NeoPixel library that can be loaded 
in the aurdino IDE or found here: https://github.com/adafruit/Adafruit_NeoPixel

Version Notes:
v1.3s     Added optional "security" led states for when the vehicle is off
v1.2s     Switching canbus libraries
v1.1s     bug fixes
v1.0s     Stripped out all the code for the 7 segment display and encoder leaving only the 
          CANBus and the LED squential shift light.
v1.00     Finished full menu system and 7 seg
v0.14     Adding CANBus code
v0.12     Minor updates for stablity.
v0.11     New version for menu system and basic 7segment display and 
          LED strip control. Still driven by fake RPM delivered by the encoder.
  
To Do:

--------------------------------------------------------*/

#define PIN 6             // declares Digital Pin 6 as the output for the NeoPixel Display 
#define USE_SERIAL 1      // comment out this line to stop serial debuging

//-------------------------
// Includes
//-------------------------
#include <Adafruit_NeoPixel.h> 
#include <Canbus.h>

//-------------------------
// Library initialization
//-------------------------
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);      // Configure the neopixel strip and the matrix 7-seg 

//-------------------------
// Variables
//-------------------------
uint32_t 
  green = strip.Color(0, 255, 0),
  yellow = strip.Color(255, 255, 0),
  red = strip.Color(255, 0, 0),
  red2_3 = strip.Color(45, 0, 0),
  red1_3 = strip.Color(15, 0, 0),
  color[3] = {green, yellow, red};
  
char buffer[512];  //Data will be temporarily stored to this buffer before being written to the file
long prevBlinkTime = 0;

int 
  prev_range = 10,
  shiftPT[2] = {4200, 6200},  // point 0 = the activation point 1 = warning point
  ledStages[4] = {0,3,5,7},   // this is where each stage of the led strip is set. i.e. from ledStages[0] and ledStages[1] is stage one and so on
  warningState = 0,
  blinkInterval = 150,
  security_led_opt = 1,       // 0 = off, 1 = blink , 2 = knight rider
  stripBrightness = 189;      // 0 = off, 255 = fullbright

float 
  rpm, 
  prev_rpm;

//====================
// Setup
//====================
void setup() {
#ifdef USE_SERIAL  
  Serial.begin(115200);
  Serial.println("CANBus Shiftlight starting"); 
#endif    

  strip.begin();
  strip.setBrightness(stripBrightness);
  strip.show(); // Initialize all pixels to 'off' 
  knight_rider(1);
  
START_INIT:
  if(Canbus.init(CANSPEED_500))                   // init can bus : baudrate = 500k
  {
#ifdef USE_SERIAL     
    Serial.println("CAN BUS Shield init ok!");
#endif    
    blink_led(2, 150, color[0]);
  }
  else
  {
#ifdef USE_SERIAL 
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
#endif    
    blink_led(3, 500, color[2]);
    delay(1000);
    goto START_INIT;
  }
} 

//====================
// Main loop 
//====================
void loop() {
  
  if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1){
    ledStrip_update(atoi(buffer));
  }
  /*
  for(int i = 3000; i <7000; i++) {
    ledStrip_update(i);
    delay(10);
  }*/
}

// Updates the LED strip with the current RPMs
void ledStrip_update(uint16_t rpm) {
  unsigned long currentMillis = millis();
  
  if (rpm >= shiftPT[0] && rpm < shiftPT[1]) { //if the RPM is between the activation pt and the shift pt
    //map the RPM values to 9(really 8 since the shift point and beyond is handled below)and constrain the range
    int rpmMapped = map(rpm, shiftPT[0], shiftPT[1], 0, 8);
    int rpmConstrained = constrain(rpmMapped, 0, 8);
   
    if (prev_range != rpmConstrained) { //This makes it so we only update the LED when the range changes so we don't readdress the strip every reading
      prev_range = rpmConstrained;
#ifdef USE_SERIAL      
      Serial.print("RPM LED Range: ");
      Serial.println(rpmConstrained);
      Serial.print("RPM: ");
      Serial.println(rpm);
#endif     
      clearStrip();
      for (int ledNum = 0; ledNum <= rpmConstrained; ledNum++) {
        if (ledNum <= ledStages[1]) { strip.setPixelColor(ledNum, color[0]); }
        else if (ledNum > ledStages[1] && ledNum <= ledStages[2]) { strip.setPixelColor(ledNum, color[1]); }
        else if (ledNum > ledStages[2] && ledNum < strip.numPixels()) { strip.setPixelColor(ledNum, color[2]); }
      }
      strip.show();
    }
  }
  else if (rpm >= shiftPT[1]) { //SHIFT DAMNIT!! This blinks the LEDS back and forth with no delay to block button presses
    prev_range = 8;
    if (currentMillis - prevBlinkTime > blinkInterval){
      prevBlinkTime = currentMillis;
      
      if (warningState == 0){
        warningState = 1;
        for(int i = 0; i < strip.numPixels(); i=i+2){
          strip.setPixelColor(i, color[2]);
        }
        for(int i = 1; i < strip.numPixels(); i=i+2){
          strip.setPixelColor(i, 0);
        }
      }
      else {
        warningState = 0;
        for(int i = 1; i < strip.numPixels(); i=i+2){
          strip.setPixelColor(i, color[2]);
        }
        for(int i = 0; i < strip.numPixels(); i=i+2){
          strip.setPixelColor(i, 0);
        }
      }
      strip.show();
    }
  }
  else {
    if (prev_range != 10) {
      prev_range = 10;
      clearStrip();
#ifdef USE_SERIAL      
      Serial.println("Exiting strip range");
#endif      
    }
  }
}

//====================
// Clears the led strip
//====================
void clearStrip() {
  for (int ledNum = ledStages[0]; ledNum <= strip.numPixels(); ledNum++) {
    strip.setPixelColor(ledNum, 0);
  }
  strip.show(); 
}

//====================
// Blink the first led 
//====================
void blink_led(int count, int ms_delay, int colorInt) {
  clearStrip();
  strip.setBrightness(stripBrightness/4);
  for (int i = 0; i < count; i++) {
    strip.setPixelColor(0, colorInt);
    strip.show(); 
    delay(ms_delay);
    clearStrip();
    delay(ms_delay/2);
  }
  strip.setBrightness(stripBrightness);
  clearStrip();
}

//====================
// knight rider led sweep
//====================
void knight_rider(int count){
  for (int i = 0; i < count; i++) {
    for (int l = 0; l < strip.numPixels()+2; l++) {
      if(l > 0 && l < strip.numPixels()) { strip.setPixelColor(l, red); }
      if(l > 0 && l < strip.numPixels()+1) { strip.setPixelColor(l-1, red2_3); }
      if(l > 0 && l < strip.numPixels()+2) { strip.setPixelColor(l-2, red1_3); }
      strip.show();
      delay(75);
      clearStrip();
    }
    for (int y = strip.numPixels()-1; y >= -2 ; y--) {
      if(y >= 0 && y < strip.numPixels()) { strip.setPixelColor(y, red); }
      if(y >= -1 && y < strip.numPixels()-2) { strip.setPixelColor(y+1, red2_3); }
      if(y >= -2 && y < strip.numPixels()-3) { strip.setPixelColor(y+2, red1_3); }
      strip.show();
      delay(75);
      clearStrip();
    }
    if (count > 1) {
      delay(500);
    }
  }
}
