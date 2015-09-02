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
the code has been reimagined from the groud up. 
http://www.chippernut.com/blog/welcome

This uses a slightly modified version of the CAN bus library 
from sparkfun that I have included in this repo. There's will 
work for this sketch but I added other PID values for the other 
examples. Essentially this example just uses ODB PID requests 
over the CAN bus to get the RPM signal.

I'm using the Adafruit NeoPixel, GFX, and LED backpack libraries that can be loaded 
in the aurdino IDE or found here: 
https://github.com/adafruit/Adafruit_NeoPixel
https://github.com/adafruit/Adafruit-LED-Backpack-Library
https://github.com/adafruit/Adafruit-GFX-Library

I'm also using the ClickEncoder library (https://github.com/0xPIT/encoder/tree/arduino)
which also requires the use of the Timer1 library (http://playground.arduino.cc/Code/Timer1)

Version Notes:
v2.00 Added menu settings for changing the number of pixels and animation styles
v1.00 Final with full menu system and 7 seg
v0.14 Adding CANBus code
v0.12 Minor updates for stablity.
v0.11 Final revision for menu system and basic 7segment display and 
  LED strip control. Still driven by fake RPM delivered by the encoder.
  
To Do:
Test Center out animation
Debug option
--------------------------------------------------------*/

//-------------------------
// Includes
//-------------------------
#include <Wire.h> 
#include "Adafruit_LEDBackpack.h" 
#include "Adafruit_GFX.h" 
#include <Adafruit_NeoPixel.h> 
#include <EEPROM.h> 
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Canbus.h>

void(* resetFunc) (void) = 0;
// declares Digital Pin 6 as the output for the NeoPixel Display 
#define PIN 6 

// Configure the neopixel strip and the matrix 7-seg 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(EEPROM.read(11), PIN, NEO_GRB + NEO_KHZ800);
Adafruit_7segment matrix = Adafruit_7segment(); 
// Encoder lib settings encoder(<channel a pin>, <channel b pin>, <button pin, defaulted to high>, <encoder steps per click>)
ClickEncoder encoder(8, 7, A0, 4);

// Timer for the encoder service
void timerIsr() {
  encoder.service();
}

//-------------------------
// Variables
//-------------------------
uint32_t color[3];
int rpm;
int lastRPM = 0;
int prev_range = 10;
int NUMPIXELS = 8;
int animationStyle = 1;
char buffer[512];  //Data will be temporarily stored to this buffer before being written to the file

//These are stored memory variables for adjusting the (3) colors (wheel values), activation rpm, shift rpm, brightness 
//Stored in EEPROM Memory 
int wheelValues[3] = {255, 48, 86};
int shiftPT[2] = {4800, 6200};
int globalBrightness = 15;
int segBrightness = globalBrightness;
int stripBrightness = map(globalBrightness, 0, 15, 32, 255);
int shiftSaved1 = shiftPT[0]/100;
int shiftSaved2 = shiftPT[1]/100;
int ledStages[4] = {0,3,5, (NUMPIXELS - 1)};

// 7seg display masks in dec, this makes it easier to just add "128" to a digit mask to add the decimal point
char menuLabel[9][4] = {{57, 9, 9, 15}, {0, 124, 80, 120}, {0, 119, 57, 120}, {109, 118, 113, 120},
{57, 56, 51, 6}, {57, 56, 51, 91}, {57, 56, 51, 79}, {56,121,94,6}, {56,121,94,91}};
int chase[12][2] = {{4,4},{4,2},{4,1},{3,1},{1,1},{0,1},{0,32},{0,16},{0,8},{1,8},{3,8},{4,8}};

// Sleep settings
const int segDispRPMcutout = 1000;
boolean sleepState = false;
boolean sleepHold = false;

//LCD display update settings
long prevDispTime = 0;
long dispInterval = 100;
long prevBlinkTime = 0;
long blinkInterval = 150;
int warningState = 0;

//This subroutine reads the stored variables from memory 
void getEEPROM(){ 
  globalBrightness = EEPROM.read(1);
  shiftSaved1 = EEPROM.read(2);
  shiftSaved2 = EEPROM.read(3);
  wheelValues[0] = EEPROM.read(4); 
  wheelValues[1] = EEPROM.read(5); 
  wheelValues[2] = EEPROM.read(6);
  ledStages[0] = EEPROM.read(9);
  ledStages[1] = EEPROM.read(7);
  ledStages[2] = EEPROM.read(8);
  ledStages[3] = EEPROM.read(10);
  NUMPIXELS = EEPROM.read(11);
  animationStyle = EEPROM.read(12);
  
  color[0] = Wheel(wheelValues[0] & 255);
  color[1] = Wheel(wheelValues[1] & 255);
  color[2] = Wheel(wheelValues[2] & 255);
  shiftPT[0] = shiftSaved1 *100;
  shiftPT[1] = shiftSaved2 *100;
} 

//This subroutine writes the stored variables to memory 
void writeEEPROM(){ 
  shiftSaved1 = shiftPT[0] /100;
  shiftSaved2 = shiftPT[1] /100;
  EEPROM.write(1, globalBrightness); 
  EEPROM.write(2, shiftSaved1); 
  EEPROM.write(3, shiftSaved2); 
  EEPROM.write(4, wheelValues[0]); 
  EEPROM.write(5, wheelValues[1]); 
  EEPROM.write(6, wheelValues[2]); 
  EEPROM.write(9, ledStages[0]);
  EEPROM.write(7, ledStages[1]);
  EEPROM.write(8, ledStages[2]);
  EEPROM.write(10, ledStages[3]);
  EEPROM.write(11, NUMPIXELS);
  EEPROM.write(12, animationStyle);
}

//write just one variable to memory
void writeVarEEPROM(int spot, int var){
  EEPROM.write(spot, var);
}

// Clears both the 7segment display and the led strip by default or can be call for just one just
void clearDisplays(boolean seg = true, boolean LEDstrip = true) {
  if (seg) {
    matrix.clear(); 
    matrix.writeDisplay();
  }
  if (LEDstrip) {
    for( int i = 0; i < strip.numPixels(); i++){ 
      strip.setPixelColor(i, 0); 
    }
    strip.show(); 
  }
}

//====================
// Setup
//====================
void setup() { 
  Serial.begin(115200);
  Serial.println("CANBus Shiftlight starting");
  
  matrix.begin(0x70); 
  strip.begin(); 
  strip.show(); // Initialize all pixels to 'off' 

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  
  if(Canbus.init(CANSPEED_500))  /* Initialise MCP2515 CAN controller at the specified speed */
  {
    Serial.println("CAN Init ok");
  } else
  {
    Serial.println("Can't init CAN");
  } 
  
  //get stored variables 
  getEEPROM();  
  //writeEEPROM();
  
  //Set the 7-segment's brightness level
  segBrightness = globalBrightness;
  matrix.setBrightness(segBrightness);
  
  //Set the LED strip's brightness level
  stripBrightness = map(globalBrightness, 0, 15, 32, 255);
  strip.setBrightness(stripBrightness); 
  menu_transition();
} 

//====================
// Main loop 
//====================
void loop() {  
  if (!sleepHold) {// If it's not set to sleep mode
    if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1){
      rpm = atoi(buffer);
      if (rpm < segDispRPMcutout) { // if the RPM is above the cutout
      //only go into sleep mode if it isn't already sleeping
        if (sleepState == false) {
          sleepState = true;
          clearDisplays();
        }
      }
      else { //update the strip and 7segment display
        ledStrip_update(rpm);
        segDisplay_update(rpm);
      }
    }
  }
  
  //Check for button clicks
  switch (encoder.getButton()) {
    //single click enters menu
    case ClickEncoder::Clicked: //Enter menu
      menu_transition();
      menu();
      break;
    //double click sleeps the entire system
    case ClickEncoder::DoubleClicked:
      sleepHold = !sleepHold;
      clearDisplays();
      break;
  }
}

// Updates the 7 segment display with the current RPMs
void segDisplay_update(uint16_t rpm) {
  if (lastRPM != rpm) {
    lastRPM = rpm;
    unsigned long currentMillis = millis();
    //I don't want to update the 7segment display too often because it looks flickery then.
    if (currentMillis - prevDispTime > dispInterval){
      prevDispTime = currentMillis;
      Serial.println(rpm);
      matrix.println(rpm);
      matrix.writeDisplay();
    }
  }
}

// Updates the LED strip with the current RPMs
void ledStrip_update(uint16_t rpm) {
  unsigned long currentMillis = millis();
  if (rpm >= shiftPT[0] && rpm < shiftPT[1]) { //if the RPM is between the activation pt and the shift pt
    //map the RPM values to 9(really 8 since the shift point and beyond is handled below)and constrain the range
    int rpmMapped = map(rpm, shiftPT[0], shiftPT[1], 0, strip.numPixels());
    int rpmConstrained = constrain(rpmMapped, 0, strip.numPixels());
   
    if (prev_range != rpmConstrained) { //This makes it so we only update the LED when the range changes so we don't readdress the strip every reading
      prev_range = rpmConstrained;
      clearDisplays();
      switch(animationStyle){
        case 1: //Accending
          for (int ledNum = 0; ledNum <= rpmConstrained; ledNum++) {
            if (ledNum <= ledStages[1]) { strip.setPixelColor(ledNum, color[0]); }
            else if (ledNum > ledStages[1] && ledNum <= ledStages[2]) { strip.setPixelColor(ledNum, color[1]); }
            else if (ledNum > ledStages[2] && ledNum < strip.numPixels()) { strip.setPixelColor(ledNum, color[2]); }
          }
        break;
        
        case 2: //Center Out
          for (int ledNum = 0; ledNum <= strip.numPixels(); ledNum++) {
            if (ledNum >= ((strip.numPixels()/2)-ledStages[1]) || ledNum < ((strip.numPixels()/2)+ledStages[1])) { 
              strip.setPixelColor(ledNum, color[0]);
            }
            else if (
             ledNum < ((strip.numPixels()/2)-ledStages[1]) && ledNum >= ((strip.numPixels()/2)-ledStages[2]) ||
             ledNum > ((strip.numPixels()/2)+ledStages[1]) && ledNum <= ((strip.numPixels()/2)+ledStages[2])) {
              strip.setPixelColor(ledNum, color[1]); 
            }
            else if (
             ledNum < ((strip.numPixels()/2)-ledStages[2]) && ledNum >= ((strip.numPixels()/2)-ledStages[3]) ||
             ledNum > ((strip.numPixels()/2)+ledStages[2]) && ledNum <= ((strip.numPixels()/2)+ledStages[3])) {
              strip.setPixelColor(ledNum, color[2]); 
            }
          }
        break;
       
        case 3:
          for (int ledNum = 0; ledNum <= rpmConstrained; ledNum++) {
            if (ledNum <= ledStages[1]) { 
              strip.setPixelColor((strip.numPixels()-1-ledNum), color[0]);
            }
            else if (ledNum > ledStages[1] && ledNum <= ledStages[2]) { 
              strip.setPixelColor((strip.numPixels()-1-ledNum), color[1]);
            }
            else if (ledNum > ledStages[2] && ledNum < strip.numPixels()) { 
              strip.setPixelColor((strip.numPixels()-1-ledNum), color[2]);
            }
          }
        break;
      }
      strip.show();
    }
  }
  else if (rpm >= shiftPT[1]) { //SHIFT DAMNIT!! This blinks the LEDS back and forth with no delay to block button presses
    prev_range = strip.numPixels();
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
      clearDisplays();
    }
  }
}

void menu_transition() {
  clearDisplays();
  //Ascend strip 
  for (int i = 0; i < strip.numPixels(); i++){ 
  strip.setPixelColor(i, strip.Color(0, 0, 25)); 
  strip.show(); 
  delay(35); 
  }
  
  //chase matrix
  for (int i = 0; i < 12; i++) {
    matrix.writeDigitRaw(chase[i][0], chase[i][1]);
    matrix.writeDisplay();
    delay(20);
  }
  
  // Descend Strip 
  for (int i = 1; i < strip.numPixels(); i++){ 
  strip.setPixelColor(strip.numPixels()-i, 0);
  strip.show(); 
  delay(35); 
  }
  prev_range = 10;
  clearDisplays();
  lastRPM = 0;
}

void menu_next(int menu, boolean transition = false) {
  if(transition) { menu_transition(); } else { clearDisplays(); }
  
  matrix.writeDigitRaw(0, menuLabel[menu][0]);
  matrix.writeDigitRaw(1, menuLabel[menu][1]);
  matrix.writeDigitRaw(3, menuLabel[menu][2]);
  matrix.writeDigitRaw(4, menuLabel[menu][3]);
  matrix.writeDisplay();
  
  for (int ledNum =0; ledNum < menu; ledNum++) {
    strip.setPixelColor(ledNum, strip.Color(0, 0, 127));
  }
  strip.show();
}

// MENU SYSTEM 
void menu(){ 
  int menu = 0;
  int menuEnter = 0;
  int menuvar =1;
  
  menu_next(menu);
  
  //this keeps us in the menu 
  while (menuvar == 1){
    int menuMovement = encoder.getValue();
    if (menuMovement) {
      menu = (menuMovement > 0) ? menu+1 : menu-1;
      if (menu > 8) { menu = 0; }
      if (menu < 0) { menu = 8; }
      
      menu_next(menu);
    }
    int limitLED = menu - 6;
    int stage = menu -4;
    int shiftPos = menu - 2;
      
    switch (menu){ 
      case 0: //Menu Screen. Exiting saves variables to EEPROM         
        switch(encoder.getButton()) {
          case 5: // single click saves all settings to the EEPROM and exits
            writeEEPROM(); 
            getEEPROM(); 
            menu_transition();
            menuvar = 0;
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
      break; 
          
      case 1: //Adjust the global brightness 
        switch(encoder.getButton()) {
          case 5: // single click enters the menu selection
            menuEnter = 1;
            menu_transition();
            
            matrix.println(segBrightness); 
            matrix.writeDisplay();
            
            showStages();
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
        
        while (menuEnter == 1){
          int brightMovement = encoder.getValue();
          if (brightMovement) {
            //if movement is detected then we add or subtract depending on direction of the encoder rotation
            globalBrightness = (brightMovement > 0) ? globalBrightness+1 : globalBrightness-1;
            globalBrightness = constrain(globalBrightness, 0 , 15);
            
            segBrightness = globalBrightness;
            stripBrightness = map(globalBrightness, 0, 15, 32, 255);
            
            matrix.setBrightness(segBrightness); 
            matrix.println(segBrightness); 
            matrix.writeDisplay(); 

            clearDisplays(0,1);            
            strip.setBrightness(stripBrightness);
            showStages();
          }
          
          switch(encoder.getButton()) {
            case 5: // single click saves just this variable to the EEPROM and exits to the main menu
              writeVarEEPROM(menu, globalBrightness);
              menuEnter = 0; 
              menu_next(menu, true);
            break;
            case 6: // double click exits the menu without saving
              menu_transition();
              menuvar = 0;
              menuEnter = 0;
              getEEPROM(); // reloads unchanged values
            break;
          }  
        }
      break; 
      
      case 2: // Activation RPM 
      case 3: // Shift RPM
      int lowSPT, highSPT;
        switch(encoder.getButton()) {
          case 5: // single click enters the menu selection
            menuEnter = 1;
            menu_transition();
            matrix.println(shiftPT[shiftPos]); 
            matrix.writeDisplay();
            if (shiftPos ==0) { 
              lowSPT = segDispRPMcutout;
              highSPT = shiftPT[1];
            }
            else if ( shiftPos == 1) { 
              lowSPT = shiftPT[0];
              highSPT = 8000;
            }
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
        
        while (menuEnter == 1){     
          int actMovement = encoder.getValue();
          if (actMovement) {
            shiftPT[shiftPos] = (actMovement > 0) ? shiftPT[shiftPos]+100 : shiftPT[shiftPos]-100;
            shiftPT[shiftPos] = constrain(shiftPT[shiftPos], lowSPT , highSPT);
            matrix.println(shiftPT[shiftPos]); 
            matrix.writeDisplay();
          }
          
          switch(encoder.getButton()) {
            case 5: // single click saves just this variable to the EEPROM and exits to the main menu
              writeVarEEPROM(menu, shiftPT[shiftPos] /100);
              menuEnter = 0; 
              menu_next(menu, true);
            break;
            case 6: // double click exits the menu without saving
              menu_transition();
              menuvar = 0;
              menuEnter = 0;
              getEEPROM(); // reloads unchanged values
            break;
          } 
        }
      break;
      
      case 4: //Color 1
      case 5: //Color 2
      case 6: //Color 3
      int lowLED;
        switch(encoder.getButton()) {
          case 5: // single click enters the menu selection
            lowLED = (ledStages[stage]==0) ? ledStages[stage] : ledStages[stage]+1;
            menuEnter = 1;
            menu_transition();
            for (int ledNum = lowLED; ledNum <= ledStages[stage+1]; ledNum++) {
              strip.setPixelColor(ledNum, color[stage]);
            }
            strip.show();   
            matrix.println(wheelValues[stage]); 
            matrix.writeDisplay(); 
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
        
        while (menuEnter == 1){
          int colorMovement = encoder.getValue();
          if (colorMovement) {
            wheelValues[stage] = (colorMovement > 0) ? wheelValues[stage]+1 : wheelValues[stage]-1;
            if (wheelValues[stage] >= 255) { wheelValues[stage] = 0; }
            if (wheelValues[stage] < 0) { wheelValues[stage] = 255; }
            color[stage] = Wheel(wheelValues[stage] & 255);
            
            for (int ledNum = lowLED; ledNum <= ledStages[stage+1]; ledNum++) {
              strip.setPixelColor(ledNum, color[stage]);
            }
            strip.show();   
            matrix.println(wheelValues[stage]); 
            matrix.writeDisplay();    
          }
          
          switch(encoder.getButton()) {
            case 5: // single click saves just this variable to the EEPROM and exits to the main menu
              menuEnter = 0; 
              writeVarEEPROM(menu, color[stage]);
              menu_next(menu, true);
            break;
            case 6: // double click exits the menu without saving
              menu_transition();
              menuvar = 0;
              menuEnter = 0;
              getEEPROM(); // reloads unchanged values
            break;
          }   
        }
      break; 
      
      case 7: //stage serperator 1
      case 8: //stage serperator 2
        switch(encoder.getButton()) {
          case 5: // single click enters the menu selection
            menuEnter = 1;
            menu_transition();
            if(animationStyle == 2) {
              ledStages[limitLED] = constrain(ledStages[limitLED], 0 , (strip.numPixels()/2));
            }
            showStages();
            matrix.println(ledStages[limitLED]); 
            matrix.writeDisplay();
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
        
        while (menuEnter == 1){
          unsigned long currentMillis = millis();
          int limitLEDmov = encoder.getValue();
          if (limitLEDmov) {
            ledStages[limitLED] = (limitLEDmov > 0) ? ledStages[limitLED]+1 : ledStages[limitLED]-1;
            ledStages[limitLED] = constrain(ledStages[limitLED], ledStages[limitLED-1]+1 , ledStages[limitLED+1]-1);
              if(animationStyle == 2) {
                ledStages[limitLED] = constrain(ledStages[limitLED], 0 , (strip.numPixels()/2));
              }
            matrix.println(ledStages[limitLED]); 
            matrix.writeDisplay();
            
            showStages();
          }
          
          // blink the one we are changing
          if (currentMillis - prevBlinkTime > blinkInterval){
            prevBlinkTime = currentMillis;
            
            if (warningState == 0){
              warningState = 1;
              strip.setPixelColor(ledStages[limitLED], color[menu-7]);
            }
            else {
              warningState = 0;
              strip.setPixelColor(ledStages[limitLED], 0);
            }
            strip.show();
          }
          
          switch(encoder.getButton()) {
            case 5: // single click saves just this variable to the EEPROM and exits to the main menu
              writeVarEEPROM(menu, ledStages[limitLED]);
              menuEnter = 0;
              menu_next(menu, true);
            break;
            case 6: // double click exits the menu without saving
              menu_transition();
              menuvar = 0;
              getEEPROM(); // reloads unchanged values
            break;
          }
        }      
      break;
      
      case 9: //Number of total LEDs
        switch(encoder.getButton()) {
          case 5: // single click enters the menu selection
            menuEnter = 1;
            menu_transition();
            showStages();
            matrix.println(NUMPIXELS); 
            matrix.writeDisplay();
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
        
        while (menuEnter == 1){
          unsigned long currentMillis = millis();
          int NUMPIXELSmov = encoder.getValue();
          if (NUMPIXELSmov) {
            NUMPIXELS = (NUMPIXELSmov > 0) ? NUMPIXELS+1 : NUMPIXELS-1;
            NUMPIXELS = constrain(NUMPIXELS, 0 , 64);
            
            matrix.println(NUMPIXELS); 
            matrix.writeDisplay();
            
            showStages();
          }
          
          switch(encoder.getButton()) {
            case 5: // single click saves just this variable to the EEPROM and exits to the main menu
              writeVarEEPROM(menu, NUMPIXELS);
              resetFunc();
              menuEnter = 0;
              menu=7;
              menu_next(menu, true);
            break;
            case 6: // double click exits the menu without saving
              menu_transition();
              menuvar = 0;
              getEEPROM(); // reloads unchanged values
            break;
          }
        }      
      break;
      
      case 10: //Animation Style
        switch(encoder.getButton()) {
          case 5: // single click enters the menu selection
            menuEnter = 1;
            menu_transition();
            matrix.println(animationStyle); 
            matrix.writeDisplay();
          break;
          case 6: // double click exits the menu without saving
            menu_transition();
            menuvar = 0;
          break;
        }
        
        while (menuEnter == 1){
          unsigned long currentMillis = millis();
          int animationStylemov = encoder.getValue();
          if (animationStylemov) {
            animationStyle = (animationStylemov > 0) ? animationStyle+1 : animationStyle-1;
            animationStyle = constrain(animationStyle, 0 , 3);
            
            matrix.println(animationStyle); 
            matrix.writeDisplay();
          }
          
          switch(encoder.getButton()) {
            case 5: // single click saves just this variable to the EEPROM and exits to the main menu
              writeVarEEPROM(menu, animationStyle);
              menuEnter = 0;
              menu=7;
              menu_next(menu, true);
            break;
            case 6: // double click exits the menu without saving
              menu_transition();
              menuvar = 0;
              getEEPROM(); // reloads unchanged values
            break;
          }
        }   
      break;
    }
  }
}

// Lights up all the LEDs in their stage colors
void showStages() {
  clearDisplays(0,1);
  switch(animationStyle){
    case 1: //Accending
      for (int ledNum = 0; ledNum <= strip.numPixels(); ledNum++) {
        if (ledNum <= ledStages[1]) { strip.setPixelColor(ledNum, color[0]); }
        else if (ledNum > ledStages[1] && ledNum <= ledStages[2]) { strip.setPixelColor(ledNum, color[1]); }
        else if (ledNum > ledStages[2] && ledNum < strip.numPixels()) { strip.setPixelColor(ledNum, color[2]); }
      }
    break;
    
    case 2: //Center Out
      for (int ledNum = 0; ledNum <= strip.numPixels(); ledNum++) {
        if (ledNum >= ((strip.numPixels()/2)-ledStages[1]) || ledNum < ((strip.numPixels()/2)+ledStages[1])) { 
          strip.setPixelColor(ledNum, color[0]);
        }
        else if (
         ledNum < ((strip.numPixels()/2)-ledStages[1]) && ledNum >= ((strip.numPixels()/2)-ledStages[2]) ||
         ledNum > ((strip.numPixels()/2)+ledStages[1]) && ledNum <= ((strip.numPixels()/2)+ledStages[2])) {
          strip.setPixelColor(ledNum, color[1]); 
        }
        else if (
         ledNum < ((strip.numPixels()/2)-ledStages[2]) && ledNum >= ((strip.numPixels()/2)-ledStages[3]) ||
         ledNum > ((strip.numPixels()/2)+ledStages[2]) && ledNum <= ((strip.numPixels()/2)+ledStages[3])) {
           strip.setPixelColor(ledNum, color[2]); 
        }
      }
    break;
   
    case 3:
      for (int ledNum = 0; ledNum <= strip.numPixels(); ledNum++) {
        if (ledNum <= ledStages[1]) { 
          strip.setPixelColor((strip.numPixels()-1-ledNum), color[0]);
        }
        else if (ledNum > ledStages[1] && ledNum <= ledStages[2]) { 
          strip.setPixelColor((strip.numPixels()-1-ledNum), color[1]);
        }
        else if (ledNum > ledStages[2] && ledNum < strip.numPixels()) { 
          strip.setPixelColor((strip.numPixels()-1-ledNum), color[2]);
        }
      }
    break;
  }
  strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
