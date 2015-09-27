/*--------------------------------------------------------
MULTI-DISPLAY 
CAN bus driven display using ODBII PIDs
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
Requesting data on the CAN bus using standard ODBII PIDs then displaying
the data on an OLED and RGB LED ring (NeoPixel)

Wiring:
- Button is a momentary high and connected to digital pin 3
- Neo pixel output is on digital pin 6
- OLED is controled via I2C

Libraries needed:
- Adafruit NeoPixel: https://github.com/adafruit/Adafruit_NeoPixel
- u8glib: https://code.google.com/p/u8glib/
- My version of the SkPang CANbus library: https://github.com/rainierez/CAN-Bus-Arduino-compatible

Version Notes:
v0.3 - Changing to the 16 pixel ring from adafruit
v0.2 - adding in more gauges and switching to floats for the data types

To Do:
- Add code to change the NeoPixels brightness based on Lux values.
--------------------------------------------------------*/

//-------------------------
// Includes
//-------------------------
#include <Wire.h>
#include <CrumpCanbus.h>
#include <Adafruit_NeoPixel.h>
#include "U8glib.h"
#include <BH1750FVI.h>

BH1750FVI LightSensor;

// declares pin 6 as the output for the NeoPixel Display 
#define PIN 6
#define NUMNEOPIXELS 16

// declares pin 3 as the input for the Button
#define BUTTON_PIN    5

// Initialize NeoPixel library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMNEOPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Initialize OLED library
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI 

//-------------------------
// Variables
//-------------------------

/* Stage points ====================================================================
Everything below the first stage point is out of range and won't trigger the LED strip.
Everthing above the second stage point is in the warning zone.
==================================================================================*/
int stagePT[2] = {4200, 6200};

/* LED levels =====================================================================
Between the active stage points above we have to break up the LEDs to levels
The outside numbers should always be the limit of the pixel strip, they are 0 aligned
==================================================================================*/
int ledStages[4] = {strip.numPixels()-1, 5, 2, 0}; //For top down direction i.e. leds 8->0

/* Colors for each stage ===========================================================
* {First level color, second, thrid and warning}
* 0 = green, 48 = yellow, 86 = red
==================================================================================*/
int wheelValues[3] = {0, 48, 86}; 

// 255 full bright to 32 most dim
int stripBrightness = 64;

//Button settings
boolean button_was_pressed = 1;     // the previous reading from the input pin
long debounceDelay = 25;    // the debounce time; increase if the output flickers

//OLED display update and LED settings
long prevDispTime = 0;
long dispInterval = 100;
long prevBlinkTime = 0;
long blinkInterval = 150;
int warningState = 0;
float lastDisplayValue = 0;
uint32_t color[3];
int prev_range = 100;
int gaugeType = 1;
boolean alarmOutsideRange = 1;

/*=== FUNCTION clearLEDs =============================================================
 * Purpose:
 * Clears the LED strip
 * 
 * Parameter Passed: void
 * 
 * Returns: void
================================================================================== */
void clearLEDs(){
  for( int i = 0; i < strip.numPixels(); i++){ 
    strip.setPixelColor(i, 0); 
  }
  strip.show(); 
}

//====================
// Setup
//====================
void setup() { 
  // Initial initialize the button pin to an input
  pinMode(BUTTON_PIN, INPUT);

  // Start the hardware serial
  Serial.begin(115200);
  Serial.println("CANBus Shiftlight starting");

  //Start up the NeoPixel Strip
  strip.begin(); 
  strip.show(); // Initialize all pixels to 'off' 
  strip.setBrightness(stripBrightness);

  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L);
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  
START_INIT:  
  // Start the CAN Bus
  if(Canbus.init(CANSPEED_500)) { /* Initialise MCP2515 CAN controller at the specified speed */
    u8g.firstPage();  
    do {
      u8g.setFont(u8g_font_fub11r);
      u8g.drawStr( 0, 11, "Starting");
      u8g.drawStr( 0, 40, "CAN OK" );
    } while( u8g.nextPage() );
    Serial.println("CAN Init ok");
  } else {
    u8g.firstPage();  
    do {
      u8g.setFont(u8g_font_fub11r);
      u8g.drawStr( 0, 11, "Starting");
      u8g.drawStr( 0, 40, "CAN Failed" );
    } while( u8g.nextPage() );
    Serial.println("CAN Failed");
    delay(5000);
    goto START_INIT;
  } 
  
  clearLEDs();
  delay(1000);
} 

/*=== FUNCTION handle_button=========================================================
 * Purpose:
 * Checks button state
 * 
 * Parameter Passed: void
 * 
 * Returns: boolean, true if button is in a different state than before
================================================================================== */
void handle_button() {
  boolean event;
  int button_now_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed

  event = button_now_pressed && !button_was_pressed;
  button_was_pressed = button_now_pressed;
  if(event) {
    gaugeType++;
    if(gaugeType > 8) {gaugeType =1;}
  }
}

/*=== FUNCTION set_levels=========================================================
 * Purpose:
 * Allows each different gauge to change the LED behavior settings
 * 
 * Parameter Passed: (each parameter is set to the default values for the Tachometer)
 * - int stagePT0 = 4200
 * - int stagePT1 = 6200
 * - int stage0 = 5
 * - int stage1 = 2
 * - int color0 = 255
 * - int color1 = 48
 * - int color2 = 86
 * - boolean alarm = 1
 * 
 * Returns: void
================================================================================== */
void set_levels(int stagePT0 = 4200, int stagePT1 = 6200, int stage0 = 5, int stage1 = 2, int color0 = 255, int color1 = 48, int color2 = 86, boolean alarm = 1){
  stagePT[0] = stagePT0;
  stagePT[1] = stagePT1;
  ledStages[1] = stage0;
  ledStages[2] = stage1;
  color[0] = Wheel(color0 & 255);
  color[1] = Wheel(color1 & 255);
  color[2] = Wheel(color2 & 255);
  alarmOutsideRange = alarm;
}

//====================
// Main loop 
//====================
void loop() {
  float rpm = 0.9, vss = 0.9, throtle = 0.9, maf = 0.9, manifoldAP = 0.9, bar = 0.9, coolant = 0.9, temp = 0.9, fuel = .9;
  
  switch(gaugeType){
    case 1: // RPM Shift light and Tachometer
      rpm = Canbus.ecu_req(ENGINE_RPM);
      if (rpm != .9){
        set_levels();
        OLED_update(rpm, 4, 0, "RPM");
        ledStrip_update(rpm);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 2: //Speedometer
      vss = Canbus.ecu_req(VEHICLE_SPEED);
      if (vss != .9){
        vss = vss * .62137119;
        set_levels(1,100);
        OLED_update(vss, 3, 1, "Speed (mph)");
        ledStrip_update(vss);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 3: // Fuel Economny, Instant MPG
      vss = Canbus.ecu_req(VEHICLE_SPEED);
      maf = Canbus.ecu_req(MAF_SENSOR);
      if (vss != .9 || maf != .9){
        float instantMPG = (14.7 * 6.17 * 454 * vss * .6213) / (3600 * maf);
        set_levels(1, 50, 14, 12, 48, 10, 0, 0);
        OLED_update(instantMPG, 4, 1, "Instant MPG");
        ledStrip_update(instantMPG);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 4: // Boost Gauge
    default:
      manifoldAP = Canbus.ecu_req(MAP);
      bar = Canbus.ecu_req(BAROMETRIC);
      if (manifoldAP != .9 || bar != .9){
        float boost = (manifoldAP - bar) * .1450377377;
        set_levels(-15, 10, 12, 7, 20, 10, 0, 0);
        OLED_update(boost, 4, 1, "Boost (psi)");
        ledStrip_update(boost);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 5: // Fuel level (Gas guague)
      fuel = Canbus.ecu_req(FUEL_LEVEL);
      if (fuel != .9){
        set_levels(1, 100, 14, 12, 86, 48, 0, 0);
        stagePT[0] = 1; 
        stagePT[1] = 100;
        ledStages[1] = strip.numPixels(); //all LEDs inside the stage 1
        OLED_update(fuel, 4, 1, "Fuel Level (%)");
        ledStrip_update(fuel);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 6: // Coolant temp ( most thermostats are triggered at 180 so alerting above that)
      coolant = Canbus.ecu_req(ENGINE_COOLANT_TEMP);
      if (coolant != .9){
        coolant = (coolant *1.8)+32;
        set_levels(190, 220, 5, 2, 0, 48, 86, 0);
        OLED_update(coolant, 4, 1, "Coolant (f)");
        ledStrip_update(coolant);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 7: // Ambient temp (in the engine compartment I think)
      temp = Canbus.ecu_req(AMBIENT_TEMP);
      if (temp != .9){
        temp = (temp *1.8)+32;
        set_levels(85, 110, 5, 2, 0, 48, 86,0);
        OLED_update(temp, 4, 1, "Ambient (f)");
        ledStrip_update(temp);
      } else {
          u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_fub11r);
            u8g.drawStr( 0, 11, "No Data");
          } while( u8g.nextPage() );
      }
    break;
    
    case 8: // Light intensity testing
      uint16_t lux = LightSensor.GetLightIntensity();// Get Lux value
      set_levels(0, 10000, 5, 2, 0, 48, 86,0);
      OLED_update(lux, 5, 0, "LUX");
      ledStrip_update(lux);
      delay(1000);
    break;
  }
  
  handle_button();
}

/*=== FUNCTION OLED_update=========================================================
 * Purpose:
 * Update the OLED with the new value to display
 * 
 * Parameter Passed:
 * float displayValue => the value to be displayed on the OLED
 * byte width => width (number of digits) of the value to be displayed
 * byte precision => Number of places after the decimal to display
 * char* label => the Label to be placed at the top of the display for the data type
 * 
 * Returns: void
================================================================================== */
void OLED_update(float displayValue, byte width, byte precision, char* label) {
  if (lastDisplayValue != displayValue) {
    lastDisplayValue = displayValue;
    unsigned long currentMillis = millis();
    //I don't want to update the 7segment display too often because it looks flickery then.
    if (currentMillis - prevDispTime > dispInterval){
      prevDispTime = currentMillis;
      u8g.firstPage();  
      do {
        char buf[10];
        char* displayValueStr = dtostrf(displayValue, width, precision, buf);
        // graphic commands to redraw the complete screen should be placed here  
        u8g.setFont(u8g_font_fub11r);
        u8g.drawStr( 0, 11, label);
      
        u8g.setFont(u8g_font_fub30r);
        int w = u8g.getStrWidth(displayValueStr);
        int startPos = 64-(w/2);
        u8g.drawStr( startPos, 50, displayValueStr );
      } while( u8g.nextPage() );
    }
  }
}


/*=== FUNCTION ledStrip_update=========================================================
 * Purpose:
 * Map current value inside the activation zone into levels for the 
 * number of LEDs and update the proper LED number and color
 * 
 * Parameter Passed:
 * float displayValue => the value to be mapped to the LED strip
 * 
 * Returns: void
================================================================================== */
void ledStrip_update(float displayValue) {
  unsigned long currentMillis = millis();
  
  if (displayValue >= stagePT[0] && (displayValue < stagePT[1] || alarmOutsideRange == 0)) { //if the RPM is between the activation pt and the shift pt
    //map the RPM values to 9(really 8 since the shift point and beyond is handled below)and constrain the range
    int displayValueMapped = map(displayValue, stagePT[0], stagePT[1], 0, strip.numPixels());
    int displayValueConstrained = constrain(displayValueMapped, 0, strip.numPixels());

    int numOfLEDs = strip.numPixels()-1- displayValueConstrained;
    if (prev_range != numOfLEDs) { //This makes it so we only update the LED when the range changes so we don't readdress the strip every reading
      prev_range = numOfLEDs;
      clearLEDs();
      for (int ledNum = strip.numPixels()-1; ledNum >= numOfLEDs; ledNum--) {
        Serial.print("ledNum: ");
        Serial.println(ledNum);
        if (ledNum >= ledStages[1]) { strip.setPixelColor(ledNum, color[0]); Serial.println("===1===");}
        else if (ledNum < ledStages[1] && ledNum >= ledStages[2]) { strip.setPixelColor(ledNum, color[1]); Serial.println("===2===");}
        else if (ledNum < ledStages[2]) { strip.setPixelColor(ledNum, color[2]); Serial.println("===3===");}
      }
      strip.show();
    }
  }
  else if (displayValue >= stagePT[1] && alarmOutsideRange) { //SHIFT DAMNIT!! This blinks the LEDS back and forth with no delay to block button presses
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
    if (prev_range != 100) {
      prev_range = 100;
      clearLEDs();
    }
  }
}

/*=== FUNCTION ledStrip_update=========================================================
 * Purpose:
 * using a 255 value color wheel to store the color values
 * The colors are a transition r - g - b - back to r.
 * 
 * Parameter Passed:
 * byte WheelPos => 0 to 255
 * 
 * Returns: int => the strip color in rgb
================================================================================== */
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
