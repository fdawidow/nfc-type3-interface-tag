/*
 * NFC Dynamic Tag / FeliCa Plug sample sketch "Changing_SmartPoster"
 * Cycles through a number of NFC SmartPosters when a button is pressed
 *
 * Copyright 2010 Sony Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */



#include <EEPROM.h>
#include "FeliCaPlug.h"
#include <avr/pgmspace.h> <
#include <inttypes.h>
#define BUTTON_PIN 8  // Arduino Digital I/O pin where button is connected
#define NUM_BLOCKS 5  // capacity of NFC Tag in 16-byte blocks
#define NUM_TAGS 3    // number of NFC Tags in flash memory
FeliCaPlug plug;


PROGMEM const prog_uchar dataStorage[NUM_TAGS*NUM_BLOCKS*16] = {
// TT3 Attribute block + NDEF Message with SmartPoster http://blog.felicalauncher.com/en/
0x10, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x5c, \
0xd1, 0x02, 0x3b, 0x53, 0x70, 0x91, 0x01, 0x1c, 0x55, 0x03, 0x62, 0x6c, 0x6f, 0x67, 0x2e, 0x66, \
0x65, 0x6c, 0x69, 0x63, 0x61, 0x6c, 0x61, 0x75, 0x6e, 0x63, 0x68, 0x65, 0x72, 0x2e, 0x63, 0x6f, \
0x6d, 0x2f, 0x65, 0x6e, 0x2f, 0x51, 0x01, 0x17, 0x54, 0x02, 0x65, 0x6e, 0x4e, 0x46, 0x43, 0x2d, \
0x46, 0x20, 0x44, 0x65, 0x76, 0x65, 0x6c, 0x6f, 0x70, 0x65, 0x72, 0x20, 0x62, 0x6c, 0x6f, 0x67, \

// TT3 Attribute block + NDEF Message with SmartPoster http://launchpad.net/nfcpy
0x10, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x00, 0x55, \
0xd1, 0x02, 0x34, 0x53, 0x70, 0x91, 0x01, 0x14, 0x55, 0x03, 0x6c, 0x61, 0x75, 0x6e, 0x63, 0x68, \
0x70, 0x61, 0x64, 0x2e, 0x6e, 0x65, 0x74, 0x2f, 0x6e, 0x66, 0x63, 0x70, 0x79, 0x51, 0x01, 0x18, \
0x54, 0x02, 0x65, 0x6e, 0x50, 0x79, 0x74, 0x68, 0x6f, 0x6e, 0x20, 0x6d, 0x6f, 0x64, 0x75, 0x6c, \
0x65, 0x20, 0x66, 0x6f, 0x72, 0x20, 0x4e, 0x46, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \

// TT3 Attribute block + NDEF Message with SmartPoster http://arduino.cc/
0x10, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x00, 0x47, \
0xd1, 0x02, 0x26, 0x53, 0x70, 0x91, 0x01, 0x0c, 0x55, 0x03, 0x61, 0x72, 0x64, 0x75, 0x69, 0x6e, \
0x6f, 0x2e, 0x63, 0x63, 0x2f, 0x51, 0x01, 0x12, 0x54, 0x02, 0x65, 0x6e, 0x41, 0x72, 0x64, 0x75, \
0x69, 0x6e, 0x6f, 0x20, 0x50, 0x72, 0x6f, 0x6a, 0x65, 0x63, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
};

uint8_t blockData[NUM_BLOCKS*16];
uint16_t numOfBlocks = NUM_BLOCKS;
int idx = 0;

void setup()
{ 
  // initialize the NFC Dynamic Tag as NDEF Tag Type 3
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(115200);
  uint8_t dfc[2] = {0xff, 0xe0};
  uint8_t userParam[4] = {0x01, 0x23, 0x45, 0x67}; 
  
  plug.initPort(); 
  plug.setParam(FELICA_PLUG_MODE_TYPE3, dfc, userParam);
  plug.setLEDPin(FELICA_PLUG_LED_PIN);
  for(int i=0; i< numOfBlocks*16 ; i++)
    blockData[i] = (uint8_t)pgm_read_byte(&dataStorage[i]);
  plug.setBlockData(blockData, numOfBlocks, 0);
  Serial.println("Setup complete"); 
}

void loop()
{
  plug.doLoop(); 
  if(digitalRead(BUTTON_PIN) == LOW){
    Serial.println("triggered");    // now wait for button release 
    while(digitalRead(BUTTON_PIN)==LOW)
      delay(10);
   Serial.println("Button Pressed");
   if(++idx > NUM_TAGS-1)
     idx=0;
   Serial.println(idx);
  for(int i=0; i< numOfBlocks*16 ; i++)
      blockData[i] = (uint8_t)pgm_read_byte(&dataStorage[idx*numOfBlocks*16+i]);
  }  
}




