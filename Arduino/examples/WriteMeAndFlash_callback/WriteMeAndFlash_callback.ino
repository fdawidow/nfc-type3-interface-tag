/*
 * NFC Dynamic Tag / FeliCa Plug sample sketch "WriteMeAndFlash_callback"
 * Emulates a writeable NFC Tag (NDEF Text record containing "Write on me,
 * and I will blink for you. Reset device to start over." 
 * Once an NFC device writes content to the tag, the LED flashes until the 
 * button attached to I/O pin BUTTON_PIN is pressed.
 * The "write" is detected in the write callback function ftWriteCallback()
 *
 * Copyright 2012 Sony Corporation
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
#define NUM_OF_BLOCKS 13 // capacity of NFC Tag in 16-byte blocks

#include "FeliCaPlug.h"
#include <avr/pgmspace.h> <
#include <inttypes.h>

FeliCaPlug plug;

PROGMEM const prog_uchar dataStorage[NUM_OF_BLOCKS*16] = {
// TT3 Attribute block + NDEF Message Text Message 
0x10, 0x08, 0x08, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x49, 0x00, 0x77, \
0xd1, 0x01, 0x45, 0x54, 0x02, 0x65, 0x6e, 0x57, 0x72, 0x69, 0x74, 0x65, 0x20, 0x6f, 0x6e, 0x20, \
0x6d, 0x65, 0x2c, 0x20, 0x61, 0x6e, 0x64, 0x20, 0x49, 0x20, 0x77, 0x69, 0x6c, 0x6c, 0x20, 0x62, \
0x6c, 0x69, 0x6e, 0x6b, 0x20, 0x66, 0x6f, 0x72, 0x20, 0x79, 0x6f, 0x75, 0x2e, 0x20, 0x52, 0x65, \
0x73, 0x65, 0x74, 0x20, 0x64, 0x65, 0x76, 0x69, 0x63, 0x65, 0x20, 0x74, 0x6f, 0x20, 0x73, 0x74, \
0x61, 0x72, 0x74, 0x20, 0x6f, 0x76, 0x65, 0x72, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
};

uint8_t blockData[NUM_OF_BLOCKS*16];
uint16_t numOfBlocks = NUM_OF_BLOCKS;
boolean dataChanged = false;

void setup()
{ 
  Serial.begin(115200);
  uint8_t dfc[2] = {0xff, 0xe0};
  uint8_t userParam[4] = {0x01, 0x23, 0x45, 0x67};
  dataChanged = false; 
  
  plug.initPort(); 
  plug.setParam(FELICA_PLUG_MODE_TYPE3, dfc, userParam);
  plug.setLEDPin(FELICA_PLUG_LED_PIN);
  for(int i=0; i< numOfBlocks*16 ; i++)
    blockData[i] = (uint8_t)pgm_read_byte(&dataStorage[i]);
  plug.setBlockData(blockData, numOfBlocks, 0);
  plug.setFTWriteCallback(ftWriteCallback); // initialize write callback 
  Serial.println("Setup complete"); 
  Serial.println("Write data on NFC Tag to start"); 
}

void loop()
{
  plug.doLoop();
  Serial.print(".");
  if(dataChanged){
    Serial.println("#modified data detected#");
    digitalWrite(FELICA_PLUG_LED_PIN, LOW);
    delay(50);
    digitalWrite(FELICA_PLUG_LED_PIN, HIGH);
  }
}

void ftWriteCallback(uint16_t blockNum, uint8_t* data)
{  
  dataChanged = true;
  memcpy(blockData+blockNum*16, data, 16);
}



