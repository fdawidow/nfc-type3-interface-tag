/*
 * NFC Dynamic Tag / FeliCa Plug sample sketch "Simple_SmartPoster"
 * Implements a simple NFC SmartPoster pointing to http://blog.felicalauncher.com/en/?p=70 
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

#include <FeliCaPlug.h>
#include <inttypes.h>

FeliCaPlug plug;

// TT3 Attribute block + NDEF Message with SmartPoster http://blog.felicalauncher.com/en/?p=70 
uint8_t blockData[96] ={ \
0x10, 0x06, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00, 0x68, \
0xd1, 0x02, 0x41, 0x53, 0x70, 0x91, 0x01, 0x21, 0x55, 0x03, 0x62, 0x6c, 0x6f, 0x67, 0x2e, 0x66, \
0x65, 0x6c, 0x69, 0x63, 0x61, 0x6c, 0x61, 0x75, 0x6e, 0x63, 0x68, 0x65, 0x72, 0x2e, 0x63, 0x6f, \
0x6d, 0x2f, 0x65, 0x6e, 0x2f, 0x3f, 0x70, 0x3d, 0x37, 0x30, 0x51, 0x01, 0x18, 0x54, 0x02, 0x65, \
0x6e, 0x4e, 0x46, 0x43, 0x2d, 0x46, 0x20, 0x44, 0x65, 0x76, 0x65, 0x6c, 0x6f, 0x70, 0x65, 0x72, \
0x73, 0x20, 0x42, 0x6c, 0x6f, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  \
};  
uint16_t numOfBlocks = 6;

int readOnly = 0x00;

void setup()
{ 
  uint8_t dfc[2] = {0xff, 0xe0};
  uint8_t userParam[4] = {0x01, 0x23, 0x45, 0x67};
  
  plug.initPort(); 
  plug.setParam(FELICA_PLUG_MODE_TYPE3, dfc, userParam);
  plug.setLEDPin(FELICA_PLUG_LED_PIN);
  plug.setBlockData(blockData, numOfBlocks, readOnly); 
}

void loop()
{
  plug.doLoop();
}

