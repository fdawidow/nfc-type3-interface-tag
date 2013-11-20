/*
 * NFC Dynamic Tag / FeliCa Plug sample sketch "LCD_RW_Counter"
 * Emulates a writeable NFC Tag (Smartposter containing http://arduino.cc).
 * An attached LCD display shows the link and read/write counters that track 
 * the number of reads and writes detected by the NFC Dynamic Tag
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
#include "FeliCaPlug.h"
#include <inttypes.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(0,1,4,5,6,7,8); // (rs, rw, enable, d4, d5, d6, d7) 

FeliCaPlug plug;

// TT3 Attribute block + NDEF Message with SmartPoster http://arduino.cc/
uint8_t blockData[80] ={\
0x10, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x2b, 0x00, 0x48, \
0xd1, 0x02, 0x26, 0x53, 0x70, 0x91, 0x01, 0x0c, 0x55, 0x03, 0x61, 0x72, 0x64, 0x75, 0x69, 0x6e, \
0x6f, 0x2e, 0x63, 0x63, 0x2f, 0x51, 0x01, 0x12, 0x54, 0x02, 0x65, 0x6e, 0x41, 0x72, 0x64, 0x75, \
0x69, 0x6e, 0x6f, 0x20, 0x50, 0x72, 0x6f, 0x6a, 0x65, 0x63, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
};
uint16_t numOfBlocks = 5;
int readOnly = 0x00;
uint8_t cmd;
char* url;
int readCounter = 0;
int writeCounter = 0;


void setup()
{ 
  
  uint8_t dfc[2] = {0xff, 0xe0};
  uint8_t userParam[4] = {0x01, 0x23, 0x45, 0x67};
  
  plug.initPort();
  
  lcd.begin(4, 20) ;
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("NFC-SmartPoster");

  
  plug.setParam(FELICA_PLUG_MODE_TYPE3, dfc, userParam);
  plug.setLEDPin(FELICA_PLUG_LED_PIN);
  plug.setBlockData(blockData, numOfBlocks, readOnly); 
  plug.setFTWriteCallback(ftWriteCallback);
  plug.setFTReadCallback(ftReadCallback);   
}


void loop()
{
  plug.doLoop();
  updateLCD();
  delay(10);
}


void ftWriteCallback(uint16_t blockNum, uint8_t* data)
{
  if(blockNum == 1)
    writeCounter ++;
  memcpy(blockData+blockNum*16, data, 16);
}

void ftReadCallback(uint16_t blockNum, uint8_t* data)
{
  if(blockNum == 1)
    readCounter++;
  memcpy(data, blockData+blockNum*16, 16);

}

void updateLCD()
{
  int remainingDigits=16;
  int len=0;
  lcd.setCursor(0,1);
  lcd.clear();

  for(int i = 0; i < numOfBlocks*16; i++)
  {
    if(blockData[i]=='U')
    {
      len=blockData[i-1]-1; // length with out Identifier code
      for(int j = i+2;((j<(i+2)+16)&&(j<(i+2)+len));j++)
      {
        lcd.write(blockData[j]);
        --remainingDigits;
      }
    }
  }

  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(readCounter);

  lcd.print("  W:");
  lcd.print(writeCounter);

}

