/*
 * NFC Dynamic Tag / FeliCa Plug sample sketch "NFCTemperatureLamp"
 * Implements a temperature-controled RGB lamp that outputs temperature history 
 * through NFC. The setup consists of an Arduino MEGA, a temperature sensor, 
 * three RGB LEDs, a button to switch on/off the LEDs, 
 * and a Sony RC-S801 NFC Dynamic Tag  
 
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
#define MAX_HIST_NUMBER 32
#define HIST_INTERVAL 3600000UL /* in milliseconds, 1000UL is one second, 3600000UL one hour */

#define IN_ANALOG_PIN A0
#define IN_BUTTON_PIN 8
#define OUT_LED_RED 4
#define OUT_LED_GREEN 5
#define OUT_LED_BLUE 6

#define TEMP_CORRECT 9 // used to calibrate temp sensor
#define TEMP_OPT_HIGH 46 // celsius *2 
#define TEMP_OPT_LOW 40 // celsius *2 
#define TEMP_LOW 30
#define TEMP_HIGH 60

#include <FeliCaPlug.h>

byte histIdx;
char tempHistory[MAX_HIST_NUMBER];

char currentTempValue;
uint8_t allDark;
unsigned long timeRef;
unsigned long currentTime;
unsigned long delta;
unsigned long time;

float sensorValue;
char startup;

uint8_t blockData[48+MAX_HIST_NUMBER];
uint16_t numOfBlocks = 5; 
FeliCaPlug plug;

void setup() {
  timeRef=0UL;
  currentTime=0UL;
  delta=0UL;

  allDark=false;
  pinMode(IN_ANALOG_PIN, INPUT);
  pinMode(IN_BUTTON_PIN, INPUT);
  pinMode(OUT_LED_RED, OUTPUT);
  pinMode(OUT_LED_GREEN, OUTPUT);
  pinMode(OUT_LED_BLUE, OUTPUT);
  Serial.begin(115200);
  
  uint8_t readOnly = 0x00;
  uint8_t dfc[2] = {0xff, 0xe0};
  uint8_t userParam[4] = {0x01, 0x23, 0x45, 0x67};
  
  plug.initPort();  
  plug.setParam(FELICA_PLUG_MODE_TYPE3, dfc, userParam);
  plug.setLEDPin(FELICA_PLUG_LED_PIN);
  plug.setBlockData(blockData, numOfBlocks, readOnly);
  
  histIdx=0; // index of history values. Reset with every restart
  startup=true;
  
  Serial.println("Setup complete!");  
}


void loop() 
{
  delay(10);
  plug.doLoop();  
  /* read temperature sensor sensor */
  currentTempValue =  (char) (((analogRead(IN_ANALOG_PIN)+ analogRead(IN_ANALOG_PIN) + analogRead(IN_ANALOG_PIN)+ analogRead(IN_ANALOG_PIN))/4)-TEMP_CORRECT); // this should get us the temperature in Celsius * 2
  
  /* check button and trigger turning on/off light*/
  if(digitalRead(IN_BUTTON_PIN)==HIGH){
    // now wait for button release 
    while(digitalRead(IN_BUTTON_PIN)==HIGH)
    {
      delay(10);
    }
    if(allDark==true){
      allDark=false;
    }else{
      allDark=true;
    }
  }
  
  /* let's switch on the LEDs */
  if(allDark==true){
    /* all dark */
    analogWrite(OUT_LED_RED, 255);
    analogWrite(OUT_LED_GREEN, 255);
    analogWrite(OUT_LED_BLUE, 255);
  }else{
    if(currentTempValue > TEMP_HIGH)
    {
      /* red */
      analogWrite(OUT_LED_RED, 0);
      analogWrite(OUT_LED_GREEN, 255);
      analogWrite(OUT_LED_BLUE, 255);
    }
    else if (currentTempValue < TEMP_LOW)
    {
      /* blue */
    analogWrite(OUT_LED_RED, 255);
    analogWrite(OUT_LED_GREEN, 255);
    analogWrite(OUT_LED_BLUE, 0);    
    }
    else // this is the fading area 
    {
      if((currentTempValue <= TEMP_HIGH) && (currentTempValue >= TEMP_OPT_HIGH)){
        /* fading red */
        analogWrite(OUT_LED_RED, (255*(TEMP_HIGH-currentTempValue)/(TEMP_HIGH-TEMP_OPT_HIGH)));
        analogWrite(OUT_LED_GREEN, (255*(currentTempValue-TEMP_OPT_HIGH)/(TEMP_HIGH-TEMP_OPT_HIGH)));
        analogWrite(OUT_LED_BLUE, 255);
      }else if((currentTempValue >= TEMP_LOW) && (currentTempValue <= TEMP_OPT_LOW)){
        /* fading blue */
        analogWrite(OUT_LED_RED, 255);
        analogWrite(OUT_LED_GREEN, (255*(TEMP_OPT_LOW-currentTempValue)/(TEMP_OPT_LOW-TEMP_LOW)));
        analogWrite(OUT_LED_BLUE, (255*(currentTempValue-TEMP_LOW)/(TEMP_OPT_LOW-TEMP_LOW)));
      }else{
        /* green */
        analogWrite(OUT_LED_RED, 255);
        analogWrite(OUT_LED_GREEN, 0);
        analogWrite(OUT_LED_BLUE, 255);
      }
    } 


//   update history on NFC tag in a certain interval devined by HIST_INTERVAL
  currentTime = millis();
  delta = currentTime - timeRef;
  if(delta >= HIST_INTERVAL || startup==true) 
  {
    timeRef = millis();
    startup=false;

    tempHistory[histIdx]=currentTempValue;
    if(++histIdx >= MAX_HIST_NUMBER)
      histIdx=0;    
    ConfigureNDEF(TEMP_OPT_HIGH-TEMP_OPT_LOW, TEMP_HIGH, TEMP_LOW, currentTempValue);

    }
    
// for debugging   
//    Serial.print(" V: ");
//    Serial.print(currentTempValue, DEC);
//    Serial.print(" OL: ");
//    Serial.print(TEMP_OPT_LOW, DEC);
//    Serial.print(" OH: ");
//    Serial.print(TEMP_OPT_HIGH, DEC);
//    Serial.print(" H: ");
//    Serial.print(TEMP_HIGH, DEC);
//    Serial.print(" L: ");
//    Serial.print(TEMP_LOW, DEC);
//    Serial.println();
//    Serial.print(" I: ");
//    Serial.print(histIdx, DEC);
//    Serial.println(); 
  }
}


void ConfigureNDEF(char opt, char high, char low, char current)
{
    /* attribute data */
    blockData[0]=0x10; // version field
    blockData[1]=0x08; // Nbr
    blockData[2]=0x08; // Nbw
    blockData[3]=0x00; //Nmaxb
    blockData[4]=0x0B; //Nmaxb (11 NDEF blocks max)
    blockData[5]=0x00; //unused
    blockData[6]=0x00; //unused
    blockData[7]=0x00; //unused
    blockData[8]=0x00; //unused
    blockData[9]=0x00; //WriteF
    blockData[10]=0x01; //RW Flag
    blockData[11]=0x00; //Ln
    blockData[12]=0x00; //Ln
    blockData[13]=0x40; //Ln
    blockData[14]=0x00; //Checksum
    blockData[15]=0x6c; //Checksum
    
    /* NDEF HEADER  and initial payload */
    blockData[16]=0xD4; // NDEF geader byte short record, ext type
    blockData[17]=0xA; //Type length for sony.de:tm
    blockData[18]=19+MAX_HIST_NUMBER; // 3 RFU + 1 block config data  and MAX_HIST_NUMBER (32) history values 
    blockData[19]='s'; // type [10 bytes]
    blockData[20]='o';
    blockData[21]='n';
    blockData[22]='y';
    blockData[23]='.';
    blockData[24]='d';
    blockData[25]='e';
    blockData[26]=':';
    blockData[27]='t';
    blockData[28]='m';
    blockData[29]=0; // RFU
    blockData[30]=0; // RFU
    blockData[31]=current; // current temperature
    blockData[32]=opt; //optimum temperature 21°C (celsius * 2)
    blockData[33]=high; //high temperature 21°C (celsius * 2)
    blockData[34]=low; //low temperature 21°C (celsius * 2)
    blockData[35]=histIdx; // history index; // RFU
    if(blockData[35] >= MAX_HIST_NUMBER)
      blockData[35]=0;
    blockData[36]=0; // RFU
    blockData[37]=0; // RFU
    blockData[38]=0; // RFU
    blockData[39]=0; // RFU
    blockData[40]=0; // RFU
    blockData[41]=0; // RFU
    blockData[42]=0; // RFU
    blockData[43]=0; // RFU
    blockData[44]=0; // RFU
    blockData[45]=0; // RFU
    blockData[46]=0; // RFU
    blockData[47]=0; // RFU
    
    /* read history values from RAM  */
    memcpy(blockData+48,tempHistory,MAX_HIST_NUMBER);
}


