/*
 * FeliCaPlug library for Arduino
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


#include <inttypes.h>

#ifndef FELICA_PLUG_H_
#define FELICA_PLUG_H_


/* --------------------------------
 * Constant
 * -------------------------------- */
#ifdef FELICA_PLUG_ALTERNATIVE_PIN_LAYOUT
#define FELICA_PLUG_SW_PIN          9
#define FELICA_PLUG_SEL_PIN         4
#define FELICA_PLUG_DATA_PIN        5
#define FELICA_PLUG_SPICLK_PIN      6
#define FELICA_PLUG_IRQ_PIN         2
#define FELICA_PLUG_RFDET_PIN       3

#else
#define FELICA_PLUG_SW_PIN          9
#define FELICA_PLUG_SEL_PIN         10
#define FELICA_PLUG_DATA_PIN        11
#define FELICA_PLUG_SPICLK_PIN      12
#define FELICA_PLUG_IRQ_PIN         2
#define FELICA_PLUG_RFDET_PIN       3
#endif
#define FELICA_PLUG_LED_PIN         13
#define FELICA_PLUG_LED_PIN_NONE    0xff

#define FELICA_PLUG_NO_RF_SUSPEND_TIME      3000 /* ms */
#define FELICA_PLUG_NO_ACTION_INTERVAL      100  /* ms */

#define FELICA_PLUG_MODE_FB         0x18
#define FELICA_PLUG_MODE_FT         0x1a
#define FELICA_PLUG_MODE_TYPE3      0x1b
#define FELICA_PLUG_MODE_NONE       0x00

#define FELICA_PLUG_PMM_READ        0xff
#define FELICA_PLUG_PMM_WRITE       0xff

#define FELICA_PLUG_EEPROM_NUM_OF_BLOCKS    31

/* --------------------------------
 * Type and Structure
 * -------------------------------- */

typedef void (*FeliCaPlug_common_callback_t)(void);

typedef void (*FeliCaPlug_FB_callback_t)(
    uint8_t* numOfBlocks,
    const uint8_t* blockList,
    uint8_t* blockData);

typedef void (*FeliCaPlug_FT_callback_t)(
    uint16_t blockNum,
    uint8_t* blockData);

/* --------------------------------
 * Class Declaration
 * -------------------------------- */

class FeliCaPlug
{
public:
    void initPort(
        uint8_t swPin = FELICA_PLUG_SW_PIN,
        uint8_t selPin = FELICA_PLUG_SEL_PIN,
        uint8_t dataPin = FELICA_PLUG_DATA_PIN,
        uint8_t spiclkPin = FELICA_PLUG_SPICLK_PIN,
        uint8_t irqPin = FELICA_PLUG_IRQ_PIN,
        uint8_t rfdetPin = FELICA_PLUG_RFDET_PIN);
    void setParam(
        uint8_t mode,
        uint8_t dfc[2],
        uint8_t userParam[4],
        uint8_t pmmRead = FELICA_PLUG_PMM_READ,
        uint8_t pmmWrite = FELICA_PLUG_PMM_WRITE);
    void setLEDPin(uint8_t ledPin);
    void setBlockData(
        uint8_t* blockData,
        uint16_t numOfBlocks,
        int readOnly = 0);
    int setSyncEEPROM(
        int on = 1,
        uint16_t maxNumOfBlocks = FELICA_PLUG_EEPROM_NUM_OF_BLOCKS);
    void setSuspendCallback(FeliCaPlug_common_callback_t func);
    void setResumeCallback(FeliCaPlug_common_callback_t func);
    void setFBCallback(FeliCaPlug_FB_callback_t func);
    void setFTReadCallback(FeliCaPlug_FT_callback_t func);
    void setFTWriteCallback(FeliCaPlug_FT_callback_t func);

    void doLoop(unsigned long noActionInterval =
                FELICA_PLUG_NO_ACTION_INTERVAL);

private:
    void changeState(uint8_t newState);

    void initDevice(void);
    void execCmdFB(void);
    void execCmdFT(void);
    void execCmdFTRead(void);
    void execCmdFTWrite(void);
    uint8_t checkReadBlock(
        uint16_t blockNum,
        uint8_t* blockData);
    void readBlock(
        uint16_t blockNum,
        uint8_t* readBlockData);
    uint8_t checkWriteBlock(
        uint16_t blockNum,
        const uint8_t* blockData);
    void writeBlock(
        uint16_t blockNum,
        uint8_t* blockData);

    void suspend(void);
    void resume(void);
    void beginSend(void);
    void endSend(void);
    void sendByte(uint8_t data);
    void sendByteArray(
        const uint8_t* data,
        uint8_t length);
    uint8_t receiveByte(void);
    void receiveByteArray(
        uint8_t* data,
        uint8_t length);
    void updateLastRFDetTime(void);

    int readBlockDataFromEEPROM(void);
    void writeBlockDataToEEPROM(void);
    void writeEEPROMIfChanged(
        int addr,
        uint8_t value);

private:
    uint8_t swPin;
    uint8_t selPin;
    uint8_t dataPin;
    uint8_t spiclkPin;
    uint8_t irqPin;
    uint8_t rfdetPin;

    uint8_t ledPin;

    uint8_t mode;
    uint8_t state;

    uint8_t dfc[2];
    uint8_t userParam[4];
    uint8_t pmmRead;
    uint8_t pmmWrite;

    unsigned long lastRFDetTime;
    unsigned long noRFSuspendTime;

    uint8_t* blockData;
    uint16_t numOfBlocks;
    int readOnly;

    FeliCaPlug_common_callback_t suspendCallback;
    FeliCaPlug_common_callback_t resumeCallback;
    FeliCaPlug_FB_callback_t fbCallback;
    FeliCaPlug_FT_callback_t ftReadCallback;
    FeliCaPlug_FT_callback_t ftWriteCallback;

    int syncEEPROM;
    static uint8_t blockDataEEPROM[];
};

#endif /* !FELICA_PLUG_H_ */
