/*
 * mcp9808.h
 *
 *  Created on: Mar 6, 2019
 *      Author: Nicholas
 */

#ifndef MCP9808_H_
#define MCP9808_H_

#include "msp.h"
#include <stdint.h>

enum mcp9808_registers {MCP9808_CONFIG = 0b0001,
                        MCP9808_TUPPER = 0b0010,
                        MCP9808_TLOWER = 0b0011,
                        MCP9808_TCRIT = 0b0100,
                        MCP9808_TA = 0b0101,
                        MCP9808_MANUID = 0b0110,
                        MCP9808_DEVID = 0b0111,
                        MCP9808_RESOLUTION = 0b1000};

enum mcp9808_resolution {MCP9808_HALF_DEG = 0b00,
                         MCP9808_QUARTER_DEG = 0b01,
                         MCP9808_EIGHTH_DEG = 0b10,
                         MCP9808_SIXTEENTH_DEG = 0b11};

typedef struct MSP432P4_MCP9808 {
    uint8_t address;
    volatile EUSCI_B_Type *i2cDevice;
} MCP9808;

int initMCP9808 (MCP9808* device, volatile EUSCI_B_Type *i2cPeripheral, uint8_t address);
int setAddress (MCP9808* device, uint8_t address);
void setAddressByBits (MCP9808* device, uint8_t A2, uint8_t A1, uint8_t A0);
uint8_t getAddress(MCP9808* device);
int shutdownWake (MCP9808* device, uint8_t shtdwn);
uint8_t getMCP9808Res (MCP9808* device);
int setMCP9808Res (MCP9808* device, enum mcp9808_resolution res);


int enableDisableAlertPin (MCP9808* device, uint8_t enable);
int setAlertOutputMode (MCP9808* device, uint8_t interruptMode);
int setAlertConditions (MCP9808* device, uint8_t tCritOnly);
int setAlertOutputPolarity (MCP9808* device, uint8_t activeHigh);
int clearAlertInterrupt (MCP9808* device);

float getMCP9808Temp (MCP9808* device);
int setTempHysteresis (MCP9808* device, uint8_t hyst);
int lockUnlockTempSentinels (MCP9808* device, uint8_t lock);
float getMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg);
int setMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg, float temp);

//uint16_t readMCP9808 (MCP9808* device, enum mcp9808_registers reg);
//void writeMCP9808 (MCP9808* device, enum mcp9808_registers reg, uint16_t data);

/* static int read8 (MCP9808* device, enum mcp9808_registers reg);
static int read16 (MCP9808* device, enum mcp9808_registers reg);
static void write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data);
static void write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data); */
#endif /* MCP9808_H_ */
