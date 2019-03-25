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

#define

enum mcp9808_registers {MCP9808_CONFIG = 0001,
                        MCP9808_TUPPER = 0010,
                        MCP9808_TLOWER = 0011,
                        MCP9808_TCRIT = 0100,
                        MCP9808_TA = 0101,
                        MCP9808_MANUID = 0110,
                        MCP9808_DEVID = 0111,
                        MCP9808_RESOLUTION = 1000};

enum mcp9808_resolution {MCP9808_HALF_DEG = 00,
                         MCP9808_QUARTER_DEG = 01,
                         MCP9808_EIGHTH_DEG = 10,
                         MCP9808_SIXTEENTH_DEG = 11};

typedef struct MSP432P4_MCP9808 {
    uint8_t address;
    EUSCI_B_Type *i2cDevice;

    uint16_t readBuf;
    uint16_t config;
} MCP9808;

int initMCP9808 (MCP9808* device, EUSCI_B_Type i2cPeripheral, uint8_t addressBits);
int setAddress (MCP9808* device, uint8_t addressBits);
uint8_t getAddress(MCP9808* device);

void shutdownOrWakeup (MCP9808* device, uint8_t shtdwnOrWk);
void shutdown (MCP9808* device);
void wakeup (MCP9808* device);

float getMCP9898Temp (MCP9808* device);
uint8_t getMCP9808Res (MCP9808* device);
void setMCP9808Res (MCP9808* device, enum mcp9808_resolution res);
uint16_t getMCP9808Config (MCP9808* device);
void setMCP9808Config (MCP9808* device, uint16_t config);
uint16_t getMCP9808TempSent (MCP9808* device, enum mcp9808_registers reg);
void setMCP9808TempSent (MCP9808* device, enum mcp9808_registers reg, uint16_t temp);

//uint16_t readMCP9808 (MCP9808* device, enum mcp9808_registers reg);
//void writeMCP9808 (MCP9808* device, enum mcp9808_registers reg, uint16_t data);

static int read8 (MCP9808* device, enum mcp9808_registers reg);
static int read16 (MCP9808* device, enum mcp9808_registers reg);
static void write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data);
static void write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data);
#endif /* MCP9808_H_ */
