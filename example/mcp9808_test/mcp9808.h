/*
 *      mcp9808.c
 *      Author: N. Chin
 *      Driver for using the MCP9808 I2C temperature sensor with TI's MSP432P4XX microcontroller family.
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

enum mcp9808_hysteresis {MCP9808_HYST_0DEG = 0b00,
                         MCP9808_HYST_15DEG = 0b01,
                         MCP9808_HYST_3DEG = 0b10,
                         MCP9808_HYST_6DEG = 0b11};

typedef struct MSP432P4_MCP9808 {
    uint8_t address;
    volatile EUSCI_B_Type *i2cDevice;
} MCP9808;

int initMCP9808 (MCP9808* device, volatile EUSCI_B_Type *i2cPeripheral, uint8_t address);
int setAddress (MCP9808* device, uint8_t address);
int shutdownWake (MCP9808* device, uint8_t shtdwn);
int setMCP9808Resolution (MCP9808* device, enum mcp9808_resolution res);
uint8_t getMCP9808Resolution (MCP9808* device);

int enableDisableAlertPin (MCP9808* device, uint8_t enable);
int setAlertOutputMode (MCP9808* device, uint8_t interruptMode);
int setAlertConditions (MCP9808* device, uint8_t tCritOnly);
int setAlertOutputPolarity (MCP9808* device, uint8_t activeHigh);
int clearAlertInterrupt (MCP9808* device);
int lockAlertSettings (MCP9808* device);

float getMCP9808Temp (MCP9808* device);
int setTempHysteresis (MCP9808* device, uint8_t hyst);
float getMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg);
int setMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg, float temp);

#endif /* MCP9808_H_ */
