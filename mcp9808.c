/*
 * mcp9808.c
 *
 *  Created on: Mar 6, 2019
 *      Author: Nicholas
 */
#include "mcp9808.h"
#include <stdint.h>
#include <stdlib.h>


/* Declaration for local functions and resources that do not need to be exposed to the user. */
#define NACKIV 0x4

static int read8 (MCP9808* device, enum mcp9808_registers reg, uint8_t* buffer);
static int read16 (MCP9808* device, enum mcp9808_registers reg, uint16_t* buffer);
static int write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data);
static int write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data);
static uint16_t getConfig (MCP9808* device);
static int setMCP9808Config (MCP9808* device, uint16_t config);


int initMCP9808 (MCP9808* device, volatile EUSCI_B_Type *i2cPeripheral, uint8_t addressBits){
    device->i2cDevice = i2cPeripheral;
    if (setAddress (device, addressBits) == -1)
        return -1;

    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC;
    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_MODE_3 | EUSCI_A_CTLW0_MST;
    i2cPeripheral->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
    i2cPeripheral->IE = EUSCI_B_IE_RXIE0 | EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;

    uint16_t manuID, devID;

    if (read16(device, MCP9808_MANUID, &manuID) == -1 || read16(device, MCP9808_DEVID, &devID) == -1)
        return -3;

    if (manuID != 0x0054 || devID != 0x0400)
        return -1;

    return 0;
}

int setAddress (MCP9808* device, uint8_t address){
    if (address < 0x18 || address > 0x1F)
        return -1;

    device->address = address;
    return 0;
}

uint8_t getAddress(MCP9808* device){
    return device->address;
}

void shutdownOrWakeup (MCP9808* device, uint8_t shtdwn){
//    device->config = (device->config & 0xEF) | shtdwn;
//    write16 (device, MCP9808_CONFIG, device->config);
}

float getMCP9808Temp (MCP9808* device){
    uint16_t ta;
    if (read16 (device, MCP9808_TA, &ta) == -1)
        return -65535;
    float temp = (float) (ta & 0xFFF);
    temp /= 16.0;
    if (ta & 0x1000)
        temp -= 256;
    return temp;
}
uint8_t getMCP9808Resolution (MCP9808* device){
    uint8_t buffer;

    if (read8 (device, MCP9808_RESOLUTION, &buffer) != 0)
        return 0x100;
    return buffer;
}

int setMCP9808Resolution (MCP9808* device, enum mcp9808_resolution res){
    return write8 (device, MCP9808_RESOLUTION, (uint8_t) res);
}

float getMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg){
    if (reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT){
        uint16_t data;
        if (read16 (device, reg, &data) == 0){
            data = data >> 2;
            float temp = (float) (data & 0x03FF);
            temp /= 4;
            if (data & 0x400)
                temp -= 256;

            return temp;
        }
    }

    return -65535;
}

int setMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg, float temp){
    if ((reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT) && temp <= 255.75 && temp >= -255.75){
        uint16_t data = (uint16_t) (temp * 4);
        data = (data & 0x1FFF) << 2;
        return write16 (device, reg, data);
    }
    return -1;
}

static int read8 (MCP9808* device, enum mcp9808_registers reg, uint8_t* buffer){
    __disable_irq();
        if (device == NULL || buffer == NULL || reg != MCP9808_RESOLUTION)
            return -1;

        uint16_t prevIEReg = device->i2cDevice->IE;
        uint16_t prevTBCNT = device->i2cDevice->TBCNT;
        uint16_t prevCTLW1 = device->i2cDevice->CTLW1;

        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
        device->i2cDevice->IE = EUSCI_B_IE_RXIE0 | EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;
        device->i2cDevice->I2CSA = (uint16_t) device->address;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        while (device->i2cDevice->IV == 0);

        device->i2cDevice->TXBUF = reg;

        while (device->i2cDevice->IV == 0);
        if (device->i2cDevice->IV & NACKIV){
            device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            device->i2cDevice->IFG = 0;
            return -1;
        }


        //while (device->i2cDevice->IFG & EUSCI_B_IFG_STTIFG == 0);
        // HAVE TO USE AUTOMATIC STOP GENERATION.
//        device->i2cDevice->CTLW0 |= EUSCI_A_CTLW0_SWRST;
//        device->i2cDevice->TBCNT = 0x1;
//        device->i2cDevice->CTLW1 |= EUSCI_B_CTLW1_ASTP_2;
//        device->i2cDevice->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
//        device->i2cDevice->IE = EUSCI_B_IE_RXIE0 | EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;

        device->i2cDevice->CTLW0 &= ~EUSCI_B_CTLW0_TR;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        while (device->i2cDevice->CTLW0 & EUSCI_B_CTLW0_TXSTT);
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

        while (device->i2cDevice->IV == 0);
        if (device->i2cDevice->IV & NACKIV){
            device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            device->i2cDevice->IFG = 0;
            return -1;
        }
        //device->i2cDevice->IFG = 0;
        *buffer = EUSCI_B0->RXBUF;

        while (device->i2cDevice->IFG & EUSCI_B_IFG_STPIFG == 0);
        device->i2cDevice->IFG = 0;
        //device->i2cDevice->CTLW0 |= EUSCI_A_CTLW0_SWRST;
        //device->i2cDevice->CTLW1 = prevCTLW1;
        //device->i2cDevice->TBCNT = prevTBCNT;
        //device->i2cDevice->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
        //device->i2cDevice->IE = prevIEReg;

        __enable_irq();
        return 0;
}

static int read16 (MCP9808* device, enum mcp9808_registers reg, uint16_t* buffer){
    __disable_irq();
    if (device == NULL || buffer == NULL || reg == MCP9808_RESOLUTION)
        return -1;

    uint16_t prevIEReg = device->i2cDevice->IE;

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
    device->i2cDevice->IE = EUSCI_B_IE_RXIE0 | EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IV == 0);

    device->i2cDevice->TXBUF = reg;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }
    //device->i2cDevice->IFG = 0;


    device->i2cDevice->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }
    //device->i2cDevice->IFG = 0;

    *buffer = EUSCI_B0->RXBUF << 8;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }

    *buffer |= EUSCI_B0->RXBUF;

    while (device->i2cDevice->IFG & EUSCI_B_IFG_STPIFG == 0);
    device->i2cDevice->IFG = 0;
    device->i2cDevice->IE = prevIEReg;

    __enable_irq();
    return 0;
}

static int write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data){
    __disable_irq();
        if (device == NULL || reg != MCP9808_RESOLUTION)
            return -1;

        uint16_t prevIEReg = device->i2cDevice->IE;

        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
        device->i2cDevice->IE = EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;
        device->i2cDevice->I2CSA = (uint16_t) device->address;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        while (device->i2cDevice->IV == 0);

        device->i2cDevice->TXBUF = reg;

        while (device->i2cDevice->IV == 0);
        if (device->i2cDevice->IV & NACKIV){
            device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            device->i2cDevice->IFG = 0;
            return -1;
        }

        device->i2cDevice->TXBUF = data & 0xFF;

        while (device->i2cDevice->IV == 0);
        if (device->i2cDevice->IV & NACKIV){
            device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            device->i2cDevice->IFG = 0;
            return -1;
        }

        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

        while (device->i2cDevice->IFG & EUSCI_B_IFG_STPIFG == 0);
        device->i2cDevice->IFG = 0;
        device->i2cDevice->IE = prevIEReg;

        __enable_irq();
        return 0;
}

static int write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data){
    __disable_irq();
    if (device == NULL || reg == MCP9808_RESOLUTION)
        return -1;

    uint16_t prevIEReg = device->i2cDevice->IE;

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
    device->i2cDevice->IE = EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IV == 0);

    device->i2cDevice->TXBUF = reg;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }

    device->i2cDevice->TXBUF = data >> 8;


    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }


    device->i2cDevice->TXBUF = data & 0xFF;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    while (device->i2cDevice->IFG & EUSCI_B_IFG_STPIFG == 0);
    device->i2cDevice->IFG = 0;
    device->i2cDevice->IE = prevIEReg;

    __enable_irq();
    return 0;
}
