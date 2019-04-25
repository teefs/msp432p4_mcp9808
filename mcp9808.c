/*
 *      mcp9808.c
 *      Author: N. Chin
 *      Driver for using the MCP9808 I2C temperature sensor with TI's MSP432P4XX microcontroller family.
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

/*
 * Initialize the MCP9808 struct
 * Error code:
 *      -1: Invalid MCP9808 slave address
 *      -2: Slave device other than MCP9808 found @ address
 *      -3: Physical layer error
 */
int initMCP9808 (MCP9808* device, volatile EUSCI_B_Type *i2cPeripheral, uint8_t address){
    device->i2cDevice = i2cPeripheral;
    if (setAddress (device, address) == -1)
        return -1;

    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC;
    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_MODE_3 | EUSCI_A_CTLW0_MST;
    i2cPeripheral->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
    i2cPeripheral->IE = EUSCI_B_IE_RXIE0 | EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;

    uint16_t manuID, devID;

    if (read16(device, MCP9808_MANUID, &manuID) == -1 || read16(device, MCP9808_DEVID, &devID) == -1)
        return -3;

    if (manuID != 0x0054 || devID != 0x0400)
        return -2;

    return 0;
}

/*
 * Sets the address field of the MCP9808 struct
 * Error code:
 *      -1: Address invalid for MCP9808 sensors
 */
int setAddress (MCP9808* device, uint8_t address){
    if (address < 0x18 || address > 0x1F)
        return -1;

    device->address = address;
    return 0;
}

/*
 * Puts the MCP9808 to sleep or wakes it up.
 * Sleep: shtdwn == 1
 * Wake:  shtdwn == 0
 */
int shutdownOrWakeup (MCP9808* device, uint8_t shtdwn){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (shtdwn == 0)
            configReg &= 0xFEFF;
        else
            configReg |= 0x0100;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Sets the resolution of the MCP9808
 */
int setMCP9808Resolution (MCP9808* device, enum mcp9808_resolution res){
    return write8 (device, MCP9808_RESOLUTION, (uint8_t) res);
}

/*
 * Returns the MCP9808 resolution register
 * Error code:
 *      8: I2C read failed
 */
uint8_t getMCP9808Resolution (MCP9808* device){
    uint8_t buffer;

    if (read8 (device, MCP9808_RESOLUTION, &buffer) != 0)
        return 0x08;
    return buffer;
}

/*
 * Enables/ disables the alert output
 * Error code:
 *      -1: I2C R/W failed
 *      -2: Alert settings have been locked and cannot be unlocked unless MCP9808 is power-cycled
 */
int enableDisableAlertPin (MCP9808* device, uint8_t enable){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (configReg & 0x00C0)
            return -2;

        if (enable == 0)
            configReg &= 0xFFF7;
        else
            configReg |= 0x0008;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Puts the alert pin into interrupt or comparator mode
 * Error code:
 *      -1: I2C R/W failed
 *      -2: Alert settings have been locked and cannot be unlocked unless MCP9808 is power-cycled
 */
int setAlertOutputMode (MCP9808* device, uint8_t interruptMode){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (configReg & 0x00C0)
            return -2;

        if (interruptMode == 0)
            configReg &= 0xFFFE;
        else
            configReg |= 0x0001;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Changes the conditions that set the alert pin
 * tCritOnly == 1 disables comparisons with Tlower and Tupper
 * Error code:
 *      -1: I2C R/W failed
 *      -2: Alert settings have been locked and cannot be unlocked unless MCP9808 is power-cycled
 */
int setAlertConditions (MCP9808* device, uint8_t tCritOnly){
    uint16_t configReg;
        if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (configReg & 0x0040)
            return -2;

        if (tCritOnly == 0)
            configReg &= 0xFFFB;
        else
            configReg |= 0x0004;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Sets the alert pin to active-high or active-low
 * Error code:
 *      -1: I2C R/W failed
 *      -2: Alert settings have been locked and cannot be unlocked unless MCP9808 is power-cycled
 */
int setAlertOutputPolarity (MCP9808* device, uint8_t activeHigh){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (configReg & 0x00C0)
            return -2;

        if (activeHigh == 0)
            configReg &= 0xFFFD;
        else
            configReg |= 0x0002;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Clears the interrupt when operating in interrupt mode
 * Error code:
 *      -1: I2C R/W failed
 */
int clearAlertInterrupt (MCP9808* device){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        configReg &= 0xFFDF;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Sets Win. Lock and Crit. Lock bits of the configuration register which can only be reset by power-cycle
 * Prevents changing of Tlower, Tupper, Tcrit, alert polarity, alert conditions, and alert mode
 * Error code:
 *      -1: I2C R/W failed
 */
int lockAlertSettings (MCP9808* device){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        configReg |= 0x00C0;
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Returns the temperature reading in Celsius
 * Error code:
 *      -65535: I2C R/W failed
 */
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

/*
 * Sets hysteresis bands for Tupper and Tlower
 * Error code:
 *      -1: I2C R/W failed
 *      -2: Alert settings have been locked and cannot be unlocked unless MCP9808 is power-cycled
 */
int setTempHysteresis (MCP9808* device, enum mcp9808_hysteresis hyst){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (configReg & 0x00C0)
            return -2;
        configReg &= 0xF9FF;
        configReg |= (hyst << 9);
        return write16(device, MCP9808_CONFIG, configReg);
    }
    return -1;
}

/*
 * Returns Tupper, Tlower, or Tcrit
 * Error code:
 *      -65535: I2C R/W failed
 */
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

/*
 * Sets Tlower, Tupper, Tcrit
 * Error code:
 *      -1: I2C R/W failed
 *      -2: Alert settings have been locked and cannot be unlocked unless MCP9808 is power-cycled
 */
int setMCP9808TempSentinel (MCP9808* device, enum mcp9808_registers reg, float temp){
    uint16_t configReg;
    if (read16(device, MCP9808_CONFIG, &configReg) == 0){
        if (configReg & 0x00C0)
            return -2;

        if ((reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT) && temp <= 255.75 && temp >= -255.75){
            uint16_t data = (uint16_t) (temp * 4);
            data = (data & 0x1FFF) << 2;
            return write16 (device, reg, data);
        }
    }
    return -1;
}

/*
 * I2C reads one (1) byte and places it in buffer
 */
static int read8 (MCP9808* device, enum mcp9808_registers reg, uint8_t* buffer){
    __disable_irq();
        if (device == NULL || buffer == NULL || reg != MCP9808_RESOLUTION)
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
        *buffer = EUSCI_B0->RXBUF;

        while (device->i2cDevice->IFG & EUSCI_B_IFG_STPIFG == 0);
        device->i2cDevice->IFG = 0;
        device->i2cDevice->IE = prevIEReg;
        __enable_irq();
        return 0;
}

/*
 * I2C reads two (2) bytes and places it in buffer
 */
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

    device->i2cDevice->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }

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

/*
 * I2C writes one (1) byte, data, to the MCP9808
 */
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

/*
 * I2C writes two (2) bytes, data, to the MCP9808
 */
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
