/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_IIS2MDC

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_iis2mdc.h"

#define IIS2MDC_MAG_I2C_ADDRESS 0x1E

// IIS2MDC Registers
#define IIS2MDC_ADDR_CFG_REG_A  0x60
#define IIS2MDC_ADDR_CFG_REG_B  0x61
#define IIS2MDC_ADDR_CFG_REG_C  0x62
#define IIS2MDC_ADDR_STATUS_REG 0x67
#define IIS2MDC_ADDR_OUTX_L_REG 0x68
#define IIS2MDC_ADDR_WHO_AM_I   0x4F

// IIS2MDC Definitions
#define IIS2MDC_WHO_AM_I         0b01000000
#define IIS2MDC_STATUS_REG_READY 0b00001111
// CFG_REG_A
#define COMP_TEMP_EN    (1 << 7)
#define MD_CONTINUOUS   (0 << 0)
#define ODR_100         ((1 << 3) | (1 << 2))
// CFG_REG_B
#define OFF_CANC        (1 << 1)
// CFG_REG_C
#define BDU             (1 << 4)

static bool iis2mdcInit(magDev_t *magDev)
{
    bool ack = true;
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    ack = ack && busWriteRegister(dev, IIS2MDC_ADDR_CFG_REG_A, MD_CONTINUOUS | ODR_100 | COMP_TEMP_EN);
    ack = ack && busWriteRegister(dev, IIS2MDC_ADDR_CFG_REG_B, OFF_CANC);
    ack = ack && busWriteRegister(dev, IIS2MDC_ADDR_CFG_REG_C, BDU);

    if (!ack) {
        return false;
    }

    magDev->magOdrHz = 100;
    return true;
}

static bool iis2mdcRead(magDev_t *magDev, int16_t *magData)
{
    extDevice_t *dev = &magDev->dev;

    struct {
        uint8_t xout0;
        uint8_t xout1;
        uint8_t yout0;
        uint8_t yout1;
        uint8_t zout0;
        uint8_t zout1;
        uint8_t tout0;
        uint8_t tout1;
    } buffer;


    uint8_t status = 0;
    if (!busReadRegisterBuffer(dev, IIS2MDC_ADDR_STATUS_REG, &status, 1)) {
        return false;
    }

    if (!(status & IIS2MDC_STATUS_REG_READY)) {
        return false;
    }

    if (!busReadRegisterBuffer(dev, IIS2MDC_ADDR_OUTX_L_REG, (uint8_t *) &buffer, sizeof(buffer))) {
        return false;
    }

    // TODO: is this the correct place to apply the scale?
    const float range_scale = 100.f / 65.535f; // +/- 50,000 milligauss, 16bit

    magData[X] = (((buffer.xout1 << 8) | buffer.xout0)) *  range_scale;
    magData[Y] = (((buffer.yout1 << 8) | buffer.yout0)) *  range_scale;
    magData[Z] = (-1 * ((buffer.zout1 << 8) | buffer.zout0)) *  range_scale;

    return true;
}

bool iis2mdcDetect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = IIS2MDC_MAG_I2C_ADDRESS;
    }

    uint8_t whoami = 0;
    bool ack = busReadRegisterBuffer(dev, IIS2MDC_ADDR_WHO_AM_I, &whoami, 1);

    if (ack && whoami == IIS2MDC_WHO_AM_I) {
        magDev->init = iis2mdcInit;
        magDev->read = iis2mdcRead;
        return true;
    }

    return false;
}
#endif
