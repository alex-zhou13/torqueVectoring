// Source: https://github.com/adafruit/Adafruit_LSM6DS/blob/master/Adafruit_ISM330DHCX.h

/*!
 *  @file Adafruit_ISM330DHCX.h
 *
 * 	I2C Driver for the Adafruit ISM330DHCX 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit ISM330DHCX breakout:
 * 	https://www.adafruit.com/products/4480
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ISM330DHCX_H
#define _ADAFRUIT_ISM330DHCX_H

#include "Adafruit_LSM6DS.h"

#define ISM330DHCX_CHIP_ID 0x6B ///< ISM330DHCX default device id from WHOAMI

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ISM330DHCX I2C Digital Potentiometer
 */
// class Adafruit_ISM330DHCX : public Adafruit_LSM6DSOX {
// public:
//   Adafruit_ISM330DHCX();
//   ~Adafruit_ISM330DHCX(){};

// private:
//   bool _init(int32_t sensor_id);
// };


//REGISTERS

#define GRAVITY 9.80665
#define OUTX_L_G (0X22)
#define OUTX_H_G (0x23)    
#define OUTY_L_G (0x24)
#define OUTY_H_G (0x25)
#define OUTZ_L_G (0x26)
#define OUTZ_H_G (0x27)
#define OUTX_L_A (0x28)
#define OUTX_H_A (0x29)
#define OUTY_L_A (0x2A)
#define OUTY_H_A (0x2B)
#define OUTZ_L_A (0x2C)
#define OUTZ_H_A (0x2D)

#endif
