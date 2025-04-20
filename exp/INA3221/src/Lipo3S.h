/*
 * Lipo3S.h
 *
 *  Created on: 20 Apr 2025
 *      Author: jondurrant
 */

#ifndef EXP_INA3221_SRC_LIPO3S_H_
#define EXP_INA3221_SRC_LIPO3S_H_

#include "Adafruit_INA3221.h"
#include "pico/stdlib.h"

class Lipo3S {
public:
	Lipo3S(
			uint8_t i2c_addr = INA3221_DEFAULT_ADDRESS,
             i2c_inst_t *i2c = i2c0);
	virtual ~Lipo3S();

	void init(
			uint8_t i2c_addr = INA3221_DEFAULT_ADDRESS,
             i2c_inst_t *i2c = i2c0);

	float totalVoltage();
	float totalCurrent();
	float cellVoltage(uint8_t cell);

private:
	Adafruit_INA3221 xIna;
};

#endif /* EXP_INA3221_SRC_LIPO3S_H_ */
