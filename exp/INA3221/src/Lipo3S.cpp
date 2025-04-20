/*
 * Lipo3S.cpp
 *
 *  Created on: 20 Apr 2025
 *      Author: jondurrant
 */

#include "Lipo3S.h"


Lipo3S::Lipo3S(
		uint8_t i2cAddr,
        i2c_inst_t *i2c
		) {
	init(i2cAddr, i2c);
}

Lipo3S::~Lipo3S() {
	//Nop
}


void Lipo3S::init(
			uint8_t i2cAddr,
             i2c_inst_t *i2c
			 ){
	xIna.begin(i2cAddr, i2c);
}



float Lipo3S::totalVoltage(){
	return xIna.getBusVoltage(0);
}

float Lipo3S::totalCurrent(){
	return 	xIna.getCurrentAmps(0);
}


float Lipo3S::cellVoltage(uint8_t cell){
	if (cell > 2){
		return NAN;
	}

	float prevVolt = 0.0;
	if (cell < 2){
		prevVolt =  xIna.getBusVoltage(cell + 1);
	}

	return xIna.getBusVoltage(cell) - prevVolt;
}
