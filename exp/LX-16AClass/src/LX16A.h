/*
 * LX16A.h
 *
 *  Created on: 1 Jun 2025
 *      Author: jondurrant
 */

#ifndef EXP_LX_16ACLASS_SRC_LX16A_H_
#define EXP_LX_16ACLASS_SRC_LX16A_H_

#include "pico/stdlib.h"

#include "hardware/uart.h"

class LX16A {
public:
	LX16A();
	virtual ~LX16A();

	bool config(uint8_t gpTx, uint8_t gpRx);

	void moveServoDeg(
			uint8_t servo,
			float deg,
			uint ms
			);

	void moveServoRad(
			uint8_t servo,
			float rad,
			uint ms
			);

	bool getVin(uint8_t servo, int *mv);

	bool getVinLimit(uint8_t servo, int *minMv,  int *maxMv);


	bool getPosDeg(uint8_t servo, float *deg);
	bool getPosRad(uint8_t servo, float *rad);
protected:
	virtual bool writeUart(uint8_t *buf, uint8_t len);
	virtual bool readTimeout(uint8_t *buf, uint8_t len, uint ms);
	virtual uint8_t checksum(uint8_t *cmd);

private:
	void dumpCmd(uint8_t * buf);
	uint8_t xGpTx;
	uint8_t xGpRx;
	uart_inst_t *pUart;

};

#endif /* EXP_LX_16ACLASS_SRC_LX16A_H_ */
