/*
 * LX16A.cpp
 *
 *  Created on: 1 Jun 2025
 *      Author: jondurrant
 */

#include "LX16A.h"
#include <cstdio>
#include <math.h>

LX16A::LX16A(){
}

bool LX16A::config(uint8_t gpTx, uint8_t gpRx){
	uart_inst_t *txUart = NULL;
	uart_inst_t *rxUart = NULL;
	xGpTx = gpTx;
	xGpRx = gpRx;
	switch(gpTx){
	case 0:
		txUart = uart0;
		break;
	case 4:
		txUart = uart1;
		break;
	case 8:
		txUart = uart1;
		break;
	case 12:
		txUart = uart0;
		break;
	case 16:
		txUart = uart0;
		break;
	case 20:
		txUart = uart1;
		break;
	}


	switch(gpRx){
		case 1:
			rxUart = uart0;
			break;
		case 5:
			rxUart = uart1;
			break;
		case 9:
			rxUart = uart1;
			break;
		case 13:
			rxUart = uart0;
			break;
		case 17:
			rxUart = uart0;
			break;
		case 21:
			rxUart = uart1;
			break;
		}

	if (txUart == rxUart){
		pUart = txUart;
	} else {
		pUart = NULL;
	}

	if (pUart != NULL){
		uart_init(pUart, 115200);
		return true;
	} else {
		return false;
	}
}

LX16A::~LX16A() {
	// TODO Auto-generated destructor stub
}


uint8_t LX16A::checksum(uint8_t *cmd){
	int16_t checksum = 0;
	uint8_t len = cmd[3] + 2;
	for (int i=2; i <= len; i++){
		checksum += cmd[i];
	}
	int8_t checkByte = checksum & 0xff;
	return ~checkByte;
}


bool LX16A::readTimeout(uint8_t *buf, uint8_t len, uint ms){
	if (pUart == NULL){
		return false;
	}
	uint32_t timeout = to_ms_since_boot (get_absolute_time()) + ms;

	uint32_t now = to_ms_since_boot (get_absolute_time());

	gpio_set_function(xGpRx, GPIO_FUNC_UART);

	uint index = 0;
	uint8_t b;
	while (now < timeout){
		if (uart_is_readable (pUart)){
			uart_read_blocking (
					pUart,
					&b,
					1);
			buf[index] = b;
			if ( (index == 0)){
				if (b == 0x55){
					index++;
					printf("%X ", b);
				} else {
					//skip data
					//printf("skipping data %X\n", b);
				}
			} else {
				index++;
				//printf("%X(%d) ", b, index);
			}

			if (index == len){
				gpio_set_function(xGpRx, GPIO_FUNC_NULL);
				return true;
			}
		}

		now = to_ms_since_boot (get_absolute_time());
	}
	//printf("Timeout\n");
	gpio_set_function(xGpRx, GPIO_FUNC_NULL);
	return false;

}

bool LX16A::writeUart(uint8_t *buf, uint8_t len){
	if (pUart == NULL){
		return false;
	}
	gpio_set_function(xGpTx, GPIO_FUNC_UART);
	uart_write_blocking (
			pUart,
			buf,
			len);

	uart_tx_wait_blocking(pUart);
	gpio_set_function(xGpTx, GPIO_FUNC_NULL);
	return true;
}


void LX16A::moveServoRad(
			uint8_t servo,
			float rad,
			uint ms
			){
	float deg = rad * 180.0/M_PI;
	moveServoDeg(servo, deg, ms);
}

void LX16A::moveServoDeg(
		uint8_t servo,
		float deg,
		uint ms
		){
	uint8_t cmd[10] = {
			0x55, 0x55,
			0x0, 0x0, 0x0, 0x0,0x0,0x0,0x0,
			0x0
	};
	cmd[2] = servo;
	cmd[3] = 7;
	cmd[4] = 1;


	//Angle
	if ((deg < 0.0) || (deg > 240.0)) {
		return;
	}
	uint16_t d = (uint16_t) deg / 0.24;
	cmd[5] = d & 0xff;
	cmd[6] = (d & 0xff00) >> 8;

	//Time
	cmd[7] = ms & 0xff;
	cmd[8] = (ms & 0xff00) >> 8;

	// Checksum
	cmd[9 ] = checksum(cmd);

	writeUart( cmd, 10);
}

bool LX16A::getVin(uint8_t servo, int *mv){
	uint8_t cmd[6] = {
			0x55, 0x55,
			0x0, 0x0, 0x0,
			0x0
	};
	cmd[2] = servo;
	cmd[3] = 3;
	cmd[4] = 27;
	cmd[5] = checksum(cmd);


	writeUart( cmd, 6);

	uint8_t buf[10];
	if (readTimeout(buf, 8, 500)){
		//printf("Read Response\n");
		int m = buf[5] | (buf[6] << 8);
		*mv = m;
		return  true;
	}

	return false;
}

bool LX16A::getVinLimit(uint8_t servo, int *minMv, int *maxMv){
	uint8_t cmd[6] = {
			0x55, 0x55,
			0x0, 0x0, 0x0,
			0x0
	};
	cmd[2] = servo;
	cmd[3] = 3;
	cmd[4] = 23;
	cmd[5] = checksum(cmd);


	writeUart( cmd, 6);

	uint8_t buf[10];

	if (readTimeout(buf, 10, 500)){
		//printf("Read Response\n");
		int lmv =  buf[5] | (buf[6] << 8);
		int hmv = buf[7] | (buf[8] << 8);
		//dumpCmd(buf);

		*minMv =  lmv;
		*maxMv = hmv;
		return true;
	}

	return false;
}

bool LX16A::getPosDeg(uint8_t servo, float *deg){
	uint8_t cmd[6] = {
			0x55, 0x55,
			0x0, 0x0, 0x0,
			0x0
	};
	cmd[2] = servo;
	cmd[3] = 3;
	cmd[4] = 28;
	cmd[5] = checksum(cmd);


	writeUart( cmd, 6);

	uint8_t buf[8];

	if (readTimeout(buf, 8, 500)){
		//dumpCmd(buf);
		int pos =  buf[5] | (buf[6] << 8);
		*deg = (float)pos * 0.24;

		return true;
	}

	return false;
}
bool LX16A::getPosRad(uint8_t servo, float *rad){
	float deg;
	if (getPosDeg(servo, &deg)){
		*rad = M_PI / 180.0 * deg;
		return true;
	}
	return false;
}

void LX16A::dumpCmd(uint8_t * buf){
	uint len = buf[3] + 3;
	printf("Dump: ");
	for (int i=0; i < len; i++){
		printf("0x%X ", buf[i]);
	}
	printf("\n");
}

