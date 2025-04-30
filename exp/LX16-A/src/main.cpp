/***
 * Demo
 * Jon Durrant
 * 9-Feb-2025
 */


#include "pico/stdlib.h"

#include <stdio.h>


#include "hardware/uart.h"

#define GP_TX 20
#define GP_RX 21


uint8_t checksum(uint8_t *cmd){
	int16_t checksum = 0;
	uint8_t len = cmd[3] + 2;
	for (int i=2; i <= len; i++){
		checksum += cmd[i];
	}
	int8_t checkByte = checksum & 0xff;
	return ~checkByte;
}


void dumpCmd(uint8_t * buf){
	uint len = buf[3] + 3;
	printf("Dump: ");
	for (int i=0; i < len; i++){
		printf("0x%X ", buf[i]);
	}
	printf("\n");
}

bool readTimeout(uint8_t *buf, uint8_t len, uint ms){
	uint32_t timeout = to_ms_since_boot (get_absolute_time()) + ms;

	uint32_t now = to_ms_since_boot (get_absolute_time());

	gpio_set_function(GP_RX, GPIO_FUNC_UART);

	uint index = 0;
	uint8_t b;
	while (now < timeout){
		if (uart_is_readable (uart1)){
			uart_read_blocking (
					uart1,
					&b,
					1);
			buf[index] = b;
			if ( (index == 0)){
				if (b == 0x55){
					index++;
					printf("%X ", b);
				} else {
					//skip data
					printf("skipping data %X\n", b);
				}
			} else {
				index++;
				printf("%X(%d) ", b, index);
			}

			if (index == len){
				gpio_set_function(GP_RX, GPIO_FUNC_NULL);
				return true;
			}
		}

		now = to_ms_since_boot (get_absolute_time());
	}
	printf("Timeout\n");
	gpio_set_function(GP_RX, GPIO_FUNC_NULL);
	return false;

}

bool writeUart(uint8_t *buf, uint8_t len){
	gpio_set_function(GP_TX, GPIO_FUNC_UART);
	uart_write_blocking (
			uart1,
			buf,
			len);

	uart_tx_wait_blocking(uart1);
	gpio_set_function(GP_TX, GPIO_FUNC_NULL);
	return true;
}

void moveServo(
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

	printf("Move Cmd: ");
	for (int i=0; i < 10; i++){
		printf("0x%X ", cmd[i]);
	}
	printf("\n");


	writeUart( cmd, 10);
}


int getVin(uint8_t servo){
	uint8_t cmd[10] = {
			0x55, 0x55,
			0x0, 0x0, 0x0, 0x0,0x0,0x0,0x0,
			0x0
	};
	cmd[2] = servo;
	cmd[3] = 3;
	cmd[4] = 27;
	cmd[5] = checksum(cmd);


	writeUart( cmd, 6);

	uint8_t buf[10];
	if (readTimeout(buf, 8, 500)){
		printf("Read Response\n");
		int mv = buf[5] | (buf[6] << 8);
		return  mv;
	} else {
		printf("Failed to read Response\n");
	}



	return 0;
}

bool getVinLimit(uint8_t servo, uint *minMv, uint *maxMv){
	uint8_t cmd[10] = {
			0x55, 0x55,
			0x0, 0x0, 0x0, 0x0,0x0,0x0,0x0,
			0x0
	};
	cmd[2] = servo;
	cmd[3] = 3;
	cmd[4] = 23;
	cmd[5] = checksum(cmd);


	writeUart( cmd, 6);



	uint8_t buf[10];

	if (readTimeout(buf, 10, 500)){
		printf("Read Response\n");
		int lmv =  buf[5] | (buf[6] << 8);
		int hmv = buf[7] | (buf[8] << 8);
		dumpCmd(buf);

		*minMv =  lmv;
		*maxMv = hmv;
		return true;
	} else {
		printf("Failed to read Response\n");
	}



	return false;
}

/***
 * Main
 * @return
 */
int main( void )
{
	//Setup serial over USB and give a few seconds to settle before we start
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");

    uart_init(uart1, 115200);
	//uart_set_format( uart1,  8,  1, UART_PARITY_NONE);
	//gpio_set_function(GP_TX, GPIO_FUNC_UART);
	//gpio_set_function(GP_RX, GPIO_FUNC_UART);


	int mv;
	uint lmv, hmv;
	for (;;){
		moveServo(1, 50, 500);
		moveServo(2, 200, 800);

	    mv = getVin(1);
		printf("1: %dmv\n", mv);
		mv = getVin(2);
		printf("2: %dmv\n", mv);
		if(getVinLimit(1, &lmv, &hmv)){
			printf("Limit %umv to %umv\n", lmv, hmv);
		}
		sleep_ms(2000);

		moveServo(1, 200, 500);
		moveServo(2, 50, 800);

		mv = getVin(1);
		printf("%dmv\n", mv);
		sleep_ms(2000);

	}

    return 0;
}
