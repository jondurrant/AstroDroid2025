/***

 */


#include "pico/stdlib.h"
#include <cstdio>

#include "hardware/i2c.h"

#include "Adafruit_INA3221.h"
#include "Adafruit_I2CDevice.h"
#include "Lipo3S.h"

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
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

    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);

    printf("\nI2C Bus Scan\n");
	printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

	for (int addr = 0; addr < (1 << 7); ++addr) {
		if (addr % 16 == 0) {
			printf("%02x ", addr);
		}

		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns
		// -1.

		// Skip over any reserved addresses.
		int ret;
		uint8_t rxdata;
		if (reserved_addr(addr))
			ret = PICO_ERROR_GENERIC;
		else
			ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

		printf(ret < 0 ? "." : "@");
		printf(addr % 16 == 15 ? "\n" : "  ");
	}
	printf("Done.\n");



	Adafruit_I2CDevice dev(0x40, i2c0);
	dev.begin();

	uint8_t w[2] = {0xFE, 0x00};
	uint8_t r[2];
	bool res = dev.write_then_read(
			w,
			1,
			r,
			2
			);
	if (res){
		printf("Read data %X %X\n", r[0], r[1]);
	} else {
		printf("Write then Read failed\n");
	}



	Lipo3S lipo;
	lipo.init();

	for (;;){
		printf("%.2fV %.2fmA [%.2fV, %.2fV, %.2fV\n",
				lipo.totalVoltage(),
				lipo.totalCurrent()/1000.0,
				lipo.cellVoltage(2),
				lipo.cellVoltage(1),
				lipo.cellVoltage(0)
				);

		sleep_ms(1000);
	}




	Adafruit_INA3221 ina;
	ina.begin();
	printf("INA3221 ManufacturerID %X Die ID %X\n",
			ina.getManufacturerID(),
			ina.getDieID());

	ina.enableChannel( 0);
	ina.setShuntResistance(0, 0.1);
	ina.enableChannel( 1);
	ina.setShuntResistance(0, 0.1);
	ina.enableChannel( 2);
	ina.setShuntResistance(0, 0.1);



    for (;;){
    	/*
    	printf("Voltage Shunt %.4f Bus %.4f, Current %.4f\n",
    			ina.getShuntVoltage(0),
				ina.getBusVoltage(0),
				ina.getCurrentAmps(0)
    			);*/

    	printf("Channels %.2fV, %.2fV, %.2fV Total Current %.2fmA\n",
    			ina.getBusVoltage(0),
				ina.getBusVoltage(1),
				ina.getBusVoltage(2),
				ina.getCurrentAmps(0)/1000.0
    			);

    	sleep_ms(1000);
    }



    return 0;
}
