/***
 * Demo
 * Jon Durrant
 * 9-Feb-2025
 */


#include "pico/stdlib.h"

#include <stdio.h>

#include "AstroDroidConfig.h"

#include <PicoLed.hpp>




uint8_t colours[7][3]{
		{0xff,0x00,0x00},
		{0xff,0xa5,0x00},
		{0xff,0xff,0x00},
		{0x00,0x80,0x00},
		{0x00,0x00,0xff},
		{0x4b,0x00,0x82},
		{0xee,0x82,0xee}
};



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


    // 0. Initialize LED strip
	auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(
			pio0, 0,
			16,
			1,
			PicoLed::FORMAT_GRB);
	ledStrip.setBrightness(64);

	for (;;){
		for (int i=0; i < 7; i++){
			printf("Show pattern %d\n", i);

			ledStrip.fill( PicoLed::RGB(colours[i][0], colours[i][1], colours[i][2]) );
			ledStrip.show();

			sleep_ms(1000);

		}
	}


    return 0;
}
