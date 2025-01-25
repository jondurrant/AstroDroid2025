/***
 * Demo
 * Uses FreeRTOS Task
 * Jon Durrant
 * 9-Jan-2025
 */


#include "pico/stdlib.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "BlinkAgent.h"
#include "AstroDroidConfig.h"

#include <PicoLed.hpp>
#include "LatchedSwitch.h"



//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )



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
 * Main task to blink external LED
 * @param params - unused
 */
void mainTask(void *params){

	printf("Main task started\n");

	BlinkAgent blink(DEBUG_LED2);
	blink.start("Blink", TASK_PRIORITY);

	LatchedSwitch sw(SWITCH_GP);

	// 0. Initialize LED strip
	auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(
			pio0, 0,
			NEOPIXEL,
			NEOPIXEL_NUM,
			PicoLed::FORMAT_GRB);
	ledStrip.setBrightness(64);

	for (;;){
		for (int i=0; i < 7; i++){
			printf("Show pattern %d\n", i);

		    ledStrip.fill( PicoLed::RGB(colours[i][0], colours[i][1], colours[i][2]) );
		    ledStrip.show();

		    if (sw.isLatchOn()){
		    	vTaskDelay(300);
		    } else {
		    	vTaskDelay(1000);
		    }
		}
	}
}




/***
 * Launch the tasks and scheduler
 */
void vLaunch( void) {

	//Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
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

    //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();


    return 0;
}
