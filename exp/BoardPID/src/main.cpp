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
#include "MotorMgr.h"
#include "MotorPID.h"

#include <math.h>

#include "AstroDroidConfig.h"


//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

//LED PAD to use
#define LED_PAD	PICO_DEFAULT_LED_PIN

//Blink Delay
#define DELAY				500

//Motor Pins
#define D37CW 		MOTOR_A
#define D37CCW 	MOTOR_B
#define D37A 		    ENCODER_A
#define D37B		    ENCODER_B


//D37 131.25:1   Max Speed is 7 Rad PS.
#define KP	0.3
#define KI	0.001
#define KD	 0.01


/***
 * Main task to blink external LED
 * @param params - unused
 */
void mainTask(void *params){

	printf("Main task started\n");

	BlinkAgent blink(LED_PAD);
	blink.start("Blink", TASK_PRIORITY);

	MotorPID d37(D37CW, D37CCW, D37A, D37B);
	d37.printConfig();

	d37.configPID( KP, KI, KD);


	d37.setSpeedRadPS(1.0, true);
	for (;;){
		for (int s=1; s < 10; s++){
			float speed = (float)s / 1.0;
			d37.setSpeedRadPS(speed, true);
			printf("%f Rad Per Sec %f avg %f \n",  speed, d37.getRadPerSec(), d37.getAvgRadPerSec());
			for (int i = 0; i < 100; i++){
				d37.doPID();
				vTaskDelay(100);
			}
			printf("%f Rad Per Sec %f avg %f \n",  speed, d37.getRadPerSec(), d37.getAvgRadPerSec());
		}
	}


	for (;;){
		//d37.turnDeltaPos(16 * 131 + 4, 0.3);
		d37.turnDeltaRadians(M_PI, 0.5);
		for (int i=0; i < 1000; i++){
			printf("Rad Per Sec %f avg %f \n", d37.getRadPerSec(), d37.getAvgRadPerSec());
		}
		vTaskDelay(8000);
		printf("POS %d\n",d37. getDeltaPos() );
	}


	/*
	printf("POS %d\n", d37.getDeltaPos());
	d37.setThrottle( 0.3, true);
	for (;;){
		vTaskDelay(500);
		printf("POS %d\n", d37.getDeltaPos());
	}
	*/


	for(;;){

		for (float f = 0.1; f <= 1.0; f=f+0.1){

			printf("Throttle set: %f\n", f);
			d37.setThrottle( f, true);
			vTaskDelay(3000);
			printf("RPS %f\n", d37.getAvgRadPerSec());
			vTaskDelay(3000);

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
