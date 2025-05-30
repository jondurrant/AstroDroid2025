/*
 * main.cpp
 *
 *  Created on: 10-Mar-2024
 *      Author: jondurrant
 */


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "hardware/clocks.h"
#include <math.h>
#include "Stepper.h"


//Breadboard
/*
#define GP_STEP_DETECT 	12
#define GP_STEP 						13
#define GP_DIR    						14
#define GP_M0							10
#define GP_M1							11
#define GP_M2							12
*/

// Driver PCB
#define GP_STEP_DETECT 	3
#define GP_STEP 						2
#define GP_DIR    						4
#define GP_SLEEP    					6
#define GP_M0							26
#define GP_M1							27
#define GP_M2							28
#define GP_HALL						12

int main(){

	stdio_init_all();
	sleep_ms(2000);
	printf("Go\n");


	/*
	gpio_init(GP_M0);
	gpio_init(GP_M1);
	gpio_init(GP_M2);
	gpio_set_dir(GP_M0, GPIO_OUT);
	gpio_set_dir(GP_M1, GPIO_OUT);
	gpio_set_dir(GP_M2, GPIO_OUT);
	gpio_put(GP_M0, 0);
	gpio_put(GP_M1, 0);
	gpio_put(GP_M2, 0);
	*/

	/*
	gpio_init(GP_SLEEP);
	gpio_set_dir(GP_SLEEP, GPIO_OUT);
	gpio_put(GP_SLEEP, 1);
	*/


	Stepper xStep(
			GP_STEP,
			GP_DIR,
			GP_STEP_DETECT,
			GP_HALL
			);
	xStep.configMicroStepGp(GP_M0, GP_M1, GP_M2);
	xStep.configSleepGp(GP_SLEEP);
	uint maxSteps = 381904/32;
	xStep.setMaxSteps(maxSteps);

	/*
	printf("Calibrate....\n");
	uint maxSteps = xStep.calibrate();
	xStep.setMaxSteps(maxSteps);
	printf("Calibrate Done\n");
	sleep_ms(10000);
	*/

	printf("Move to Zero\n");
	xStep.findZero(true, 0.3);
	while(!xStep.isStopped()){
		sleep_ms(200);
	}
	sleep_ms(2000);




	bool dir = false;


	/*
	printf("Continuous CW\n");
	xStep.continuous(dir,   0.1);
	for (int i=0; i < 6; i++){
		sleep_ms(1000);
		printf("Stepper at %u  %ff\n",
						xStep.getPos(),
						xStep.getPosRad()
						);
	}
	printf("Continuous CCW\n");
	xStep.continuous(!dir,   0.1);
		for (int i=0; i < 5; i++){
			sleep_ms(1000);
			printf("Stepper at %u  %ff\n",
							xStep.getPos(),
							xStep.getPosRad()
							);
		}


		printf("Move to Zero\n");
		xStep.findZero(true, 0.3);
		while(!xStep.isStopped()){
			sleep_ms(200);
		}
		sleep_ms(2000);

*/

	if (maxSteps != xStep.getMaxSteps()){
		printf("ERROR MAX STEPS is wrong %u != %u\n",
				maxSteps,
				 xStep.getMaxSteps()
				);
	}



	uint steps = xStep.getMaxSteps() / 4;
	/*
	printf("moveBySteps(%u)...\n", steps);
	for (int i=1; i <=4; i++){
		printf("Move 1/4 in 3s\n");
		xStep.moveBySteps(steps,  i%2,  3000);
		for (int s=0; s< 10;s++){
				sleep_ms(1000);
				printf("Stepper at %u  %f\n",
						xStep.getPos(),
						xStep.getPosRad()
						);
			}
	}
	*/

/*

	printf("Move to step\n");
	for (int i=1; i <=4; i++){
		printf("Move 1/4 in 3s\n");
		xStep.moveToStep((steps *i),  true,  5000);
		for (int s=0; s< 10;s++){
				sleep_ms(1000);
				printf("Stepper at %u  %f\n",
						xStep.getPos(),
						xStep.getPosRad()
						);
			}
	}
	printf("Move to step CCW\n");
	for (int i=3; i >0; i--){
			printf("Move 1/4 in 3s\n");
			xStep.moveToStep((steps *i),  false,  5000);
			for (int s=0; s< 10;s++){
					sleep_ms(1000);
					printf("Stepper at %u  %f\n",
							xStep.getPos(),
							xStep.getPosRad()
							);
				}
		}
*/




	dir = true;
	printf("Move to pos\n");
	for (int i=1; i < 12; i++){
		float target = (M_PI/6.0) * (float) i;
		printf("Move to %f in 5000ms\n", target);
		xStep.moveToRad(
				target,
				dir,
				5000);
		for (int s=0; s< 10;s++){
			sleep_ms(1000);
			printf("Stepper at %u  %f = %f\n",
					xStep.getPos(),
					xStep.getPosRad(),
					target);
		}
	}






	for (;;){
		sleep_ms(3000);
	}

}


