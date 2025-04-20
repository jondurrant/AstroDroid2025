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

extern"C"{
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"
}

#include "uRosEntities.h"
#include "PubEntities.h"
#include "uRosBridge.h"

#include "ConfigEntity.h"
#include "AstroEntities.h"
#include "NVSJson.h"
#include "MotorsAgent.h"

#include "HCSR04Agent.h"
#include "LightsAgent.h"


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

	//Get Node name from NVS
	NVSJson *nvs = NVSJson::getInstance();
	size_t nameLen = nvs->size(ROS2_NODE);
	char *name =  (char *)pvPortMalloc(nameLen);
	if (nvs->get_str ( ROS2_NODE,  name,  &nameLen) != NVS_OK){
		printf("ERROR Retrieving ROS2_NODE from NVS\n");
		nvs->printNVS();
	}

	//Config
	ConfigEntity confEntities;
	MotorsAgent motEntities;
	motEntities.addMotor(
			0,
			MOTOR_A,
			MOTOR_B,
			ENCODER_A,
			ENCODER_B
			);
	double kp = 0.3;
	double ki = 0.001;
	double kd = 0.01;
	nvs->get_double( CONFIG_PID_KP, &kp);
	nvs->get_double( CONFIG_PID_KI, &ki);
	nvs->get_double( CONFIG_PID_KD, &kd);
	printf("PID Config (%.3f,  %.3f, %.3f)\n", kp, ki, kd);
	motEntities.configAllPID(kp, ki, kd);
	motEntities.start("Motors", TASK_PRIORITY);
	AstroEntities entities;
	entities.addEntity(&confEntities);
	entities.addEntity(&motEntities);


	//HCSR04
	bool left;
	nvs->get_bool(CONFIG_LEFT,  &left);
	char side[6];
	char sensorName[12];
	if (left){
		strcpy(side, "LEFT");
	} else {
		strcpy(side, "RIGHT");
	}
	HCSR04Agent hcsr04Sensors;
	sprintf(sensorName,"%s-BACK", side);
	hcsr04Sensors.addSensor(TRIGGER1, ECHO1,  sensorName);
	sprintf(sensorName,"%s-45", side);
	hcsr04Sensors.addSensor(TRIGGER2, ECHO2,  sensorName);
	sprintf(sensorName,"%s-SIDE", side);
	hcsr04Sensors.addSensor(TRIGGER3, ECHO3,  sensorName);
	hcsr04Sensors.start("HCSR04s", TASK_PRIORITY);
	entities.addEntity(&hcsr04Sensors);


	//Lights
	LightsAgent lights;
	entities.addEntity(&lights);



	//Start up a uROS Bridge
	uRosBridge *bridge = uRosBridge::getInstance();
	bridge->setNodeName(name);

	//PubEntities entities;
	bridge->setuRosEntities(&entities);
	//bridge->setLed(DEBUG_LED1);
	bridge->setLed(ZERO_NEOPIXEL);
	bridge->start("Bridge",  TASK_PRIORITY+2);

	for (;;){
		vTaskDelay(3000);
	}



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





}




/***
 * Launch the tasks and scheduler
 */
void vLaunch( void) {

	//Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 1024*4, NULL, TASK_PRIORITY, &task);

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
    stdio_filter_driver(&stdio_uart);
    sleep_ms(2000);
    printf("GO\n");

    //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();


    return 0;
}
