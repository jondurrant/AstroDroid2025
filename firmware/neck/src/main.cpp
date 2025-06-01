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

#include "Stepper.h"
#include "JointAgent.h"



//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )


//Blink Delay
#define DELAY				500


/***
 * Main task to blink external LED
 * @param params - unused
 */
void mainTask(void *params){

	printf("Main task started\n");

	//Get Node name from NVS
	NVSJson *nvs = NVSJson::getInstance();
	size_t nameLen = nvs->size(ROS2_NODE);
	char *name =  (char *)pvPortMalloc(nameLen);
	if (nvs->get_str ( ROS2_NODE,  name,  &nameLen) != NVS_OK){
		printf("ERROR Retrieving ROS2_NODE from NVS\n");
		nvs->printNVS();
	}

	//Config
	AstroEntities entities;
	ConfigEntity confEntities;
	entities.addEntity(&confEntities);

	//Stepper
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
	xStep.findZero();
	while (!xStep.isStopped()){
		vTaskDelay(100);
	}

	//Joint Agent
	JointAgent joints;
	joints.addStepper(&xStep);
	entities.addEntity(&joints);

	//Start up a uROS Bridge
	uRosBridge *bridge = uRosBridge::getInstance();
	bridge->setNodeName(name);

	//PubEntities entities;
	bridge->setuRosEntities(&entities);
	//bridge->setLed(DEBUG_LED1);
	bridge->setLed(ZERO_NEOPIXEL);
	bridge->start("Bridge",  TASK_PRIORITY+2);

	joints.start("Joints", TASK_PRIORITY);

	for (;;){
		vTaskDelay(3000);
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
