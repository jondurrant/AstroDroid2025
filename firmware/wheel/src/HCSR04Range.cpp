/*
 * HCSR04Range.cpp
 *
 *  Created on: 5 Sept 2023
 *      Author: jondurrant
 */

#include "HCSR04Range.h"
#include "hardware/gpio.h"
#include "GPIOInputMgr.h"
#include <cstdio>
#include <cinttypes>




HCSR04Range::HCSR04Range(uint8_t GPTrigger, uint8_t GPEcho){
	xGPTrigger = GPTrigger;
	xGPEcho = GPEcho;

	gpio_init(GPTrigger);
	gpio_set_dir(GPTrigger, GPIO_OUT);

	GPIOInputMgr::getMgr()->addObserver(xGPEcho,  this);

}

HCSR04Range::~HCSR04Range() {
	// TODO Auto-generated destructor stub
}


void HCSR04Range::trigger(){
	xSensing = true;
	gpio_put (xGPTrigger,true);

	xAlarm = add_alarm_in_us	(10,
		HCSR04Range::alarmCB,
		this,
		true
	);

}

int HCSR04Range::getDistanceMM(){
	if (xEchoEnd <= xEchoStart){
		printf("End is before start %" PRId64 " %" PRId64 "\n",
				xEchoStart,
				xEchoEnd
				);
		return -1;
	}
	uint64_t us = xEchoEnd - xEchoStart;

	float mm = (0.340 * (float)us) / 2.0;
	return (int)mm;
}

float HCSR04Range::getDistanceMeters(){
	int mm = getDistanceMM();
	if ( mm < 0){
		return -1.0;
	}
	return ((float)mm) / 1000.0;
}


/***
 * handle GPIO  events
 * @param gpio - GPIO number
 * @param events - Event
 */
 void HCSR04Range::handleGPIO(uint gpio, uint32_t events){
	 if (!xSensing){
		 printf("Echo but not sensing %d\n", gpio);
		 return;
	 }
	 uint64_t now = to_us_since_boot (get_absolute_time());
	 if (gpio == xGPEcho){
		 if ((events & GPIO_IRQ_EDGE_RISE)  > 0) {
			 xEchoStart = now;
		 }
		 if ((events &  GPIO_IRQ_EDGE_FALL) > 0){
			 xEchoEnd = now;
			 xSensing = false;
		 }
	 }
 }


 int64_t HCSR04Range::alarmCB(alarm_id_t id, void *user_data){

	 HCSR04Range *pRange = (HCSR04Range *) user_data;
	 pRange->triggerEnd(id);

	 return 0;
 }

 void HCSR04Range::triggerEnd(alarm_id_t id){
	 if (id == xAlarm){
		 gpio_put (xGPTrigger, false);
	 }
 }
