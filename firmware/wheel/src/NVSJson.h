/*
 * NVSJson.h
 *
 *  Created on: 29 Jan 2025
 *      Author: jondurrant
 */

#ifndef FIRMWARE_WHEEL_SRC_NVSJSON_H_
#define FIRMWARE_WHEEL_SRC_NVSJSON_H_

#include "pico/stdlib.h"
#include "NVSOnboard.h"

#define NVS_STR_LEN 80

class NVSJson : public NVSOnboard {
public:
	/***
	 * Singleton to return the current instance
	 * @param cleanNVS - Only usable on first call.
	 * If true returns a clean NVS without reading the Flash
	 * Used mainly for rest purposes.
	 * Default is false
	 * @return the NVSOnboard instance
	 */
	static NVSJson * getInstance(bool cleanNVS=false);

	virtual ~NVSJson();

	void toJSON(char *dest, size_t len);

protected:
	NVSJson(bool cleanNVS=false);

	virtual char * toJSONKey(char *dest,  size_t* remLen , const char * key,  nvs_type_t type);

private:
	static NVSJson * pSingleton;
};

#endif /* FIRMWARE_WHEEL_SRC_NVSJSON_H_ */
