/*
 * NVSJson.cpp
 *
 *  Created on: 29 Jan 2025
 *      Author: jondurrant
 */

#include "NVSJson.h"
#include "json-maker/json-maker.h"

NVSJson * NVSJson::pSingleton = NULL;

NVSJson::NVSJson(bool cleanNVS): NVSOnboard(cleanNVS) {
	//NOP
}

NVSJson::~NVSJson() {
	// TODO Auto-generated destructor stub
}

NVSJson * NVSJson:: getInstance(bool cleanNVS){
	if (NVSJson::pSingleton == NULL) {
		NVSJson::pSingleton = new NVSJson(cleanNVS);
	}
	return NVSJson::pSingleton;
}

void NVSJson::toJSON(char *dest, size_t len){

	char *res = dest;
	size_t remLen = len;

	res = json_objOpen(
			res,
			NULL,
			&remLen
			);
	map<string, nvs_entry_t *>::iterator it = xClean.begin();
	while (it != xClean.end()){
		if (xDirty.count(it->first) == 0){
			//CLEAN
			res = toJSONKey(
					res,
					&remLen ,
					it->first.c_str(),
					it->second->type
					);

		} else {
			if (xDirty[it->first]->type != NVS_TYPE_ERASE){
				//DIRTY
				res = toJSONKey(
						res,
						&remLen ,
						it->first.c_str(),
						it->second->type
						);
			}
		}
		it++;
	}

	 it = xDirty.begin();
	while (it != xDirty.end()){
		if (xClean.count(it->first) == 0){
			if (it->second->type != NVS_TYPE_ERASE ){
				//NEW
				res = toJSONKey(
						res,
						&remLen ,
						it->first.c_str(),
						it->second->type
						);
			}
		}
		it++;
	}

	res = json_objClose(
				res,
				&remLen
				);
	res = json_end(
				res,
				&remLen
				);
}


char * NVSJson::toJSONKey(char *dest,  size_t* remLen , const char * key,  nvs_type_t type){

	char * res = dest;
	switch(type){
	case NVS_TYPE_U8:{
		uint8_t v;
		if (get_u8 ( key, &v) == NVS_OK){
			res = json_uint(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_I8:{
		int8_t v;
		if (get_i8 ( key, &v) == NVS_OK){
			res = json_int(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case  NVS_TYPE_U16:{
		uint16_t v;
		if (get_u16 ( key, &v) == NVS_OK){
			res = json_uint(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_I16:{
		int16_t v;
		if (get_i16 ( key, &v) == NVS_OK){
			res = json_int(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_U32:{
		uint32_t v;
		if (get_u32 ( key, &v) == NVS_OK){
			res = json_uint(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_I32:{
		int32_t v;
		if (get_i32 ( key, &v) == NVS_OK){
			res = json_uint(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_U64:{
		uint64_t v;
		if (get_u64 ( key, &v) == NVS_OK){
			res = json_ulong(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_I64:{
		int64_t v;
		if (get_i64 ( key, &v) == NVS_OK){
			res = json_long(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_DOUBLE:{
		double v;
		if (get_double( key, &v) == NVS_OK){
			res = json_double(
					dest,
					key,
					v,
					remLen
					);
		}
		break;
	}
	case NVS_TYPE_STR:{
		char v[NVS_STR_LEN];
		size_t l = NVS_STR_LEN;
		if (get_str( key, v, &l) == NVS_OK){
				res = json_str(
						dest,
						key,
						v,
						remLen
						);
			}
		break;
	}
	case NVS_TYPE_BLOB:
		break;
	case NVS_TYPE_ERASE:
		break;
	case NVS_TYPE_ANY:
		break;
	}

	return res;
}

