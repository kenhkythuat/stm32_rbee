/*
 * config.h
 *
 *  Created on: Nov 23, 2023
 *      Author: thuanphat_7
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include "main.h"

// Codename of the farm, where we deploy this node to.
#ifndef FARM
  #define FARM "miagri"
#endif

// Serial number. Must be lower case.
#ifndef SERIAL_NUMBER
  #define SERIAL_NUMBER "rb000006"
#endif

/** MQTT
 * Global broker: mqtt.agriconnect.vn
 */
#define MQTT_HOST "tcp://mqtt.agriconnect.vn"           		// MQTT broker
#define MQTT_USER "mqttnode"                          // User - connect to MQTT broker
#define MQTT_PASS "congamo"                        		// Password - connect to MQTT broker
#define MQTT_CLIENT_ID  SERIAL_NUMBER
#define MQTT_PORT 1883
// ADRESS FLASH
#define SET_CHANGE 1
#define DATA_INT_WATER 0x08000000+1024*63
#define DATA_FLOAT_WATER 0x08000000+1024*62
#define DATA_NEGATIVE_WATER  0x08000000+1024*61



#endif /* INC_CONFIG_H_ */