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
  #define FARM "demox"
#endif

// Serial number. Must be lower case.
#ifndef SERIAL_NUMBER
  #define SERIAL_NUMBER "rb000030"
#endif
//Enter the number of papers you want to upload in seconds (s)
#define time_sleep 180     //seconds (s)
//Enter the length of the pipe you are using in centimeters (cm), Types in use: 110cm, 90cm, 70cm
#define tube_length 90    //centimeters (cm)
#define wt53r_ttl 1
#define vl53l1 0

/** MQTT
 * Global broker: mqtt.agriconnect.vn
 */
//If you want to use it for Agriconnet server, please turn on define me

#define MQTT_HOST "tcp://mqtt.agriconnect.vn"         // MQTT broker
//#define MQTT_USER "mqttnode"                          // User - connect to MQTT broker
//#define MQTT_PASS "congamo"

#define MQTT_USER "node"                          // User - connect to MQTT broker
#define MQTT_PASS "654321"
//If you want to use it for saty server, please turn on define me

//#define MQTT_HOST "tcp://local.saty.ai"           		// MQTT broker
//#define MQTT_USER "devicetest1"                          // User - connect to MQTT broker
//#define MQTT_PASS "test1password"  // Password - connect to MQTT broker

#define MQTT_CLIENT_ID  SERIAL_NUMBER
#define MQTT_PORT 1883

// ADRESS FLASH
#define SET_CHANGE 1
#define DATA_INT_WATER 0x08000000+1024*63
#define DATA_FLOAT_WATER 0x08000000+1024*62
#define DATA_NEGATIVE_WATER  0x08000000+1024*61
#define time_Period ((1000000*time_sleep)/60000)-1;



#endif /* INC_CONFIG_H_ */
