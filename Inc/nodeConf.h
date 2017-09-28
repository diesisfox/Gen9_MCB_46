/*
 * nodeConf.h
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 *  Edit this file for all node configurations!
 */

/*
 * Node configuration instructions:
 * 1. Make sure all parameters in this files is set properly according to node specifications
 * 2. Implement additional application-layer parsing in do
 * 		- main.c (loc "XXX 1")
 * 3. Implement flush queues in executeCommand()
 * 		- nodeMiscHelpers.c (loc "XXX 2" and loc "XXX 3")
 * 4. Suspend any application layer tasks in shutdown command
 * 		- nodeMiscHelpers.c (loc "XXX 4")
 *
 *
 * If you have any additional tasks/queues/mutexes/semaphores, you should try to add them through CubeMX if possible.
 * As always, ensure that there is sufficient FLASH and HEAP!
 *
 * The task priorities should ALWAYS be set as the following:
 * PRIORITY |	TASK
 * 	HIGH	  watchdogRefresh (if applicable)
 * 	 ^	 	  FreeRTOS_Timer
 *   |		  Can_Processor
 *   |---------------------------------------
 *   |		  Application Layer	Tasks		|
 *   |---------------------------------------
 *  LOW		  IdleTask
 *
 *	Deviation from this priority list may result in unstable node behavior and/or timing failure!
 */

#ifndef NODECONF_H_
#define NODECONF_H_

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "../CAN_ID.h"

#define HB_Interval		1000		// Node heartbeat send interval	(soft ms)
#define WD_Interval		16			// Watdog timer refresh interval (soft ms) | MUST BE LESS THAN 26!!!
#define Node_HB_Interval	2*HB_Interval			// Node's maximum heartbeat interval time

#define MAX_NODE_NUM	 		16		// Maximum number of nodes supported on this system
#define MAX_NODERESET_ATTEMPTS	1		// Maximum number of retries CC will attempt before flagging node as HARD ERRORs

extern uint32_t firmwareString;     // Firmware Version string
extern uint8_t selfNodeID;			// The nodeID of this node
extern uint32_t selfStatusWord;		// Initialize
#define NODE_CONFIGURED

typedef struct {
	uint8_t	 nodeConnectionState;
	uint32_t nodeFirmwareVersion;
	uint32_t nodeStatusWord;
} nodeEntry;

typedef struct {
	uint16_t switchPositions;
	float	 brakePosition;
	float	 accelPosition;
	float	 regenPosition;
} controlVars;

typedef struct{
	uint8_t  nodeID;
	uint8_t	 attempts;
	uint32_t ticks;
} resetParams;


#define CELLT_MID_THRESHOLD 25000000
#define CELLV_MID_THRESHOLD 37000
#define CATTV_MID_THRESHOLD 100000000

#define CELLT_WARN_LOW		6000000
#define CELLT_WARN_HIGH		54000000
#define CELLV_WARN_LOW		2910000
#define CELLV_WARN_HIGH		4190000
#define CUR_WARN_LOW		72000000
#define CUR_WARN_HIGH		-72000000

#ifdef NCR_PROFILE

#define VOLT_WARN_LOW		81480000
#define VOLT_WARN_HIGH		117320000

#else

#define VOLT_WARN_LOW		93120000
#define VOLT_WARN_HIGH		134080000

#endif

#endif /* NODECONF_H_ */
