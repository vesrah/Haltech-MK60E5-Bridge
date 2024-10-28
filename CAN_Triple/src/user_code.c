/* 
 * user_code.c - All User Code should be applied here unless specified otherwise.
 *
 * CAN1 is BMW MK60E1/E5 ABS unit
 * CAN2 is Haltech Nexus R3 PDM and AIM MXG2 dash
 * 		https://support.haltech.com/portal/en/kb/articles/haltech-can-pd-16-protocol
 * CAN3 is currently unused
 */

/* File Includes */
#include "user_code.h"
#include "backend_functions.h"
#include "main.h"
#include "snprintf.h"
#include <string.h>
#include "stm32g4xx_hal.h"
/* End File Includes */

/* Variable Declarations */
uint32_t serialnumber;
CAN_ErrorCounts errors;

/* MK60E1/E5 DBC Declarations */
// Wheel speed front left (kph)
float V_WHL_FLH;
// Wheel speed front right (kph)
float V_WHL_FRH;
// Wheel speed rear left (kph)
float V_WHL_RLH;
// Wheel speed rear right (kph)
float V_WHL_RRH;
// Regulation type
/* 
	255 Invalid signal
	64 Dyno active
	32 EBV regulation (electronic brake force distribution)
	16 MSR regulation (engine drag torque control)
	8 HBA regulation (hydraulic brake assist)
	4 DSC/TCS regulation (traction control)
	2 ASR regulation (acceration slip reduction)
	1 ABS regulation
	0 No regulation
*/
uint32_t ST_CLCTR;
// ABS status
/*
	3 Invalid signal
	2 Malfunction
	1 Fallback level
	0 Normal
*/
uint32_t ST_ABS;
// Brake pressure state
/*
	3 Invalid signal
	2 No statement possible (??)
	1 Brake pressure present
	0 No brake pressure
*/
uint32_t ST_BRP;
// Brake switch state
/*
	1 Brake switch active
	0 Brake switch inactive
*/
uint32_t BrakeSwitch;
// Brake pressure input (bar)
uint32_t BRP;
// Vehicle speed (kph)
float V_VEH;
// Vehicle long accel (m/s2)
float ACLN_VEH_LN_DSC;
// Vehicle lateral accel (m/s2)
float ACLN_VEH_ACRO_DSC;
// Yaw rate (deg/s)
float ANGV_YAW_DSC;
// Brake pressure front left (bar, probably shouldn't be there in E1)
uint32_t BRP_WHL_FLH;
// Brake pressure front right (bar, probably shouldn't be there in E1)
uint32_t BRP_WHL_FRH;
// Brake pressure rear left (bar, probably shouldn't be there in E1)
uint32_t BRP_WHL_RLH;
// Brake pressure rear right (bar, probably shouldn't be there in E1)
uint32_t BRP_WHL_RRH;
// Wheel tolerance adjustment front left (%)
float WHL_TOL_FLH;
// Wheel tolerance adjustment front right (%)
float WHL_TOL_FRH;
// Wheel tolerance adjustment rear left (%)
float WHL_TOL_RLH;
// Wheel tolerance adjustment rear right (%)
float WHL_TOL_RRH;
// Wheel tolerance adjustment state
/*
	2 Invalid signal
	1 Adjustment done
	0 Adjustment not done
*/
uint32_t ST_WHL_TOL;
/* End MK60E1/E5 DBC Declarations */

/* Startup Functions */
void events_Startup() {
	setupCANbus(CAN_1, 500000, NORMAL_MODE);
	setupCANbus(CAN_2, 1000000, NORMAL_MODE);
	setCAN_Termination(CAN_1, true);
	setCAN_Termination(CAN_2, false);
	startCANbus(CAN_1);
	startCANbus(CAN_2);
}
/* End Startup Functions */


void onSerialReceive(uint8_t *serialMessage) {
    // What do you want to do when you receive a UART message.. ?
	// printf("%07.4f message received...\r\n",getTimestamp());
}

void onReceive(CAN_Message Message) {
	if (Message.Bus == CAN_1) {
		if (Message.arbitration_id == 0xCE) {
			// Signal: V_WHL_FLH
			// Start bit: 0, Length: 16, Byte Order: little
			V_WHL_FLH = process_float_value(((uint32_t)Message.data[1] << 8) | (uint32_t)Message.data[0], 0xFFFF, true, 0.0625, 0, 3);

			// Signal: V_WHL_FRH
			// Start bit: 16, Length: 16, Byte Order: little
			V_WHL_FRH = process_float_value(((uint32_t)Message.data[3] << 8) | (uint32_t)Message.data[2], 0xFFFF, true, 0.0625, 0, 3);

			// Signal: V_WHL_RLH
			// Start bit: 32, Length: 16, Byte Order: little
			V_WHL_RLH = process_float_value(((uint32_t)Message.data[5] << 8) | (uint32_t)Message.data[4], 0xFFFF, true, 0.0625, 0, 3);

			// Signal: V_WHL_RRH
			// Start bit: 48, Length: 16, Byte Order: little
			V_WHL_RRH = process_float_value(((uint32_t)Message.data[7] << 8) | (uint32_t)Message.data[6], 0xFFFF, true, 0.0625, 0, 3);
		}

		if (Message.arbitration_id == 0x19E) {
			// Signal: ST_CLCTR
			// Start bit: 0, Length: 8, Byte Order: little
			ST_CLCTR = process_raw_value((uint32_t)Message.data[0], 0xFF);

			// Signal: ST_ABS
			// Start bit: 8, Length: 2, Byte Order: little
			ST_ABS = process_raw_value((uint32_t)Message.data[1], 0x3);

			// Signal: ST_BRP
			// Start bit: 40, Length: 2, Byte Order: little
			ST_BRP = process_raw_value((uint32_t)Message.data[5], 0x3);

			// Signal: BrakeSwitch
			// Start bit: 46, Length: 1, Byte Order: little
			BrakeSwitch = process_raw_value(((uint32_t)Message.data[6] << 8) | (uint32_t)Message.data[5], 0x40);

			// Signal: BRP
			// Start bit: 48, Length: 8, Byte Order: little
			BRP = process_raw_value((uint32_t)Message.data[6], 0xFF);
		}

		if (Message.arbitration_id == 0x1A0) {
			// Signal: V_VEH
			// Start bit: 0, Length: 12, Byte Order: little
			V_VEH = process_float_value(((uint32_t)Message.data[1] << 8) | (uint32_t)Message.data[0], 0xFFF, false, 0.1, 0, 3);

			// Signal: ACLN_VEH_LN_DSC
			// Start bit: 16, Length: 12, Byte Order: little
			ACLN_VEH_LN_DSC = process_float_value(((uint32_t)Message.data[3] << 8) | (uint32_t)Message.data[2], 0xFFF, true, 0.025, 0, 3);

			// Signal: ACLN_VEH_ACRO_DSC
			// Start bit: 28, Length: 12, Byte Order: little
			ACLN_VEH_ACRO_DSC = process_float_value(((uint32_t)Message.data[5] << 16) | ((uint32_t)Message.data[4] << 8) | (uint32_t)Message.data[3], 0xFFF0, true, 0.025, 0, 3);

			// Signal: ANGV_YAW_DSC
			// Start bit: 40, Length: 12, Byte Order: little
			ANGV_YAW_DSC = process_float_value(((uint32_t)Message.data[6] << 8) | (uint32_t)Message.data[5], 0xFFF, true, 0.05, 0, 3);
		}

		if (Message.arbitration_id == 0x2B2) {
			// Signal: BRP_WHL_FLH
			// Start bit: 0, Length: 8, Byte Order: little
			BRP_WHL_FLH = process_raw_value((uint32_t)Message.data[0], 0xFF);

			// Signal: BRP_WHL_FRH
			// Start bit: 8, Length: 8, Byte Order: little
			BRP_WHL_FRH = process_raw_value((uint32_t)Message.data[1], 0xFF);

			// Signal: BRP_WHL_RLH
			// Start bit: 16, Length: 8, Byte Order: little
			BRP_WHL_RLH = process_raw_value((uint32_t)Message.data[2], 0xFF);

			// Signal: BRP_WHL_RRH
			// Start bit: 24, Length: 8, Byte Order: little
			BRP_WHL_RRH = process_raw_value((uint32_t)Message.data[3], 0xFF);
		}

		if (Message.arbitration_id == 0x374) {
			// Signal: WHL_TOL_FLH
			// Start bit: 0, Length: 8, Byte Order: little
			WHL_TOL_FLH = process_float_value((uint32_t)Message.data[0], 0xFF, true, 0.1, 1, 3);

			// Signal: WHL_TOL_FRH
			// Start bit: 8, Length: 8, Byte Order: little
			WHL_TOL_FRH = process_float_value((uint32_t)Message.data[1], 0xFF, true, 0.1, 0, 3);

			// Signal: WHL_TOL_RLH
			// Start bit: 16, Length: 8, Byte Order: little
			WHL_TOL_RLH = process_float_value((uint32_t)Message.data[2], 0xFF, true, 0.1, 0, 3);

			// Signal: WHL_TOL_RRH
			// Start bit: 24, Length: 8, Byte Order: little
			WHL_TOL_RRH = process_float_value((uint32_t)Message.data[3], 0xFF, true, 0.1, 0, 3);

			// Signal: ST_WHL_TOL
			// Start bit: 32, Length: 2, Byte Order: little
			ST_WHL_TOL = process_raw_value((uint32_t)Message.data[4], 0x3);
		}
	}

	// Haltech / AIM on CAN 2
	if (Message.Bus == CAN_2) {

	}
	
	if (Message.Bus == CAN_3) {

	}
}

/* Run 2000Hz Functions here */
void events_2000Hz() {
	
}

/* Run 1000Hz Functions here */
void events_1000Hz() {

}

/* Run 500Hz Functions here */
void events_500Hz() {

}

/* Run 200Hz Functions here */
void events_200Hz() {

}

/* Run 100Hz Functions here */
void events_100Hz() {

}

/* Run 50Hz Functions here */
void events_50Hz() {

}

/* Run 20Hz Functions here */
void events_20Hz() {
	
}

/* Run 10Hz Functions here */
void events_10Hz() {

}

/* Run 5Hz Functions here */
void events_5Hz() {

}

/* Run 2Hz Functions here */
void events_2Hz() {

}

/* Run 1Hz Functions here */
void events_1Hz() {
	haltechSendKeepAlive();
}

/* Run Shutdown Functions here */
void events_Shutdown() {

}

void haltechSendKeepAlive() {
	/*
		0:7 - 0:4 => Status (1 = In Firmware)
		0:3 => USB Connected (0 = False)
		0:1 => ID Conflict (0 = False)

		1:7 - 1:3 => Boot Version (0)
		1:1 - 1:0 => Firmware Major Version (1)

		2:7 - 2:0 => Firmware Minor Version (42)

		3:7 - 3:0 => Firmware Bugfix Version (0)

		4:7 - 4:0 => Firmware Release Version (0)
	*/
	uint8_t keepAliveMessage = { 0X10, 0x01, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// PD16 A
	send_message(CAN_2, false, 0x6D5, 8, keepAliveMessage);
	// PD16 B
	send_message(CAN_2, false, 0x6DD, 8, keepAliveMessage);
}