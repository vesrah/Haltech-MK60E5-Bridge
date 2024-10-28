/* 
 * user_code.c - All User Code should be applied here unless specified otherwise.
 * 
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

/* MK60E1/E5 Declarations */
float V_WHL_FLH;
float V_WHL_FRH;
float V_WHL_RLH;
float V_WHL_RRH;
uint32_t ST_CLCTR;
uint32_t ST_ABS;
uint32_t ST_DSC;
uint32_t ST_DCRN_AVL;
uint32_t ST_BRP;
uint32_t BrakeSwitch;
uint32_t BRP;
float V_VEH;
uint32_t ST_VEH_DVCO;
float ACLN_VEH_LN_DSC;
float ACLN_VEH_ACRO_DSC;
float ANGV_YAW_DSC;
uint32_t ALIV_V;
uint32_t CHKSM_V;
uint32_t DISTANCE_1_0;
uint32_t DISTANCE_2_3;
uint32_t DISTANCE_4_5;
uint32_t BRP_WHL_FLH;
uint32_t BRP_WHL_FRH;
uint32_t BRP_WHL_RLH;
uint32_t BRP_WHL_RRH;
float WHL_TOL_FLH;
float WHL_TOL_FRH;
float WHL_TOL_RLH;
float WHL_TOL_RRH;
uint32_t ST_WHL_TOL;

/* Startup Functions */
void events_Startup(){
	setupCANbus(CAN_1, 1000000, NORMAL_MODE);
	setupCANbus(CAN_2, 1000000, NORMAL_MODE);
	setupCANbus(CAN_3, 1000000, NORMAL_MODE);
	setCAN_Termination(CAN_1, true);
	setCAN_Termination(CAN_2, true);
	setCAN_Termination(CAN_3, true);
	startCANbus(CAN_1);
	startCANbus(CAN_2);
	startCANbus(CAN_3);
}
/* End Startup Functions */


void onSerialReceive(uint8_t *serialMessage) {
    // What do you want to do when you receive a UART message.. ?
	//printf("%07.4f message received...\r\n",getTimestamp());
  }

void onReceive(CAN_Message Message) {
	// What do you want to do when you receive a CAN message.. ?	
	if (Message.Bus == CAN_1){
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

			// Signal: ST_DSC
			// Start bit: 10, Length: 3, Byte Order: little
			ST_DSC = process_raw_value((uint32_t)Message.data[1], 0x1C);

			// Signal: ST_DCRN_AVL
			// Start bit: 32, Length: 4, Byte Order: little
			ST_DCRN_AVL = process_raw_value((uint32_t)Message.data[4], 0xF);

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

			// Signal: ST_VEH_DVCO
			// Start bit: 12, Length: 3, Byte Order: little
			ST_VEH_DVCO = process_raw_value(((uint32_t)Message.data[2] << 8) | (uint32_t)Message.data[1], 0x70);

			// Signal: ACLN_VEH_LN_DSC
			// Start bit: 16, Length: 12, Byte Order: little
			ACLN_VEH_LN_DSC = process_float_value(((uint32_t)Message.data[3] << 8) | (uint32_t)Message.data[2], 0xFFF, true, 0.025, 0, 3);

			// Signal: ACLN_VEH_ACRO_DSC
			// Start bit: 28, Length: 12, Byte Order: little
			ACLN_VEH_ACRO_DSC = process_float_value(((uint32_t)Message.data[5] << 16) | ((uint32_t)Message.data[4] << 8) | (uint32_t)Message.data[3], 0xFFF0, true, 0.025, 0, 3);

			// Signal: ANGV_YAW_DSC
			// Start bit: 40, Length: 12, Byte Order: little
			ANGV_YAW_DSC = process_float_value(((uint32_t)Message.data[6] << 8) | (uint32_t)Message.data[5], 0xFFF, true, 0.05, 0, 3);

			// Signal: ALIV_V
			// Start bit: 52, Length: 4, Byte Order: little
			ALIV_V = process_raw_value(((uint32_t)Message.data[7] << 8) | (uint32_t)Message.data[6], 0xF0);

			// Signal: CHKSM_V
			// Start bit: 56, Length: 8, Byte Order: little
			CHKSM_V = process_raw_value((uint32_t)Message.data[7], 0xFF);
		}

		if (Message.arbitration_id == 0x1A6) {
			// Signal: DISTANCE_1_0
			// Start bit: 0, Length: 16, Byte Order: little
			DISTANCE_1_0 = process_raw_value(((uint32_t)Message.data[1] << 8) | (uint32_t)Message.data[0], 0xFFFF);

			// Signal: DISTANCE_2_3
			// Start bit: 16, Length: 16, Byte Order: little
			DISTANCE_2_3 = process_raw_value(((uint32_t)Message.data[3] << 8) | (uint32_t)Message.data[2], 0xFFFF);

			// Signal: DISTANCE_4_5
			// Start bit: 32, Length: 16, Byte Order: little
			DISTANCE_4_5 = process_raw_value(((uint32_t)Message.data[5] << 8) | (uint32_t)Message.data[4], 0xFFFF);
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

	if (Message.Bus == CAN_2){

	}
	
	if (Message.Bus == CAN_3){

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
	toggleLED(LED_1);
}

/* Run 2Hz Functions here */
void events_2Hz() {

}

/* Run 1Hz Functions here */
void events_1Hz() {
	
}