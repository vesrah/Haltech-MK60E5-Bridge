/*
 * user_code.c - All User Code should be applied here unless specified otherwise.
 *
 * CAN1 is BMW MK60E1 aka MK60psi / MK60E5 ABS unit and Bosch Motorsport Steering Wheel Angle Sensor LWS
 * CAN2 is Haltech Nexus R3 PDM and AIM MXG2 dash
 * 		https://support.haltech.com/portal/en/kb/articles/haltech-can-pd-16-protocol
 * 		https://support.haltech.com/portal/en/kb/articles/haltech-can-ecu-broadcast-protocol
 * 		https://support.haltech.com/portal/en/kb/articles/haltech-can-protocol-specification
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
/* End Variable Declarations */

/* Startup Functions */
void events_Startup()
{
    setupCANbus(CAN_1, 500000, NORMAL_MODE);
    setupCANbus(CAN_2, 1000000, NORMAL_MODE);
    setCAN_Termination(CAN_1, true);
    setCAN_Termination(CAN_2, false);
    startCANbus(CAN_1);
    startCANbus(CAN_2);
}
/* End Startup Functions */

void onSerialReceive(uint8_t *serialMessage)
{
}

void onReceive(CAN_Message Message)
{
    if (Message.Bus == CAN_1)
    {
        // MK60E1/E5 Wheel speeds
        if (Message.arbitration_id == 0xCE)
        {
            send_message(CAN_2, Message.is_extended_id, Message.arbitration_id, Message.dlc, Message.data);
        }

        // MK60E1/E5 System state
        if (Message.arbitration_id == 0x19E)
        {
            send_message(CAN_2, Message.is_extended_id, Message.arbitration_id, Message.dlc, Message.data);
        }

        // Vehicle speed, gforce, yaw
        if (Message.arbitration_id == 0x1A0)
        {
            send_message(CAN_2, Message.is_extended_id, Message.arbitration_id, Message.dlc, Message.data);
        }

        // MK60E1/E5 Wheel tolerance
        if (Message.arbitration_id == 0x374)
        {
            send_message(CAN_2, Message.is_extended_id, 0xCF, Message.dlc, Message.data);
        }
    }
}

/* Run 2000Hz Functions here */
void events_2000Hz()
{
}

/* Run 1000Hz Functions here */
void events_1000Hz()
{
}

/* Run 500Hz Functions here */
void events_500Hz()
{
}

/* Run 200Hz Functions here */
void events_200Hz()
{
}

/* Run 100Hz Functions here */
void events_100Hz()
{
}

/* Run 50Hz Functions here */
void events_50Hz()
{
}

/* Run 20Hz Functions here */
void events_20Hz()
{
}

/* Run 10Hz Functions here */
void events_10Hz()
{
}

/* Run 5Hz Functions here */
void events_5Hz()
{
}

/* Run 2Hz Functions here */
void events_2Hz()
{
}

/* Run 1Hz Functions here */
void events_1Hz()
{
}

/* Run Shutdown Functions here */
void events_Shutdown()
{
}