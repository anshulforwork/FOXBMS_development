/*
 * sensordisplay.h
 *
 *  Created on: 31-May-2024
 *      Author: Aartech_solonocs_gks
 */

#ifndef SENSORDISPLAY_H_
#define SENSORDISPLAY_H_
/***********************************Includes*********************************/

#include "HL_reg_sci.h"
#include "HL_sci.h"
//#include "HL_sys_common.h"
//#include "HL_sys_vim.h"

// #include "math.h"
// #include "stdio.h"

#include <stdint.h>
/*****************************Macros and definitions**************************/
#define UART3 ((sciBASE_t *)0xFFF7E500U)

#define Baud_rate (115200)

// extern uint8 voltage_sensor_Output_1[] = "output voltage";
// extern uint8 current_sensor_Input_2[]  = "Input current";
// extern uint8 current_sensor_Output_2[] = "Output current";

/*========== Extern Constant and Variable Declarations ======================*/

/*========== Extern Function Prototypes =====================================*/
extern void scidisplay();
extern void sciDisplayData(sciBASE_t *sci, uint8 *text, uint32 length);
extern void scisendtext(sciBASE_t *sci, uint8 *text, uint16_t length);
extern void wait(uint32 time);

/*========== Externalized Static Functions Prototypes (Unit Test) ===========*/
#ifdef UNITY_UNIT_TEST
#endif
#endif /* SENSORDISPLAY_H_ */
