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

#include <stdint.h>
/*****************************Macros and definitions**************************/
#define UART3     ((sciBASE_t *)0xFFF7E500U)
#define UART4     ((sciBASE_t *)0xFFF7E700U)
#define Baud_rate (115200)

/*========== Extern Constant and Variable Declarations ======================*/

/*========== Extern Function Prototypes =====================================*/
extern void scidisplay();
extern void sciDisplayData(sciBASE_t *sci, uint8 *text, uint32 length);
extern void scisendtext(sciBASE_t *sci, uint8 *text, uint16_t length);
//extern void wait(uint32 time);

/*========== Externalized Static Functions Prototypes (Unit Test) ===========*/
#ifdef UNITY_UNIT_TEST
#endif
#endif /* SENSORDISPLAY_H_ */
