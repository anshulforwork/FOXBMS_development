/*
 * sesordisplay.c
 *
 *  Created on: 31-May-2024
 *      Author: Aartech
 */

/*
 * sesordisplay.c
 *
 *  Created on: 31-May-2024
 *      Author: Aartech
 */
/***********************************Includes*********************************/
#include "sensordisplay.h"

#include "stdio.h"

#include <stdint.h>
//sciInit();

/*========== Static Constant and Variable Definitions =======================*/

/*========== Static Function Implementations ================================*/

void scisendtext(sciBASE_t *sci, uint8 *data, uint16_t length) {
    uint8_t i = 0;

    for (i = 0; i < length - 1; i++) {
        sciSendByte(UART3, data[i]);
    }
    sciSendByte(sci, '\r');
}

void sciDisplayData(sciBASE_t *sci, uint8 *text, uint32 length) {
    uint8 txt  = 0;
    uint8 txt1 = 0;

#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
    text = text + (length - 1);
    G
#endif

        while (length--) {
#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
        txt = *text--;
#else
        txt = *text++;
#endif

        txt1 = txt;

        txt &= ~(0xF0);
        txt1 &= ~(0x0F);
        txt1 = txt1 >> 4;

        if (txt <= 0x9) {
            txt += 0x30;
        } else if (txt > 0x9 && txt < 0xF) {
            txt += 0x37;
        } else {
            txt = 0x30;
        }

        if (txt1 <= 0x9) {
            txt1 += 0x30;
        } else if ((txt1 > 0x9) && (txt1 <= 0xF)) {
            txt1 += 0x37;
        } else {
            txt1 = 0x30;
        }

        //  while ((sciREG1->FLR & 0x4) == 4)
        // ;                           /* wait until busy */
        sciSendByte((UART3), txt1); /* send out text   */
                                    // while ((sciREG1->FLR & 0x4) == 4)
                                    // ;                           /* wait until busy */
        sciSendByte(UART3, txt);
        sciSendByte(sci, '\n'); /* send out text   */
    };
}
void wait(uint32 time) {
    time--;
}
