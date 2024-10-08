/**
 *
 * @copyright &copy; 2010 - 2023, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * We kindly request you to use one or more of the following phrases to refer to
 * foxBMS in your hardware, software, documentation or advertising materials:
 *
 * - &Prime;This product uses parts of foxBMS&reg;&Prime;
 * - &Prime;This product includes parts of foxBMS&reg;&Prime;
 * - &Prime;This product is derived from foxBMS&reg;&Prime;
 *
 */

/**
 * @file    main.c
 * @author  foxBMS Team
 * @date    2019-08-27 (date of creation)
 * @updated 2023-10-12 (date of last update)
 * @version v1.6.0
 * @ingroup GENERAL
 * @prefix  TODO
 *
 * @brief   Main function
 *
 * @details TODO
 */

/*========== Includes =======================================================*/
#include "main.h"

#include "HL_crc.h"
#include "HL_etpwm.h"
#include "HL_gio.h"
#include "HL_het.h"
#include "HL_pinmux.h"
#include "HL_sci.h"  // include it for the Sci Communication
#include "HL_sys_core.h"

#include "adc.h"
#include "checksum.h"
#include "diag.h"
#include "dma.h"
#include "foxmath.h"
#include "i2c.h"
#include "led.h"
#include "lwip_main.h"
#include "master_info.h"
#include "os.h"
#include "pex.h"
#include "pwm.h"
#include "spi.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/
int main(void) {
    MINFO_SetResetSource(getResetSource()); /* Get reset source and clear respective flags */
    _enable_IRQ_interrupt_();
    muxInit();
    gioInit();
    SPI_Initialize();
    adcInit();
    hetInit();
    etpwmInit();
    crcInit();
    LED_SetDebugLed();
    // for the SCI Communication
    sciInit();
    sciSetBaudrate(UART3, 9600);  // Set Baudrate
    sciSetBaudrate(UART4, 9600);  // Set Baudrate
    //_______________________________________________________________
    I2C_Initialize();
    DMA_Initialize();
    PWM_Initialize();
    DIAG_Initialize(&diag_device);
    MATH_StartupSelfTest();
    gioSetDirection(hetPORT2, 0x00200000);
    gioSetBit(hetPORT2, 21, 1);
    //********************// */
    gioSetDirection(gioPORTB, 1 << 6);
    gioSetBit(gioPORTB, 6, 1);
    /************************ */
    PEX_SetPinDirectionOutput(PEX_PORT_EXPANDER2, PEX_PIN15);
    PEX_SetPin(PEX_PORT_EXPANDER2, PEX_PIN15);
    MCU_Delay_us(2000);
    PEX_ResetPin(PEX_PORT_EXPANDER2, PEX_PIN15);
    MATH_StartupSelfTest();
    lwip_run();
    // emacrun();
    const STD_RETURN_TYPE_e checkTimeHasPassedSelfTestReturnValue = OS_CheckTimeHasPassedSelfTest();
    FAS_ASSERT(checkTimeHasPassedSelfTestReturnValue == STD_OK);
    // lwip_run();
    // // emacrun();
    OS_InitializeOperatingSystem();
    if (OS_INIT_PRE_OS != os_boot) {
        /* Could not create Queues, Mutexes, Events and Tasks do not boot further from this point on */
        FAS_ASSERT(FAS_TRAP);
    }

    if (STD_OK != CHK_ValidateChecksum()) {
        if (DIAG_HANDLER_RETURN_OK != DIAG_Handler(DIAG_ID_FLASHCHECKSUM, DIAG_EVENT_NOT_OK, DIAG_SYSTEM, 0u)) {
            /* Could not validate checksum do not boot further from this point on */
            FAS_ASSERT(FAS_TRAP);
        }
    }

    os_schedulerStartTime = OS_GetTickCount();

    OS_StartScheduler();
    /* we must never get here; there is no way to determine the exit state of this program,
     * but for the sake of correctness we exit with an error code */
    return 1;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
