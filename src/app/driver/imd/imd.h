/**
 *
 * @copyright &copy; 2010 - 2021, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
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
 * @file    imd.h
 * @author  foxBMS Team
 * @date    2020-11-20 (date of creation)
 * @updated 2020-11-20 (date of last update)
 * @ingroup DRIVERS
 * @prefix  IMD
 *
 * @brief   API header for the insulation monitoring device
 *
 */

#ifndef FOXBMS__IMD_H_
#define FOXBMS__IMD_H_

/*========== Includes =======================================================*/
#include "general.h"

#include "can_cfg.h"

#include "os.h"

/*========== Macros and Definitions =========================================*/
/** Length of data Queue */
#define IMD_QUEUE_LENGTH (5u)
/** Size of data Queue item: 9: CAN ID + 8 CAN data bytes */
#define IMD_QUEUE_ITEM_SIZE (sizeof(CAN_BUFFERELEMENT_s))

/*========== Extern Constant and Variable Declarations ======================*/
extern QueueHandle_t imd_canDataQueue;
extern StaticQueue_t imd_queueStructure;
extern uint8_t imd_queueStorageArea[IMD_QUEUE_LENGTH * IMD_QUEUE_ITEM_SIZE];

/*========== Extern Function Prototypes =====================================*/

/**
 * @brief   trigger function for the IMD driver state machine.
 * @details This function contains the sequence of events in the IMD state
 *          machine. It must be called time-triggered, every 100ms.
 */
extern void IMD_Trigger(void);

/*========== Externalized Static Functions Prototypes (Unit Test) ===========*/

#endif /* FOXBMS__IMD_H_ */