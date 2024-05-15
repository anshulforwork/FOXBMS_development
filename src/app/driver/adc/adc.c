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
 * @file    adc.c
 * @author  foxBMS Team
 * @date    2019-01-07 (date of creation)
 * @updated 2023-10-12 (date of last update)
 * @version v1.6.0
 * @ingroup DRIVERS
 * @prefix  ADC
 *
 * @brief   Driver for the ADC module.
 *
 */

/*========== Includes =======================================================*/
#include "adc.h"

#include "HL_reg_sci.h"
#include "HL_sci.h"

#include "contactor.h"
#include "database.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/

#define R_input  (20u)
#define R_output (120u)
#define sample   (25u)

#define UART   sciREG4
#define tsize1 2
uint8_t text1[tsize1] = {'A', 'B'};

/*========== Static Constant and Variable Definitions =======================*/

/**
 * @brief   describes the current state of the conversion
 * @details This variable is used as a state-variable for switching through the
 *          steps of a conversion.
 */
static ADC_STATE_e adc_conversionState = ADC_START_CONVERSION;

static adcData_t adc_adc1RawVoltages[MCU_ADC1_MAX_NR_CHANNELS] = {0};

static DATA_BLOCK_ADC_VOLTAGE_s adc_adc1Voltages = {.header.uniqueId = DATA_BLOCK_ID_ADC_VOLTAGE};

/*========== Extern Constant and Variable Definitions =======================*/
extern float_t Terminal_voltage(float Rin, float Rout, uint8_t samples);
extern void voltage_Contactor_control();
extern void sciDisplayText(sciBASE_t *sci, uint8_t *text, uint32_t length);
//extern void wait(uint32_t time);
/*========== Static Function Prototypes =====================================*/

/**
 * @brief   converts reading from ADC to a voltage in mV.
 * @param   adcCounts       digital value read by ADC
 * @return  voltage in mV
 */
static float_t ADC_ConvertVoltage(uint16_t adcCounts);

/*========== Static Function Implementations ================================*/

static float_t ADC_ConvertVoltage(uint16_t adcCounts) {
    /* AXIVION Routine Generic-MissingParameterAssert: adcValue_mV: parameter accepts whole range */

    /** For details to equation see Equation 28 in Technical Reference Manual SPNU563A - March 2018 page 852 */
    float_t result_mV = (((adcCounts + ADC_CONV_OFFSET) * (ADC_VREFHIGH_mV - ADC_VREFLOW_mV)) / ADC_CONV_FACTOR_12BIT) +
                        ADC_VREFLOW_mV;

    return result_mV;
}

/*========== Extern Function Implementations ================================*/

extern void ADC_Control(void) {
    bool conversionFinished = true;

    switch (adc_conversionState) {
        case ADC_START_CONVERSION:
            adcStartConversion(adcREG1, adcGROUP1);
            adc_conversionState = ADC_WAIT_CONVERSION_FINISHED;
            break;

        case ADC_WAIT_CONVERSION_FINISHED:
            conversionFinished = true;
            if (ADC_CONVERSION_ENDDBIT != adcIsConversionComplete(adcREG1, adcGROUP1)) {
                conversionFinished = false;
            }
            if (conversionFinished == true) {
                adc_conversionState = ADC_CONVERSION_FINISHED;
            }
            break;

        /* Start initialization procedure, data sheet figure 106 page 79 */
        case ADC_CONVERSION_FINISHED:
            adcGetData(adcREG1, adcGROUP1, &adc_adc1RawVoltages[0]);
            for (uint8_t i = 0u; i < MCU_ADC1_MAX_NR_CHANNELS; i++) {
                adc_adc1Voltages.adc1ConvertedVoltages_mV[i] =
                    ADC_ConvertVoltage(adc_adc1RawVoltages[i].value);  //FUNCTION FOR ADC
            }
            DATA_WRITE_DATA(&adc_adc1Voltages);
            adc_conversionState = ADC_START_CONVERSION;

            break;

        default:
            /* invalid state */
            FAS_ASSERT(FAS_TRAP);
            break;
    }

    Terminal_voltage(R_input, R_output, sample);
    // sciInit();
    // sciDisplayText(UART, &text1[0], tsize1);

    voltage_Contactor_control();
}
extern float_t Terminal_voltage(float Rin, float Rout, uint8_t samples) {
    float mean_data = 0.0;
    float adc_read  = 0.0;
    float_t terminal_volt;
    for (uint8_t i = 0; i <= samples; i++) {
        adc_read = ADC_ConvertVoltage(adc_adc1RawVoltages[24].value);
        mean_data += adc_read;
    }
    mean_data     = (mean_data / samples);
    terminal_volt = ((mean_data * Rin * 1000) / (2.5 * Rout));
    return terminal_volt;
    // float terminalVoltage (float RIN, float Rb, uint8_t samples) {
    // float meanData = 0.0, adcRead = 0.0;
    // for (uint8_t i = 0; i < samples ; i++){
    // adcRead = readADC();
    // meanData +=adcRead;
    // }
    // meanData /= samples;
    // return ( (meanData * RIN * 1000 ) / (2.5 * Rb) );
    // }
}
extern void voltage_Contactor_control() {
    if (Terminal_voltage(R_input, R_output, sample) >= 10000) {
        CONT_OpenContactor(0u, CONT_PLUS);    //CHARGING CONTACTOR
        CONT_CloseContactor(0u, CONT_MINUS);  //DISHARGING CONTACTOR
        //MCU_Delay_us(5000);
    } else if ((Terminal_voltage(R_input, R_output, sample) <= 1800)) {
        CONT_CloseContactor(0u, CONT_PLUS);  //CHARGING
        CONT_OpenContactor(0u, CONT_MINUS);  //DISCHARGING
        //MCU_Delay_us(5000);
    }
}
extern void sciDisplayText(sciBASE_t *sci, uint8 *text, uint32 length) {
    // sciInit();

    while (length--) {
        /* wait until busy */
        sciSendByte(UART, *text++); /* send out text   */
    };
}

void wait(uint32 time) {
    time--;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
