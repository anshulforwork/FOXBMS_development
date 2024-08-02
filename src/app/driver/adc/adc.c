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
#include "sensordisplay.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*========== Macros and Definitions =========================================*/

#define R_input  (25u)
#define R_burden (150u)
#define sample   (200u)
#define UTV      (80000u)
#define LTV      (40000u)

/*========== Static Constant and Variable Definitions =======================*/

/*************DATA AQISITION CONSTANTS******************************/

float_t terminal_volt_input;
static uint8 terminal_volt_input_int;
float_t terminal_volt_output;
static uint8 terminal_volt_output_int;
float_t terminal_current_input;
static uint8 terminal_current_input_int;
float_t terminal_current_output;
static uint8 terminal_current_output_int;

/****************SENSOR DATA READ FUNCTIONS**************************/

extern float_t Terminal_voltage_Input(float Rin, float R_burd, uint8_t samples);
extern float_t Terminal_voltage_Output(float Rin, float R_burd, uint8_t samples);
extern float_t Terminal_current_Input(float Rin, float R_burd, uint8_t samples);
extern float_t Terminal_current_Output(float Rin, float R_burd, uint8_t samples);

/*************************************SERIAL DATA READ****************************************/

static uint8 voltage_sensor_Input_1[] = "Input voltage Sensor:  ";
//static uint8 voltage_sensor_Unit_1[]   = " volts ";
static uint16_t length_vs1             = sizeof(voltage_sensor_Input_1) / sizeof(voltage_sensor_Input_1[0]);
static uint8 voltage_sensoR_burden_1[] = "Output voltage Sensor:  ";
static uint16_t length_vs2             = sizeof(voltage_sensoR_burden_1) / sizeof(voltage_sensoR_burden_1[0]);
static uint8 current_sensor_Input_1[]  = "Input current Sensor:  ";
//static uint8 Current_sensor_Unit_1[]   = " Amps ";
static uint16_t length_Is1             = sizeof(current_sensor_Input_1) / sizeof(current_sensor_Input_1[0]);
static uint8 current_sensoR_burden_1[] = "Output current Sensor:  ";
static uint16_t length_Is2             = sizeof(current_sensoR_burden_1) / sizeof(current_sensoR_burden_1[0]);
/****************************************************************************************************************/
/**
 * @brief   describes the current state of the conversion
 * @details This variable is used as a state-variable for switching through the
 *          steps of a conversion.
 */
static ADC_STATE_e adc_conversionState = ADC_START_CONVERSION;

static adcData_t adc_adc1RawVoltages[MCU_ADC1_MAX_NR_CHANNELS] = {0};

static DATA_BLOCK_ADC_VOLTAGE_s adc_adc1Voltages = {.header.uniqueId = DATA_BLOCK_ID_ADC_VOLTAGE};

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/**
 * @brief   converts reading from ADC to a voltage in mV.
 * @param   adcCounts       digital value read by ADC
 * @return  voltage in mV
 */
static float_t ADC_ConvertVoltage(uint16_t adcCounts);

/*========== Static Function Implementations ================================*/

static float_t ADC_ConvertVoltage(uint16_t adcCounts) {
    /* AXIVION R_burdine Generic-MissingParameterAssert: adcValue_mV: parameter accepts whole range */

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
}
/************************SENSOR DATA AQUISITION ON SERIAL MONITOR*****************************/
void adc_display() {
    scisendtext(UART3, &voltage_sensor_Input_1[0], length_vs1);
    sciDisplayData(UART3, &terminal_volt_input_int, 1);
    //sciSendByte(UART3, '\r');
    //scisendtext(UART3, &voltage_sensor_Unit_1[0], length_vs1);
    //sciSendByte(UART3, '\n'/);
    wait(0xffff);
    scisendtext(UART3, &voltage_sensoR_burden_1[0], length_vs2);
    sciDisplayData(UART3, &terminal_volt_output_int, 1);
    // sciSendByte(UART3, '\r');
    //scisendtext(UART3, &voltage_sensor_Unit_1[0], length_vs1);
    //sciSendByte(UART3, '\n');
    wait(0xffff);
    scisendtext(UART3, &current_sensor_Input_1[0], length_Is1);
    sciDisplayData(UART3, &terminal_current_input_int, 1);
    //sciSendByte(UART3, '\r');
    //scisendtext(UART3, &Current_sensor_Unit_1[0], length_Is1);
    //sciSendByte(UART3, '\n');
    wait(0xffff);
    scisendtext(UART3, &current_sensoR_burden_1[0], length_Is2);
    sciDisplayData(UART3, &terminal_current_output_int, 1);
    // sciSendByte(UART3, '\r');
    // scisendtext(UART3, &Current_sensor_Unit_1[0], length_Is2);
    // sciSendByte(UART3, '\n');
    wait(0xffff);
}
/***********************************INPUT VOLTAGE SENSOR DATA READ*******************************/
extern float_t Terminal_voltage_Input(float Rin, float R_burd, uint8_t samples) {
    float mean_data_input = 0.0;
    float adc_read_input  = 0.0;

    for (uint16_t i = 0; i <= samples; i++) {
        adc_read_input = ADC_ConvertVoltage(adc_adc1RawVoltages[24].value);
        mean_data_input += adc_read_input;
    }
    mean_data_input         = (mean_data_input / samples);
    terminal_volt_input     = ((mean_data_input * Rin * 1000) / (2.5 * R_burd));
    terminal_volt_input_int = (uint8)terminal_volt_input;
    return terminal_volt_input;
}
/***********************************OUTPUT VOLTAGE SENSOR DATA READ*******************************/
extern float_t Terminal_voltage_Output(float Rin, float R_burd, uint8_t samples) {
    float mean_data_output = 0.0;
    float adc_read_output  = 0.0;

    for (uint16_t i = 0; i <= samples; i++) {
        adc_read_output = ADC_ConvertVoltage(adc_adc1RawVoltages[25].value);
        mean_data_output += adc_read_output;
    }
    mean_data_output         = (mean_data_output / samples);
    terminal_volt_output     = ((mean_data_output * Rin * 1000) / (2.5 * R_burd));
    terminal_volt_output_int = (uint8)terminal_volt_output;
    return terminal_volt_output;
}
/***********************************INPUT CURRENT SENSOR DATA READ*******************************/

extern float_t Terminal_current_Input(float Rin, float R_burd, uint8_t samples) {
    float mean_data_input_current1 = 0.0;
    float adc_read_input_current1  = 0.0;

    for (uint16_t i = 0; i <= samples; i++) {
        adc_read_input_current1 = ADC_ConvertVoltage(adc_adc1RawVoltages[26].value);
        mean_data_input_current1 += adc_read_input_current1;
    }
    mean_data_input_current1   = (mean_data_input_current1 / samples);
    terminal_current_input     = ((mean_data_input_current1 * 5.0 * 2000) / (1024 * R_burd));
    terminal_current_input_int = (uint8)terminal_current_input;
    return terminal_current_input;
}
/****************************OUTPUT CURRENT SENSOR DATA READ***************************************************/
extern float_t Terminal_current_Output(float Rin, float R_burd, uint8_t samples) {
    float mean_data_output_current = 0.0;
    float adc_read_output_current  = 0.0;

    for (uint16_t i = 0; i <= samples; i++) {
        adc_read_output_current = ADC_ConvertVoltage(adc_adc1RawVoltages[27].value);
        mean_data_output_current += adc_read_output_current;
    }
    mean_data_output_current    = (mean_data_output_current / samples);
    terminal_current_output     = ((mean_data_output_current * 5.0 * 2000) / (1024 * R_burd));
    terminal_current_output_int = (uint8)terminal_current_output;
    return terminal_current_input;
}

extern void voltage_Contactor_control_input(void) {
    if (Terminal_voltage_Input(R_input, R_burden, sample) >= (UTV)) {
        /***************CHARGING CONTACTOR OPEN***************************/
        CONT_OpenContactor(0u, CONT_PLUS);
        MCU_Delay_us(200);

    } else if (Terminal_voltage_Input(R_input, R_burden, sample) <= (LTV)) {
        /***************CHARGING CONTACTOR CLOSE***************************/
        CONT_CloseContactor(0u, CONT_PLUS);
        MCU_Delay_us(200);
    }
}
extern void voltage_Contactor_control_output(void) {
    if ((Terminal_voltage_Output(R_input, R_burden, sample) >= (UTV))) {
        /************DISHARGING CONTACTOR CLOSE**************/
        CONT_CloseContactor(0u, CONT_MINUS);
        MCU_Delay_us(500);
    } else if (Terminal_voltage_Output(R_input, R_burden, sample) <= (LTV)) {
        /********************DISHARGING CONTACTOR OPEN********************/
        CONT_OpenContactor(0u, CONT_MINUS);
        MCU_Delay_us(500);
    }
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
