/**
 *
 * @copyright &copy; 2010 - 2022, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
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
 * @file    test_diag_cbs_i2c.c
 * @author  foxBMS Team
 * @date    2021-09-29 (date of creation)
 * @updated 2022-10-27 (date of last update)
 * @version v1.4.1
 * @ingroup UNIT_TEST_IMPLEMENTATION
 * @prefix  TEST
 *
 * @brief   Test of the I2C diag handler implementation.
 *
 */

/*========== Includes =======================================================*/
#include "unity.h"
#include "Mockdiag_cfg.h"

#include "diag_cbs.h"
#include "test_assert_helper.h"

TEST_FILE("diag_cbs_i2c.c")

/*========== Definitions and Implementations for Unit Test ==================*/
/** local copy of the #DATA_BLOCK_ERRORSTATE_s table */
static DATA_BLOCK_ERRORSTATE_s test_tableErrorFlags = {.header.uniqueId = DATA_BLOCK_ID_ERRORSTATE};

/** local copy of the #DATA_BLOCK_MOL_FLAG_s table */
static DATA_BLOCK_MOL_FLAG_s test_tableMolFlags = {.header.uniqueId = DATA_BLOCK_ID_MOL_FLAG};

/** local copy of the #DATA_BLOCK_RSL_FLAG_s table */
static DATA_BLOCK_RSL_FLAG_s test_tableRslFlags = {.header.uniqueId = DATA_BLOCK_ID_RSL_FLAG};

/** local copy of the #DATA_BLOCK_MSL_FLAG_s table */
static DATA_BLOCK_MSL_FLAG_s test_tableMslFlags = {.header.uniqueId = DATA_BLOCK_ID_MSL_FLAG};

const DIAG_DATABASE_SHIM_s diag_kpkDatabaseShim = {
    .pTableError = &test_tableErrorFlags,
    .pTableMol   = &test_tableMolFlags,
    .pTableRsl   = &test_tableRslFlags,
    .pTableMsl   = &test_tableMslFlags,
};

/*========== Setup and Teardown =============================================*/
void setUp(void) {
    diag_kpkDatabaseShim.pTableError->i2cPexError = 0;
}

void tearDown(void) {
}

/*========== Test Cases =====================================================*/
void testDiagI2cPex(void) {
    /* reset event sets the I2C in ok mode */
    DIAG_I2cPex(DIAG_ID_I2C_PEX_ERROR, DIAG_EVENT_RESET, &diag_kpkDatabaseShim, 0u);
    TEST_ASSERT_EQUAL(0, diag_kpkDatabaseShim.pTableError->i2cPexError);
    /* ok event must not change the I2C state */
    DIAG_I2cPex(DIAG_ID_I2C_PEX_ERROR, DIAG_EVENT_OK, &diag_kpkDatabaseShim, 0u);
    TEST_ASSERT_EQUAL(0, diag_kpkDatabaseShim.pTableError->i2cPexError);

    /* not ok event sets the i2c pex error back in not ok mode */
    DIAG_I2cPex(DIAG_ID_I2C_PEX_ERROR, DIAG_EVENT_NOT_OK, &diag_kpkDatabaseShim, 0u);
    TEST_ASSERT_EQUAL(1, diag_kpkDatabaseShim.pTableError->i2cPexError);

    /* reset event sets the i2c error back in ok mode */
    DIAG_I2cPex(DIAG_ID_I2C_PEX_ERROR, DIAG_EVENT_RESET, &diag_kpkDatabaseShim, 0u);
    TEST_ASSERT_EQUAL(0, diag_kpkDatabaseShim.pTableError->i2cPexError);
}

/** test against invalid input */
void testDIAG_I2cPexInvalidInput(void) {
    TEST_ASSERT_FAIL_ASSERT(DIAG_I2cPex(DIAG_ID_MAX, DIAG_EVENT_OK, &diag_kpkDatabaseShim, 0u));
    TEST_ASSERT_FAIL_ASSERT(DIAG_I2cPex(DIAG_ID_I2C_PEX_ERROR, 42, &diag_kpkDatabaseShim, 0u));
    TEST_ASSERT_FAIL_ASSERT(DIAG_I2cPex(DIAG_ID_I2C_PEX_ERROR, DIAG_EVENT_OK, NULL_PTR, 0u));
}

void testDIAG_I2cPexDoNothingOnWrongId(void) {
    /* Use a wrong ID to make sure, that this does not alter the I2C entry */
    uint8_t testValue                             = 42;
    diag_kpkDatabaseShim.pTableError->i2cPexError = testValue;
    DIAG_I2cPex(DIAG_ID_CELL_VOLTAGE_OVERVOLTAGE_RSL, DIAG_EVENT_RESET, &diag_kpkDatabaseShim, 0u);
    TEST_ASSERT_EQUAL(testValue, diag_kpkDatabaseShim.pTableError->i2cPexError);
}
