@REM Copyright (c) 2010 - 2022, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
@REM All rights reserved.
@REM
@REM SPDX-License-Identifier: BSD-3-Clause
@REM
@REM Redistribution and use in source and binary forms, with or without
@REM modification, are permitted provided that the following conditions are met:
@REM
@REM 1. Redistributions of source code must retain the above copyright notice, this
@REM    list of conditions and the following disclaimer.
@REM
@REM 2. Redistributions in binary form must reproduce the above copyright notice,
@REM    this list of conditions and the following disclaimer in the documentation
@REM    and/or other materials provided with the distribution.
@REM
@REM 3. Neither the name of the copyright holder nor the names of its
@REM    contributors may be used to endorse or promote products derived from
@REM    this software without specific prior written permission.
@REM
@REM THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
@REM AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@REM IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
@REM DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
@REM FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
@REM DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
@REM SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
@REM CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
@REM OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@REM OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@REM
@REM We kindly request you to use one or more of the following phrases to refer to
@REM foxBMS in your hardware, software, documentation or advertising materials:
@REM
@REM - "This product uses parts of foxBMS&reg;"
@REM - "This product includes parts of foxBMS&reg;"
@REM - "This product is derived from foxBMS&reg;"

@setlocal enableextensions enabledelayedexpansion

@pushd %~dp0
@CALL :NORMALIZE_PATH "%~dp0\..\"
@SET BAUHAUS_CONFIG=%NORMALIZED_PATH%
@ECHO Setting 'BAUHAUS_CONFIG' to '%BAUHAUS_CONFIG%'

@for /f "tokens=*" %%s in (%~dp0..\..\..\conf\env\paths_win32.txt) do @(
    @echo %%s | findstr /i "bauhaus">nul && (@SET BAUHAUS_DIR=%%s)
)

@REM ensure output directory
@SET AXIVION_BUILD_DIR=%~dp0..\..\..\build\axivion
@IF NOT EXIST %AXIVION_BUILD_DIR% (
    @mkdir %AXIVION_BUILD_DIR%
)

@REM prefer the perform_tests.exe that is found on PATH
@SET TEST_EXE=perform_tests.exe
@ECHO Searching program '%TEST_EXE%'
@WHERE %TEST_EXE% 1>NUL 2>NUL
@IF %ERRORLEVEL% neq 0 (
    @ECHO '%TEST_EXE%' is not in %%PATH%%
    @SET TEST_RUNNER=%BAUHAUS_DIR%\%TEST_EXE%
) ELSE (
    @SET TEST_RUNNER=%TEST_EXE%
)

@IF exist %TEST_RUNNER% (
    @ECHO Using '%TEST_RUNNER%'
) ELSE (
    @WHERE %TEST_EXE% 1>NUL 2>NUL
    @IF %ERRORLEVEL% neq 0 (
        @ECHO Could not find program '%TEST_RUNNER%'
        @EXIT /b 1
    )
)
@echo Found '%TEST_RUNNER%'
%TEST_RUNNER% %* --language C11 ti-cgt-arm_20.2.6.lts/*.c
@SET TEST_RUNNER_EXIT_LEVEL=%ERRORLEVEL%

@SET BAUHAUS_CONFIG=
@popd
@EXIT /B %TEST_RUNNER_EXIT_LEVEL%

:NORMALIZE_PATH
    @SET NORMALIZED_PATH=%~dpfn1
    @EXIT /B
