@echo off

::@set PROTOC_COMMAND=%cd%\..\..\..\..\..\external\nanopb\nanopb-0.4.2-windows-x86\generator-bin\protoc-gen-nanopb.exe

:: Note: as there doesn't seem to be an easy way of checking the version of the nanopb generator (protoc-gen-nanopb binary)
:: there is the possibility of a version mismatch between the generator and the nanopb source files that is used in the firmware sample.
:: If there is a mismatch, there will be a compilation error in the build step.
:: The easiest way to fix such an error is to download the appropriate nanopb generator .zip and extract it to this sample folder.
:: (https://jpa.kapsi.fi/nanopb/download/)

::Protobuf generation setup
@set PROJECT_PATH=%cd%
@set PROTOC_COMMAND=%PROJECT_PATH%\..\..\..\..\..\external\nanopb\nanopb-0.4.2-windows-x86\generator-bin\protoc.exe
if not exist %PROTOC_COMMAND% goto continue >nul

@set PROTOC_GENERATION_DIR=%cd%\..\Src\gadgets\generated\protobuf\
@set ALEXA_EMBEDDED_PATH=%cd%\..\Src\gadgets\Alexa-Gadgets-Embedded
@set INCLUDE_PATH=%ALEXA_EMBEDDED_PATH%\AlexaGadgetsProtobuf\common
@set CUSTOM_PROTO_PATH=%cd%\..\Src\gadgets\proto
@set PROTO_SCRIPT_PATH=%cd%\..\Src\gadgets\script



for /R %ALEXA_EMBEDDED_PATH%\ConnectionHelpers\BLE\Proto %%G in (*.proto) do (
    cd %%~pG
    echo Compiling %%G
    %PROTOC_COMMAND% -I%INCLUDE_PATH%;. --nanopb_out=%PROTOC_GENERATION_DIR% %%~nG%%~xG
)

for /R %ALEXA_EMBEDDED_PATH%\AlexaGadgetsProtobuf %%G in (*.proto) do (
    cd %%~pG
    :: skip the \common folder
    echo "%%G" | FIND /I "common" 1>NUL) || (
        echo Compiling %%G
        %PROTOC_COMMAND% -I%INCLUDE_PATH%;. --nanopb_out=%PROTOC_GENERATION_DIR% %%~nG%%~xG
    )
)
echo generate common
for /R %ALEXA_EMBEDDED_PATH%\AlexaGadgetsProtobuf\common %%G in (*.proto) do (
    cd %%~pG
    echo Compiling %%G
    %PROTOC_COMMAND% -I. --nanopb_out=%PROTOC_GENERATION_DIR% %%~nG%%~xG
)

echo generation custom
for /R %CUSTOM_PROTO_PATH% %%G in (*.proto) do (
    cd %%~pG
    echo Compiling %%G
    %PROTOC_COMMAND% -I%INCLUDE_PATH%;. --nanopb_out=%PROTOC_GENERATION_DIR% %%~nG%%~xG
)

echo ------------------
cd ..\..\..\Keil_5
echo %cd%
python %PROTO_SCRIPT_PATH%\add_src_to_project.py ".\ble_app_ags.uvprojx" "..\Src\gadgets\generated\protobuf"

exit

::check tools
:continue
(
    echo ---------------------------------------------------------------
    echo "nanopb not found.
    echo  Please download platform-specific nanopb from here https://jpa.kapsi.fi/nanopb/download/
    echo  Extract the files to external folder.
    echo  nanopb required by this project."
    echo ---------------------------------------------------------------
    exit
)
:end