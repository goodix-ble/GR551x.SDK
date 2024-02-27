@echo off
::set protoc bin path
@set PROTOC_BIN_PATH="..\..\..\..\..\external\nanopb-0.4.2-windows-x86\generator-bin"

::set BASE proto file path
@set BASE_PROTO_FILE_PATH_1="..\Src\gadgets\Alexa-Gadgets-Embedded\ConnectionHelpers\BLE\Proto"
@set BASE_PROTO_FILE_PATH_2="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\parser"
@set BASE_PROTO_FILE_PATH_3="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\common"
@set BASE_PROTO_FILE_PATH_4="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Alexa.Discovery\Discover"
@set BASE_PROTO_FILE_PATH_5="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Alexa.Discovery\Discover.Response"
@set NOTI_PROTO_FILE_PATH_1="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Notifications\SetIndicator"
@set NOTI_PROTO_FILE_PATH_2="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Notifications\ClearIndicator"
@set ALERT_PROTO_FILE_PATH_1="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Alerts\SetAlert"
@set ALERT_PROTO_FILE_PATH_2="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Alerts\DeleteAlert"
@set STATELISTENER_PROTO_FILE_PATH="..\Src\gadgets\Alexa-Gadgets-Embedded\AlexaGadgetsProtobuf\Alexa.Gadget.StateListener\StateUpdate"

::set protoc generation directory
@set PROTOC_GENERATION_DIR="..\Src\gadgets\generated\alexa\protobuf"

::start compilation
%PROTOC_BIN_PATH%\protoc.exe -I. -I. -I%BASE_PROTO_FILE_PATH_1% -I. -I%BASE_PROTO_FILE_PATH_2% -I. -I%BASE_PROTO_FILE_PATH_3% -I. -I%BASE_PROTO_FILE_PATH_4% -I. -I%BASE_PROTO_FILE_PATH_5% -I. -I%NOTI_PROTO_FILE_PATH_1% -I. -I%NOTI_PROTO_FILE_PATH_2% -I. -I%ALERT_PROTO_FILE_PATH_1% -I. -I%ALERT_PROTO_FILE_PATH_2% -I. -I%STATELISTENER_PROTO_FILE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% alexaGadgetStateListenerStateUpdateDirective.proto || exit /b
%PROTOC_BIN_PATH%\protoc.exe -I. -I. -I%BASE_PROTO_FILE_PATH_1% -I. -I%BASE_PROTO_FILE_PATH_2% -I. -I%BASE_PROTO_FILE_PATH_3% -I. -I%BASE_PROTO_FILE_PATH_4% -I. -I%BASE_PROTO_FILE_PATH_5% -I. -I%NOTI_PROTO_FILE_PATH_1% -I. -I%NOTI_PROTO_FILE_PATH_2% -I. -I%ALERT_PROTO_FILE_PATH_1% -I. -I%ALERT_PROTO_FILE_PATH_2% -I. -I%STATELISTENER_PROTO_FILE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% alexaGadgetStateListenerStateUpdateDirectivePayload.proto || exit /b
