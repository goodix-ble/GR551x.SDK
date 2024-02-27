@echo off
::set protoc bin path
@set PROTOC_BIN_PATH=%1

::set protoc generation directory
@set PROTOC_GENERATION_DIR=%7
@set INCLUDE_PATH=%2;%3;%4;%5;%6;

::start compilation
%PROTOC_BIN_PATH% -I%INCLUDE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% %2\*.proto || exit /b
%PROTOC_BIN_PATH% -I%INCLUDE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% %3\*.proto || exit /b
%PROTOC_BIN_PATH% -I%INCLUDE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% %4\*.proto || exit /b
%PROTOC_BIN_PATH% -I%INCLUDE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% %5\*.proto || exit /b
%PROTOC_BIN_PATH% -I%INCLUDE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% %6\*.proto || exit /b
::%PROTOC_BIN_PATH% -I%INCLUDE_PATH% --nanopb_out=%PROTOC_GENERATION_DIR% accessories.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% common.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% device.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% firmware.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% directiveParser.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% directiveHeader.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% eventParser.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% eventHeader.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% alexaDiscoveryDiscoverDirective.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% alexaDiscoveryDiscoverDirectivePayload.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% alexaDiscoveryDiscoverResponseEvent.proto || exit /b
::%PROTOC_BIN_PATH% -I. -I. -I%2 -I. -I%3 -I. -I%4 -I. -I%5 -I. -I%6 --nanopb_out=%PROTOC_GENERATION_DIR% alexaDiscoveryDiscoverResponseEventPayload.proto || exit /b
