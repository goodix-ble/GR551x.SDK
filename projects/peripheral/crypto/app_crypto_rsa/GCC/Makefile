#########################################################################################################
# GNU Compiler makefile for Goodix BLE Application Project
# Copyright(C) 2023, Shenzhen Goodix Technology Co., Ltd
#
# Default Location of GCC Compile Ref Files
#
#   {APP_ROOT_DIR} is root directory in GR551x SDK
#
#   1. sdk lib for gcc          : {APP_ROOT_DIR}}/platform/soc/linker/gcc/libble_sdk.a
#   2. symbol file for gcc      : {APP_ROOT_DIR}}/platform/soc/linker/gcc/rom_symbol_gcc.txt
#   3. link file for gcc        : {APP_ROOT_DIR}}/platform/soc/linker/gcc/gcc_linker_xxx.lds
#   4. startup asemmbly file    : (APP_ROOT_DIR)/platform/arch/arm/cortex-m/gcc/startup_gr55xx.s
#########################################################################################################


#########################################################################################################
###                                Different Configuration For Different Project
#########################################################################################################

## Set Vars by Input Value
# target name
    
MAKE_TARGET_NAME :=  app_crypto_rsa

PRJ_C_SRC_FILES:=   \
../../../../../platform/soc/common/gr_system.c  \
../../../../../platform/soc/common/gr_interrupt.c  \
../../../../../platform/soc/common/gr_platform.c  \
../../../../../platform/soc/src/gr_soc.c  \
../../../../../platform/boards/board_SK.c  \
../../../../../drivers/src/app_dma.c  \
../../../../../drivers/src/app_pwr_mgmt.c  \
../../../../../drivers/src/app_gpiote.c  \
../../../../../drivers/src/app_io.c  \
../../../../../drivers/src/app_uart.c  \
../../../../../drivers/src/app_uart_dma.c  \
../../../../../components/libraries/utility/utility.c  \
../../../../../components/libraries/app_timer/app_timer.c  \
../../../../../components/libraries/ring_buffer/ring_buffer.c  \
../../../../../components/libraries/pmu_calibration/pmu_calibration.c  \
../../../../../components/libraries/app_log/app_log.c  \
../../../../../components/libraries/app_error/app_error.c  \
../../../../../components/libraries/app_assert/app_assert.c  \
../../../../../components/libraries/app_key/app_key.c  \
../../../../../components/libraries/app_key/app_key_core.c  \
../../../../../components/libraries/app_error/cortex_backtrace.c  \
../../../../../components/libraries/fault_trace/fault_trace.c  \
../../../../../external/segger_rtt/SEGGER_RTT.c  \
../Src/main.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/aes.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/aesni.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/arc4.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/aria.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/asn1parse.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/asn1write.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/base64.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/bignum.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/blowfish.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/camellia.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ccm.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/certs.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/chacha20.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/chachapoly.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/cipher.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/cipher_wrap.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/cmac.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ctr_drbg.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/debug.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/des.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/dhm.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ecdh.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ecdsa.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ecjpake.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ecp.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ecp_curves.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/entropy.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/entropy_poll.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/error.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/gcm.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/havege.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/hkdf.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/hmac_drbg.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/md.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/md2.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/md4.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/md5.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/memory_buffer_alloc.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/net_sockets.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/nist_kw.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/oid.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/padlock.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pem.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pk.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pk_wrap.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pkcs5.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pkcs11.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pkcs12.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pkparse.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/pkwrite.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/platform.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/platform_util.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/poly1305.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto_client.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto_ecp.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto_rsa.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto_se.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto_slot_management.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_crypto_storage.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/psa_its_file.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ripemd160.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/rsa.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/rsa_internal.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/sha1.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/sha256.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/sha512.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_cache.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_ciphersuites.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_cli.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_cookie.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_msg.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_srv.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_ticket.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_tls.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/ssl_tls13_keys.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/threading.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/timing.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/version.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/version_features.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509_create.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509_crl.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509_crt.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509_csr.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509write_crt.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/x509write_csr.c  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library/xtea.c  \
../../../../../components/libraries/crypto_lib/src/crypto_aes.c  \
../../../../../components/libraries/crypto_lib/src/crypto_ecc.c  \
../../../../../components/libraries/crypto_lib/src/crypto_ecc_port.c  \
../../../../../components/libraries/crypto_lib/src/crypto_pkc.c  \
../../../../../components/libraries/crypto_lib/src/crypto_pkc_port.c  \
../../../../../components/libraries/crypto_lib/src/crypto_rsa.c  \
../../../../../components/libraries/crypto_lib/src/crypto_rsa_port.c  \
../../../../../components/libraries/crypto_lib/src/crypto_sha256.c  \
../../../../../components/libraries/crypto_lib/src/crypto_gcm.c  \


PRJ_ASM_SRC_FILES :=   \


PRJ_C_INCLUDE_PATH :=   \
../Src/platform  \
../Src/user  \
../Src/config  \
../../../../../components/boards  \
../../../../../components/drivers_ext/gr55xx  \
../../../../../components/drivers_ext/st7735  \
../../../../../components/drivers_ext/vs1005  \
../../../../../components/libraries/app_alarm  \
../../../../../components/libraries/app_assert  \
../../../../../components/libraries/app_error  \
../../../../../components/libraries/app_key  \
../../../../../components/libraries/app_log  \
../../../../../components/libraries/app_queue  \
../../../../../components/libraries/app_timer  \
../../../../../components/libraries/at_cmd  \
../../../../../components/libraries/bsp  \
../../../../../components/libraries/dfu_master  \
../../../../../components/libraries/dfu_port  \
../../../../../components/libraries/gui  \
../../../../../components/libraries/gui/gui_config  \
../../../../../components/libraries/hal_flash  \
../../../../../components/libraries/hci_uart  \
../../../../../components/libraries/pmu_calibration  \
../../../../../components/libraries/ring_buffer  \
../../../../../components/libraries/sensorsim  \
../../../../../components/libraries/fault_trace  \
../../../../../components/libraries/utility  \
../../../../../components/patch/ind  \
../../../../../components/profiles/ams_c  \
../../../../../components/profiles/ancs_c  \
../../../../../components/profiles/ans  \
../../../../../components/profiles/ans_c  \
../../../../../components/profiles/bas  \
../../../../../components/profiles/bas_c  \
../../../../../components/profiles/bcs  \
../../../../../components/profiles/bps  \
../../../../../components/profiles/common  \
../../../../../components/profiles/cscs  \
../../../../../components/profiles/cts  \
../../../../../components/profiles/cts_c  \
../../../../../components/profiles/dis  \
../../../../../components/profiles/dis_c  \
../../../../../components/profiles/gls  \
../../../../../components/profiles/gus  \
../../../../../components/profiles/gus_c  \
../../../../../components/profiles/hids  \
../../../../../components/profiles/hrrcps  \
../../../../../components/profiles/hrs  \
../../../../../components/profiles/hrs_c  \
../../../../../components/profiles/hts  \
../../../../../components/profiles/ias  \
../../../../../components/profiles/lls  \
../../../../../components/profiles/ndcs  \
../../../../../components/profiles/otas  \
../../../../../components/profiles/otas_c  \
../../../../../components/profiles/pass  \
../../../../../components/profiles/pass_c  \
../../../../../components/profiles/pcs  \
../../../../../components/profiles/rscs  \
../../../../../components/profiles/rscs_c  \
../../../../../components/profiles/rtus  \
../../../../../components/profiles/sample  \
../../../../../components/profiles/ths  \
../../../../../components/profiles/ths_c  \
../../../../../components/profiles/thscps  \
../../../../../components/profiles/tps  \
../../../../../components/profiles/wechat  \
../../../../../components/sdk/  \
../../../../../drivers/inc  \
../../../../../external/freertos/include  \
../../../../../external/segger_rtt  \
../../../../../platform/include  \
../../../../../platform/soc/include  \
../../../../../platform/arch/arm/cortex-m/cmsis/core/include  \
../Src/config  \
../../../../../components/sdk  \
../../../../../components/app_drivers/inc  \
../../../../../drivers/inc/hal  \
../../../../../platform/boards  \
../../../../../components/libraries/crypto_lib/inc  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/configs  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/include/mbedtls  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/include  \
../../../../../external/mbedtls/mbedtls-2.26.0_hw/library  \


PRJ_C_MICRO_DEFINES :=   \
DRIVER_TEST  \
MBEDTLS_CONFIG_FILE="<mbedtls_config.h>"  \


PRJ_ASM_INCLUDE_PATH :=   \


PRJ_ASM_MICRO_DEFINES :=   \


IS_WIN_OS :=  true


# Set echo cmd
ECHO = @echo

#########################################################################################################
###                                   Common Configuration Area, Change carefully
#########################################################################################################

## Set Compiler (CC/ASM use same compile cmd for now)
CROSS_COMPILE   = arm-none-eabi-
CC              = $(CROSS_COMPILE)gcc
ASM             = $(CROSS_COMPILE)gcc
CPP             = $(CROSS_COMPILE)cpp
LINK            = $(CROSS_COMPILE)gcc
OBJCOPY         = $(CROSS_COMPILE)objcopy

## Set Common Flags for C/ASM
COMMON_COMPILE_FLAGS += -std=gnu99 --inline
COMMON_COMPILE_FLAGS += -ggdb3
COMMON_COMPILE_FLAGS += -ffunction-sections -fdata-sections 
COMMON_COMPILE_FLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16  -mapcs-frame -mthumb-interwork -mthumb -mcpu=cortex-m4
COMMON_COMPILE_FLAGS += -gdwarf-2 -MD

## Set CFLAGS
# Set include path
CFLAGS += $(foreach inc,$(PRJ_C_INCLUDE_PATH),-I $(inc))
# Set macro-defines Flags
CFLAGS += $(foreach md,$(PRJ_C_MICRO_DEFINES),-D$(md))
CFLAGS += $(COMMON_COMPILE_FLAGS)

## Set ASMFLAGS
ASMFLAGS += $(foreach inc,$(PRJ_ASM_INCLUDE_PATH),-I $(inc))
# Set macro-defines Flags
ASMFLAGS += $(foreach md,$(PRJ_ASM_MICRO_DEFINES),-D$(md))
ASMFLAGS += $(COMMON_COMPILE_FLAGS)

## Set default compile ref files
LINK_SCRIPT  = ./gcc_linker_gr5515.lds

PATCH_FILE  = ../../../../../platform/soc/linker/gcc/rom_symbol_gcc.txt

GCC_STARTUP_ASM_FILE 	= ../../../../../platform/arch/arm/cortex-m/gcc/startup_gr55xx.s

## Set LDFLAGS
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -L../../../../../platform/soc/linker/gcc/ -lble_sdk

## Set compile output directory
BUILD       ?= out
BUILD_OBJ   ?= $(BUILD)/obj
BUILD_LST   ?= $(BUILD)/lst


## Set source files and objects
SRC_C   := $(PRJ_C_SRC_FILES)
SRC_ASM := $(GCC_STARTUP_ASM_FILE) $(PRJ_ASM_SRC_FILES)
OBJ_C   := $(SRC_C:.c=.o)
OBJ_ASM := $(SRC_ASM:.s=.o)

OBJ     := $(OBJ_C) $(OBJ_ASM)
OBJ_ADJUST 	= $(patsubst %.o,$(BUILD_OBJ)/%.o,$(notdir $(OBJ)))


## verbosity switch
V ?= 0
ifeq ($(V),0)
    V_CC = @echo " $(CC) " $<;
    V_ASM = @echo " $(ASM) " $<;
    V_CPP = @echo " $(CPP) " $<;
    V_LINK = @echo " $(LINK) " $<;
    V_OBJCOPY = @echo " $(OBJCOPY) " $<;
else
    V_OPT = '-v'
endif


SRC_PATH += $(dir $(SRC_C))
SRC_PATH += $(dir $(SRC_ASM))
MAKE_PATH = $(foreach n,$(SRC_PATH),:$(n))
vpath %.c ./$(MAKE_PATH)
vpath %.s ./$(MAKE_PATH)
## default make goal
all: mk_path $(BUILD)/$(MAKE_TARGET_NAME).bin $(BUILD)/$(MAKE_TARGET_NAME).hex

##  compile C & asm files
$(BUILD_OBJ)/%.o : %.c
	$(V_CC) $(CC) $(CFLAGS) -c $< -o $@

## how to compile assembly files
$(BUILD_OBJ)/%.o : %.s
	$(V_ASM) $(ASM) $(ASMFLAGS) -c $< -o $@

## make depends
$(BUILD)/$(MAKE_TARGET_NAME).hex: $(BUILD_LST)/$(MAKE_TARGET_NAME).elf
	$(ECHO) "compile hex file ..."
	$(V_OBJCOPY) $(OBJCOPY) -O ihex $< $@

$(BUILD)/$(MAKE_TARGET_NAME).bin: $(BUILD_LST)/$(MAKE_TARGET_NAME).elf
	$(ECHO) "compile binary file ..."
	$(V_OBJCOPY) $(OBJCOPY) -O binary $< $@

$(BUILD_LST)/$(MAKE_TARGET_NAME).elf: $(OBJ_ADJUST)
	$(ECHO) "compile .elf file ..."
	$(V_LINK) $(LINK) $(CFLAGS) -T $(LINK_SCRIPT) $(PATCH_FILE) $(OBJ_ADJUST) $(LDFLAGS) -Wl,-Map=$(BUILD_LST)/$(MAKE_TARGET_NAME).map -o $@


mk_path :
	mkdir -p  $(BUILD)
	mkdir -p  $(BUILD_OBJ)
	mkdir -p  $(BUILD_LST)

flash: $(BUILD)/$(MAKE_TARGET_NAME).bin
	$(ECHO) "Writing $< to the GR55xx-SK board"
	GR5xxx_console.exe eraseall 0 0
	GR5xxx_console.exe program $(BUILD)/$(MAKE_TARGET_NAME).bin 'y' 0x200000 512 0 0

clean:
	rm -rf $(BUILD)

