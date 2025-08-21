all : flash

TARGET:=main
ADDITIONAL_C_FILES:=adc.c systick.c
TARGET_MCU:=CH32V203G6U6
TARGET_MCU_PACKAGE:=CH32V203G6U6

include $(FUN_SDK_PATH)/ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean


