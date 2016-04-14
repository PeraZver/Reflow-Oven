MCU			 = atmega32u2
ARCH		 = AVR8
BOARD 		 = NONE
F_CPU 		 = 8000000
F_USB		 = $(F_CPU)
OPTIMIZATION = s
TARGET		 = MyPID
SRC			 = $(TARGET).c Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH 	 = ./LUFA
CC_FLAGS	 = -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS	 =
SERIAL		 =	com4
DFU			 = batchisp -device atmega32u2 -hardware usb

myclean: 
	rm -f *.o *.elf *.d *.bin *.lss *.sym *.hex *.eep *.map

all:
# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk


program_flip: all
	$(DFU) -operation erase f memory flash blankcheck loadbuffer $(TARGET).hex program verify
	echo "Converting EEP file..."
	mv $(TARGET).eep EE_$(TARGET).hex
	$(DFU) -operation memory eeprom loadbuffer EE_$(TARGET).hex program verify
	$(DFU) -operation start reset 0

program_avrdude: all   # with avrdude
	avrdude -p usb82 -b 19200 -P $(SERIAL) -c avrisp -v -e -u -U flash:w:$(TARGET).hex -F

