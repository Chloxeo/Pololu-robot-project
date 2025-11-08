###############################################################################
# Binary to build

PRG            = main
OBJ            = main.o print.o OLED.o SH1106.o m_usb.o macro.o

###############################################################################
# Path to AVR toolchain

ifneq (,$(wildcard /etc/redhat-release))
# Red Hat and CentOS have /etc/redhat-release
	AVR_TOOLCHAIN = /usr/local/arduino-1.8.19/hardware/tools/avr
else
# Debian and others
	AVR_TOOLCHAIN = /usr
endif

###############################################################################
# GCC parameters

CC             = $(AVR_TOOLCHAIN)/bin/avr-gcc
OBJCOPY        = $(AVR_TOOLCHAIN)/bin/avr-objcopy
OBJDUMP        = $(AVR_TOOLCHAIN)/bin/avr-objdump
SIZE           = $(AVR_TOOLCHAIN)/bin/avr-size

MCU_TARGET     = atmega32u4
OPTIMIZE       = -Os

DEFS           = -DF_CPU=16000000L
LIBS           = 

###############################################################################
# avrdude parameters

ifneq (,$(wildcard /etc/redhat-release))
CONFIG_FILE    = $(AVR_TOOLCHAIN)/etc/avrdude.conf
else
CONFIG_FILE    = /etc/avrdude.conf
endif

CPU_TYPE       = $(MCU_TARGET)
SERIAL_PORT    = /dev/ttyACM0
BAUDRATE       = 115200
PROGRAMMER_TYPE= avr109

###############################################################################
# You should not have to change anything below here.

CFLAGS         = -g -Wall -Werror $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) --std=c99
LDFLAGS        = -mmcu=$(MCU_TARGET)
#LDFLAGS        = -mmcu=$(MCU_TARGET) -Wl,-Map,$(PRG).map

all: hex

upload: hex
	$(AVR_TOOLCHAIN)/bin/avrdude -C $(CONFIG_FILE) -p $(CPU_TYPE) \
	-P $(SERIAL_PORT) \
	-c $(PROGRAMMER_TYPE) -b $(BAUDRATE) -U flash:w:$(PRG).hex

$(PRG).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(SIZE) $@

%.o : %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf *.o $(PRG).elf $(PRG).bin *.hex *.bak
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)
	rm -rf *~ *eeprom*

distclean: clean
	rm -rf $(PRG).hex

# Rules for building the rom images

hex:  $(PRG).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

###############################################################################
