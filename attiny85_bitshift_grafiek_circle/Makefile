# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

 

DEVICE     = attiny85
PROGRAMMER = -c avrispmkII -P usb
OBJECTS    = main.o
FUSES      = -U hfuse:w:0xdf:m -U lfuse:w:0xe2:m -U efuse:w:0xff
READFUSES  = -U hfuse:r:high.hex:i -U lfuse:r:low.hex:i -U efuse:r:high.hex:i -v

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -mmcu=$(DEVICE)

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@ -std=c99

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

readfuse:
	$(AVRDUDE) $(READFUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
