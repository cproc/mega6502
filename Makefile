TARGET=mega6502
SERIAL_DEV=/dev/ttyUSB0
SERIAL_LOG_DEV=/dev/ttyUSB1

all: $(TARGET).elf

run_terminal: upload reset_and_attach_to_terminal

run_log: upload reset_and_attach_to_log

reset_and_attach_to_terminal: reset attach_to_terminal

reset_and_attach_to_log: reset attach_to_log

attach_to_terminal:
	expect mega6502_terminal_flow_control.exp picocom -b 600 --imap lfcrlf $(SERIAL_DEV)

attach_to_log:
	picocom -b 4800 --imap lfcrlf $(SERIAL_LOG_DEV)

reset:
	avrdude -p atmega1284p -c arduino -P $(SERIAL_DEV) -b 115200


upload: $(TARGET).bin
	-kill `pgrep -f "picocom -b 600 --imap lfcrlf $(SERIAL_DEV)"`
	sleep 1
	avrdude -p atmega1284p -c arduino -P $(SERIAL_DEV) -b 115200 -D -U flash:w:$(TARGET).bin:r

# For some reason, when using sparse hex files, only part of the data is written to the flash.
# So, binary files are used instead.
$(TARGET).bin: $(TARGET).elf apple1basic.bin wozmonitor.bin
	avr-objcopy -O binary -R .ram --gap-fill 0xFF --pad-to 0xE000 $< $@
	cat apple1basic.bin >> $(TARGET).bin
	cat wozmonitor.bin >> $(TARGET).bin

$(TARGET).elf: $(TARGET).c Makefile
	avr-gcc -mmcu=atmega1284p \
	        -Os \
	        -g \
	        -save-temps \
	        -o $@ \
	        -Wl,-Map=$(TARGET).map,--cref \
	        -Wl,-section-start=.ram=0x801000 \
	        $<

apple1basic.bin:
	wget http://mirrors.apple2.org.za/ftp.apple.asimov.net/images/apple1/apple1basic.bin

dump_flash:
	avrdude -p atmega1284p -c arduino -P $(SERIAL_DEV) -b 115200 -U flash:r:$(TARGET).bin:r

clean:
	rm -f $(TARGET).bin
	rm -f $(TARGET).map
	rm -f $(TARGET).elf
	rm -f $(TARGET).o
	rm -f $(TARGET).s
	rm -f $(TARGET).i
