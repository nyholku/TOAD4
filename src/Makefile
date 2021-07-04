
OBJDIRS = ../obj-toad4 ../obj-toad5

ifeq (src,$(notdir $(CURDIR)))
include target.mk
else
#----- End Boilerplate





# upload the encoded hex file using the bootload process

VPATH = $(SRCDIR)
#Normal makefile rules here

# The output file name
TARGET = toad4

# This is necessary so that pk2cmd (used by 'load' script) is found and for the
# pk2cmd to find its support files (it seems to search current dir which is wrong)
PATH := ${PATH}

UPLOADER := java -cp ~/EazyCNC-Project/classes:/Volumes/Partition2/Users/nyholku/EazyCNC-Project/lib/purejavahidapi-0.0.12.jar:/Volumes/Partition2/Users/nyholku/EazyCNC-Project/lib/jna-5.6.0.jar diolanupdater.UpdateFirmware
ENCODER := /Volumes/Partition2/Users/nyholku/diolan-plus2/encoder/build/encoder
HEXMATE :=  /Applications/microchip/xc8/v2.32/pic/bin/hexmate



# The source files that make up the project go here

SRCS = main.c toad4.c usb_hid.c usb_core.c usb_pic_defs.c usb_user_config.c command_queue.c crt0iz_toad4.c swuart.c

# The libraries that are used go here
LIBS =  libc18f.lib libm18f.lib libsdcc.lib

# Where to find the compiler
SDCC = /Volumes/Partition2/Users/nyholku/sdcc340/bin/sdcc

# Compiler flags go here
# --use-crt=crt0.o            TOAD_HW_VERSION
SDCCFLAGS = "-Wl -f 0xffff" -DTOAD_HW_VERSION=${HWVERSION} --verbose --no-crt --ivt-loc=0x800 -V -L /Volumes/Partition2/Users/nyholku/sdcc340/non-free/lib/pic16 -Wa,-S,0 -Wl,-m,-s$(SRCDIR)/18f45k50.lkr -mpic16 -p18f45k50 --disable-warning 85 --std-sdcc99 --obanksel=3 --use-non-free

# Where to store the target/intermediate/temporary/object files
OBJDIR = $(CURDIR)
#-------------------------------------------------------------------------------
#
ifeq (obj-toad4,$(notdir $(CURDIR)))
	HI_SPEED_IRQ_ASM :=hi_speed_irq-toad4.asm
	HWVERSION := HW4
	BOOTLOADER := ../diolan-plus2-toad4/fw/bootloader.hex
endif

ifeq (obj-toad5,$(notdir $(CURDIR)))
	HI_SPEED_IRQ_ASM :=hi_speed_irq-toad5.asm
	HWVERSION := HW5
	BOOTLOADER := ../diolan-plus2-toad5/fw/bootloader.hex
endif
#
#
#-------------------------------------------------------------------------------
#
# This ensures that the object directory exists and re-creates it if necessary
#
# This requires make 3.81 or later, delete this section and all expressions that
# refer to .f if you have an older make
#
.SECONDEXPANSION:

# Uses a .f file as a flag file in each directory
%/.f:
	mkdir -p $(dir $@)
	touch $@

# dont' let make remove the flag files automatically
.PRECIOUS: %/.f
#
#-------------------------------------------------------------------------------
#
# Actual rules
#
# Compile the C-files
$(OBJDIR)/%.o: %.c $$(@D)/.f
	$(SDCC) -c $(SDCCFLAGS) $< -o $@

 KEY=${TOAD4PLUS_DIALON_KEY2}

# Link the compiled files and libraries
$(OBJDIR)/$(TARGET).hex: $(addprefix $(OBJDIR)/, $(SRCS:.c=.o))  $(OBJDIR)/hi_speed_irq.o
	$(SDCC) $(SDCCFLAGS) -o $(OBJDIR)/$(TARGET).hex $(addprefix $(OBJDIR)/, $(SRCS:.c=.o)) $(LIBS) $(OBJDIR)/hi_speed_irq.o
# normalize the code filling un-used code memory with 0xFF so that encoding always works on known data
	$(HEXMATE) -o$(OBJDIR)/$(TARGET)-normalized.hex -fill=0xFF@0x0800:0x7FFF r0800-7FFF,$(OBJDIR)/$(TARGET).hex
# sanitise the bootloader by keeping only the first 2kB (there is an extra jump code at 0x800 which overlaps with firmware code)
	$(HEXMATE) -o$(OBJDIR)/bootloader-normalized.hex -fill=0xFF@0x0000:0x07FF r0000-07FF,$(BOOTLOADER) r000802-FFFFFF,$(BOOTLOADER)
# combine the bootloader and firmware to one hex file that can be programmed with pickit ready to run
	$(HEXMATE) -o$(OBJDIR)/$(TARGET)-pickit.hex -fill=0xFFFF@0xF00001:0xF000FF -fill=0xA5@0xF00000:0xF00000 -fill=0x00@0x300000:0x30000D $(OBJDIR)/$(TARGET)-normalized.hex $(OBJDIR)/bootloader-normalized.hex
# encode the bootloader for bootloading purposes, suppress output so as NOT to reveal the secret key
	@$(ENCODER) -ix $(OBJDIR)/$(TARGET)-normalized.hex -ox $(OBJDIR)/$(TARGET)-encoded.hex -e ${TOAD4PLUS_DIALON_KEY2}

# Compile the high speed interrupt asm file
$(OBJDIR)/hi_speed_irq.o: $(SRCDIR)/${HI_SPEED_IRQ_ASM}
	@echo "******************************************************************************** >>> " $(SRCDIR)/${HI_SPEED_IRQ_ASM}
	gpasm -D TOAD_HW_VERSION=${HWVERSION} -o $(OBJDIR)/hi_speed_irq.o -c $(SRCDIR)/${HI_SPEED_IRQ_ASM}


#----- Begin Boilerplate
endif
