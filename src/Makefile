#-------------------------------------------------------------------------------
#
# Copyright (c) 2011, Kustaa Nyholm / SpareTimeLabs
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this list
# of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice, this
# list of conditions and the following disclaimer in the documentation and/or other
# materials provided with the distribution.
#
# Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
# contributors may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.
#
# Everyone should have this
.SUFFIXES:


HWVERSION := HW4

ifeq ($(HWVERSION), HW4)
	HI_SPEED_IRQ_ASM :=hi_speed_irq-hw4.asm
else
	HI_SPEED_IRQ_ASM :=hi_speed_irq.asm
endif

# The output file name
TARGET = toad4

# This is necessary so that pk2cmd (used by 'load' script) is found and for the
# pk2cmd to find its support files (it seems to search current dir which is wrong)
PATH := ${PATH}

UPLOADER := java -cp ~/EazyCNC-Project/classes:/Volumes/Partition2/Users/nyholku/EazyCNC-Project/lib/purejavahidapi-0.0.12.jar:/Volumes/Partition2/Users/nyholku/EazyCNC-Project/lib/jna-5.6.0.jar diolanupdater.UpdateFirmware
ENCODER := /Volumes/Partition2/Users/nyholku/diolan-plus2/encoder/build/encoder
HEXMATE :=  /Applications/microchip/xc8/v2.32/pic/bin/hexmate

BOOTLOADER := ../diolan-plus2-toad5/fw/bootloader.hex


# The source files that make up the project go here

SRCS = main.c toad4.c usb_hid.c usb_core.c usb_pic_defs.c usb_user_config.c command_queue.c crt0iz_toad4.c swuart.c

# The libraries that are used go here
LIBS =  libc18f.lib libm18f.lib libsdcc.lib

# Where to find the compiler
SDCC = /Volumes/Partition2/Users/nyholku/sdcc340/bin/sdcc

# Compiler flags go here
# --use-crt=crt0.o
SDCCFLAGS = "-Wl -f 0xffff" -DTOAD_HW_VERSION=${HWVERSION} --verbose --no-crt --ivt-loc=0x800 -V -L /Volumes/Partition2/Users/nyholku/sdcc340/non-free/lib/pic16 -Wa,-S,0 -Wl,-m,-s18f45k50.lkr -mpic16 -p18f45k50 --disable-warning 85 --std-sdcc99 --obanksel=3 --use-non-free

# Where to store the target/intermediate/temporary/object files
OBJDIR = ../obj



.PHONY: toad4-4
toad4-4: $(OBJDIR)/$(TARGET).hex
toad4-4: HI_SPEED_IRQ_ASM :=hi_speed_irq-hw4.asm
toad4-4: HWVERSION := HW4

.PHONY: toad4-5
toad4-5: $(OBJDIR)/$(TARGET).hex
toad4-5: HI_SPEED_IRQ_ASM :=hi_speed_irq.asm
toad4-4: HWVERSION := HW5
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
	@echo $(PATH)
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
	$(HEXMATE) -o$(OBJDIR)/$(TARGET)-pickit.hex -fill=0xFFFF@0xF00001:0xF000FF -fill=0xA5@0xF00000:0xF00000 -fill=0x00@0x300000:0x30000D $(OBJDIR)/$(TARGET)-normalized.hex ../obj/bootloader-normalized.hex
# encode the bootloader for bootloading purposes, suppress output so as NOT to reveal the secret key
	$(ENCODER) -ix $(OBJDIR)/$(TARGET)-normalized.hex -ox $(OBJDIR)/$(TARGET)-encoded.hex -e ${TOAD4PLUS_DIALON_KEY2}
# upload the encoded hex file using the bootload process
	$(UPLOADER) $(OBJDIR)/$(TARGET)-encoded.hex

# Compile the high speed interrupt asm file
$(OBJDIR)/hi_speed_irq.o: ${HI_SPEED_IRQ_ASM}
	gpasm -D TOAD_HW_VERSION=${HWVERSION} -o $(OBJDIR)/hi_speed_irq.o -c ${HI_SPEED_IRQ_ASM}

#
#-------------------------------------------------------------------------------
.PHONY: load
load:
	$(UPLOADER) $(OBJDIR)/$(TARGET)-encoded.hex
#-------------------------------------------------------------------------------
#
# Automatic generation of dependencies
#
# This magic code fragment from GNU make manual uses the SDCC compiler -M option
# to create a Makefile fragment for each C-source file describing the dependencies.
#
# Traditionally these fragments have the type '.d' but SDCC seems to delete them
# when it compiles files, so I use '.dep' here.
#
# Also SDCC '-M' option produces wrong dependency for the file being compiled
# in the sense that it does not contain the path, only the filename. Hence
# the 'sed' command has been mangled to inject the missing path to the fragment.
#

# First include the dependencies
include $(addprefix $(OBJDIR)/, $(SRCS:.c=.dep))

# Then recreate them
$(OBJDIR)/%.dep: %.c $$(@D)/.f
	 @set -e; rm -f $@; \
	  $(SDCC) -c -M $(SDCCFLAGS) $< > $@.$$$$; \
	  sed -e '1 s,^,$(OBJDIR)/,' -e 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
     rm -f $@.$$$$

#------------------------------------------------------------------------------
#
# pretty standard default target
#
all:	$(OBJDIR)/$(TARGET).hex
#
#-------------------------------------------------------------------------------
#
# pretty standard clean that attempts to delete all that this Makefile may left behind
#
clean:
	rm -f $(OBJDIR)/*.rel
	rm -f $(OBJDIR)/*.lnk
	rm -f $(OBJDIR)/*.S19
	rm -f $(OBJDIR)/*.map
	rm -f $(OBJDIR)/*.mem
	rm -f $(OBJDIR)/*.asm
	rm -f $(OBJDIR)/*.rst
	rm -f $(OBJDIR)/*.sym
	rm -f $(OBJDIR)/*.lst
	rm -f $(OBJDIR)/*.o
	rm -f $(OBJDIR)/*.dep
	rm -f $(OBJDIR)/*.hex
#
# cleanall deletes all in the object directory, do not use this if target dir == source dir
cleanall:
	rm $(OBJDIR)/*

#-------------------------------------------------------------------------------
