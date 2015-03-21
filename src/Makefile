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

# This is necessary so that pk2cmd (used by 'load' script) is found and for the
# pk2cmd to find its support files (it seems to search current dir which is wrong)
PATH := ${PATH}:/Users/nyholku/TOAD3/pk2cmd

# The output file name
TARGET = toad4.hex

# The source files that make up the project go here 
//SRCS =	 stepper.c cmdQueue.c cmdInterp.c usartirq.c main.c tests.c stepperirq.c usbcdc.c
SRCS =	 stepper.c cmdInterp.c stepperirq.c main.c usbcdc.c toad4.c
#
 
# The libraries that are used go here
LIBS =  libc18f.lib libio18f4550.lib libm18f.lib libsdcc.lib 

# Where to find the compiler
SDCC = /Users/nyholku/sdcc-3.4.0/bin/sdcc

# Compiler flags go here
SDCCFLAGS = -V -L /Users/nyholku/sdcc-3.4.0/share/sdcc/non-free/lib/pic16 -Wl,-m,-s18f4550.lkr -mpic16 -p18f4550 --disable-warning 85 --std-sdcc99 --obanksel=3 --use-non-free

# Where to store the target/intermediate/temporary/object files
OBJDIR = ../obj

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
  
# Link the compiled files and libraries    
$(OBJDIR)/$(TARGET): $(addprefix $(OBJDIR)/, $(SRCS:.c=.o)) 
	$(SDCC) $(SDCCFLAGS) -o $(OBJDIR)/$(TARGET) $(addprefix $(OBJDIR)/, $(SRCS:.c=.o)) $(LIBS)
	./load
#
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
all:	$(OBJDIR)/$(TARGET)
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
	rm -f $(OBJDIR)/$(TARGET)
#	
# cleanall deletes all in the object directory, do not use this if target dir == source dir
cleanall:
	rm $(OBJDIR)/*

#-------------------------------------------------------------------------------
	
   
    
    