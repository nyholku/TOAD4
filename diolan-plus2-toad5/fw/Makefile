#
# 7.7.2021 Modified to compile with gputils
#
VID		    = 0x1D50
PID		    = 0x6020
XTEA_KEY	= ${TOAD4PLUS_DIALON_KEY}

PROJECT		= bootloader
PROCESSOR	= 18F45K50

SRCS		= boot.asm boot_asm.asm usb.asm usb_desc.asm vectors.asm xtea.asm
INCS		= boot_if.inc boot.inc io_cfg.inc usb_defs.inc usb_desc.inc usb.inc
OBJS		= $(foreach f, $(SRCS), $(basename $(f)).o )

HEX		    = $(PROJECT).hex

OBJ_DIR		= _output

ASM		    = gpasm
LINK		= gplink

LKR		= $(PROJECT).lkr

ASMFLAGS	= --mpasm-compatible -p$(PROCESSOR) -c

ifneq ($(VID), "")
ASMFLAGS += -DBOOTLOADER_VID=$(VID)
endif

ifneq ($(PID), "")
ASMFLAGS += -DBOOTLOADER_PID=$(PID)
endif

ifneq ($(XTEA_KEY), "")
ASMFLAGS += -DXTEA_KEY=$(XTEA_KEY)
endif

all: $(HEX)

$(HEX): $(OBJS)
	$(LINK) -s $(LKR) $(OBJS) -m -o $@

%.o: %.asm $(INCS)
	$(ASM) $< -o $@ $(ASMFLAGS)

clean:
	rm -f *.o *.ERR *.LST *.map *.cod *.cof *.hex *.lst
	rm -rf $(OBJ_DIR)

print:
	@echo "SRCS = $(SRCS)"
	@echo "INCS = $(INCS)"
	@echo "OBJS = $(OBJS)"

.PHONY: all clean print
