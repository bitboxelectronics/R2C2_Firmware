OUTPUT=liblpcusb.a

CSRC = usbcontrol.c usbhw_lpc.c usbinit.c usbstdreq.c

ASRC =

INC = \
	../NXP/Core/CM3 \
	../NXP/Drivers/include \

PREFIX=arm-none-eabi-

CC=$(PREFIX)gcc
AS=$(PREFIX)as
LD=$(PREFIX)ld
AR=$(PREFIX)ar

CFLAGS=-mcpu=cortex-m3 -mthumb -std=gnu99 -ffunction-sections -Os -g -Wall $(patsubst %,-I%,$(INC))
ASFLAGS=--defsym RAM_MODE=0 --defsym FLASH_MODE=1
LDFLAGS=-print-gc-sections -gc-sections

ALLSRC = $(CSRC) $(ASRC)
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

ALLOBJ = $(addsuffix .o, $(ALLSRCBASE))

all: $(OUTPUT)
clean:
	rm -f $(ALLOBJ)
	rm -f $(notdir $(CSRC:.c=.al))
	rm -f $(OUTPUT)

%.a: $(ALLOBJ)
	@echo "  AR        $@"
	@$(AR) rcs $@ $^

%.elf: $(ALLOBJ)
	@echo "  LINK      $@"
	@$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

define	COMPILE_TEMPLATE
$(notdir $(basename $(1))).o: $(1)
	@echo "  CC        $$@"
	@$(CC) $(CFLAGS) -c -Wa,-adhlns=$$(@:.o=.al) -o $$@ $$<
endef
$(foreach src, $(CSRC), $(eval $(call COMPILE_TEMPLATE, $(src))))

define	ASSEMBLE_TEMPLATE
$(notdir $(basename $(1))).o: $(1)
	@echo "  AS        $$@"
	@$(AS) $(ASFLAGS) -o $$@ $$<
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src))))
