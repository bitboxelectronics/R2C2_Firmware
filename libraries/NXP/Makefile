CSRC = \
	./Core/CM3/system_LPC17xx.c \
	./Core/CM3/core_cm3.c \
	./Drivers/source/lpc17xx_gpio.c \
	./Drivers/source/lpc17xx_rtc.c \
	./Drivers/source/lpc17xx_libcfg_default.c \
	./Drivers/source/lpc17xx_dac.c \
	./Drivers/source/lpc17xx_nvic.c \
	./Drivers/source/lpc17xx_gpdma.c \
	./Drivers/source/lpc17xx_spi.c \
	./Drivers/source/lpc17xx_uart.c \
	./Drivers/source/lpc17xx_clkpwr.c \
	./Drivers/source/lpc17xx_qei.c \
	./Drivers/source/lpc17xx_adc.c \
	./Drivers/source/lpc17xx_i2s.c \
	./Drivers/source/lpc17xx_can.c \
	./Drivers/source/lpc17xx_systick.c \
	./Drivers/source/lpc17xx_pinsel.c \
	./Drivers/source/lpc17xx_i2c.c \
	./Drivers/source/lpc17xx_rit.c \
	./Drivers/source/lpc17xx_emac.c \
	./Drivers/source/lpc17xx_exti.c \
	./Drivers/source/lpc17xx_pwm.c \
	./Drivers/source/lpc17xx_timer.c \
	./Drivers/source/debug_frmwrk.c \
	./Drivers/source/lpc17xx_wdt.c \
	./Drivers/source/lpc17xx_mcpwm.c \
	./Drivers/source/lpc17xx_ssp.c \

ASRC = ./Core/CM3/startup/gcc/startup_LPC17xx.s

INC = \
	./Core/CM3 \
	./Drivers/include \

PREFIX=arm-none-eabi-

CC=$(PREFIX)gcc
AS=$(PREFIX)as
LD=$(PREFIX)ld
AR=$(PREFIX)ar

CFLAGS=-mcpu=cortex-m3 -mthumb -std=gnu99 -ffunction-sections -Os -g -Wall $(patsubst %,-I%,$(INC))
ASFLAGS=--defsym RAM_MODE=0 --defsym FLASH_MODE=1
LDFLAGS=-print-gc-sections -gc-sections

OUTPUT=liblpc17xx.a

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
