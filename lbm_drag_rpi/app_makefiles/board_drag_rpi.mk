##############################################################################
# Definitions for the Dragino GPS HAT on a Raspberry Pi 3B+ board 
##############################################################################

#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------

#MCU compilation flags
MCU_FLAGS ?= -mcpu=cortex-a53 -mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits

BOARD_C_DEFS = 

BOARD_LDSCRIPT = 

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------
BOARD_C_SOURCES = \
	smtc_modem_hal/smtc_modem_hal.c\
	smtc_hal_drag_rpi/smtc_hal_stack.c\
	smtc_hal_drag_rpi/smtc_hal_gpio.c\
	smtc_hal_drag_rpi/smtc_hal_mcu.c\
	smtc_hal_drag_rpi/smtc_hal_rtc.c\
	smtc_hal_drag_rpi/smtc_hal_rng.c\
	smtc_hal_drag_rpi/smtc_hal_spi.c\
	smtc_hal_drag_rpi/smtc_hal_lp_timer.c\
	smtc_hal_drag_rpi/smtc_hal_trace.c

BOARD_ASM_SOURCES = 

BOARD_C_INCLUDES =  \
	-I.\
	-Ismtc_modem_hal\
	-Ismtc_hal_drag_rpi
