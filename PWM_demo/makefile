 
TEMPLATEROOT = ../common

# compilation flags for gdb
CFLAGS  = -O0 -g 
ASFLAGS = -g 

# object files
OBJS=  $(STARTUP) main.o  \
stm32f30x_it.o 

# STM firmware library 
# Here you can add drivers from the STM library they are in the directory
# ~/stm32f3/STM32F3-Discovery_FW_V1.1.0/Libraries/STM32F30x_StdPeriph_Driver/src/
#
OBJS+= \
stm32f3_discovery.o \
stm32f30x_gpio.o \
stm32f30x_rcc.o \
stm32f30x_exti.o \
stm32f30x_syscfg.o \
stm32f30x_misc.o \
stm32f30x_tim.o 

# include common make file
include $(TEMPLATEROOT)/Makefile.common