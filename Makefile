TARGET_1 = mavhub
SRCS_1   = main.cpp \
	logger.cpp \
	lib/file.cpp \
	lib/setting.cpp \
	protocolstack.cpp \
	uart.cpp \
	protocollayer.cpp \
	mkpackage.cpp \
	network.cpp \
	module/coremod.cpp \
	module/testcore.cpp \
	module/mkrcmod.cpp \
	module/fc_mpkg.cpp \
	module/i2csensor.cpp \
	module/senbmp085.cpp \
	datacenter.cpp \
	mavshell.cpp

# logger flags ( STDOUTLOG, STDERRLOG, FILELOG="${TARGET_1}.log" )
CXX_CFLAGS += -DSTDOUTLOG
# CXX_CFLAGS += -DSTDERRLOG
# CXX_CFLAGS += -DFILELOG="\"${TARGET_1}.log\""

# SUBARCH tells the usermode build what the underlying arch is.
SUBARCH := $(shell uname -m | sed -e s/i.86/i386/ -e s/arm.*/arm/ )

# Set the ARCH and CROSS_COMPILE default values
ARCH ?= $(SUBARCH)

# CROSS_COMPILE specify the prefix used for all executables used during compilation.
# CROSS_COMPILE can be set on the command line
# make CROSS_COMPILE=arm-angstrom-linux-gnueabi-
# Alternatively CROSS_COMPILE can be set in the environment.
# export CROSS_COMPILE=arm-angstrom-linux-gnueabi-
CROSS_COMPILE ?= 

# define any directories containing header files other than /usr/include
INCLUDES = -I. -Imodule -I../mavlink/include/huch

# compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# CXX_CFLAGS 	+= -g -Wall -Wextra -pedantic -std=c++98 -O2 -D_REENTRANT $(GENDEPFLAGS)
# CXX_CFLAGS 	+= -g -Wall -Wextra -pedantic -std=c++98 -O2 -D_REENTRANT
CXX_CFLAGS 	+= -Wall -pedantic -O2 -D_REENTRANT
CXX_LDFLAGS 	= -lpthread

# Make variables (AS, LD, CXX, ...)
AS		= $(CROSS_COMPILE)as
LD		= $(CROSS_COMPILE)ld
CXX		= $(CROSS_COMPILE)g++
AR		= $(CROSS_COMPILE)ar
NM		= $(CROSS_COMPILE)nm
STRIP		= $(CROSS_COMPILE)strip
OBJCOPY		= $(CROSS_COMPILE)objcopy
OBJDUMP		= $(CROSS_COMPILE)objdump

RMFLAGS = -fr

# source extension
EXT = cpp
# build directory
BUILDDIR = build-$(ARCH)

# define object files
OBJS_1 = $(patsubst %.$(EXT), $(BUILDDIR)/%.o, $(SRCS_1))

.PHONY: clean

# .cpp.o:
# 	$(CXX) $(GENDEPFLAGS) $(CXX_CFLAGS) $(INCLUDES) -c $< -o $@

all: $(TARGET_1)

$(TARGET_1): $(OBJS_1)
	$(CXX) $(CXX_CFLAGS) $(CXX_LDFLAGS) -o $(TARGET_1) $(OBJS_1)

$(OBJS_1): $(BUILDDIR)/%.o: %.$(EXT)
	$(shell mkdir -p $(dir $(@)) 2>/dev/null)
	$(CXX) $(GENDEPFLAGS) $(CXX_CFLAGS) $(INCLUDES) -c $< -o $@ 

clean:
	$(RM) $(RMFLAGS) .dep/ $(TARGET_1) $(BUILDDIR)

# include the dependency files
ifneq ($(MAKECMDGOALS), clean)
# -include $(DEPS)
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
endif
