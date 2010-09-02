TARGET_1 = mavhub
SRCS_1   = main.cpp \
	logger.cpp \
	protocolstack.cpp \
	uart.cpp \
	protocollayer.cpp \
	mkpackage.cpp \
	network.cpp \
	module/coremod.cpp \
	module/mkrcmod.cpp \
	datacenter.cpp \
	mavshell.cpp

# define any directories containing header files other than /usr/include
INCLUDES = -I. -Imodule -I../mavlink/include

# compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

CXX		= g++
CXX_CFLAGS 	= -Wall -pedantic -D_REENTRANT $(GENDEPFLAGS)
CXX_LDFLAGS 	= -lpthread

# logger flags ( STDOUTLOG, STDERRLOG, FILELOG="${TARGET_1}.log" )
CXX_CFLAGS += -DSTDOUTLOG
# CXX_CFLAGS += -DSTDERRLOG
# CXX_CFLAGS += -DFILELOG="\"${TARGET_1}.log\""

RMFLAGS = -fr

# define object files
OBJS_1 = $(SRCS_1:.cpp=.o)

.PHONY: clean

.cpp.o:
	$(CXX) $(CXX_CFLAGS) $(INCLUDES) -c $< -o $@

all: $(TARGET_1)

$(TARGET_1): $(OBJS_1)
	$(CXX) $(CXX_CFLAGS) $(INCLUDES) $(CXX_LDFLAGS) -o $(TARGET_1) $(OBJS_1)

clean:
	$(RM) $(RMFLAGS) .dep/ $(TARGET_1) $(OBJS_1)

# include the dependency files
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
