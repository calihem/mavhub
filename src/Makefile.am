include $(top_srcdir)/am/am_incl_common.am

SUBDIRS = application core io lib module protocol sensor ../thirdparty

# additional include paths necessary to compile the C++ program
# AM_CXXFLAGS = 

AM_CPPFLAGS += $(GSTREAMER_CFLAGS) -Wno-variadic-macros -Wno-long-long

SRC_EXTRA_LIBS = 
if COND_USE_GSTREAMER
SRC_EXTRA_LIBS += $(top_builddir)/src/lib/libgstreamer.a
endif

if COND_OPENGL
SRC_EXTRA_LIBS += $(top_builddir)/src/lib/libopengl.a
endif

if COND_OPENCV2
SRC_EXTRA_LIBS += $(top_builddir)/src/lib/libslam.a
endif

THIRDPARTY_EXTRA_LIBS =
if COND_OPENCV2
THIRDPARTY_EXTRA_LIBS += $(top_builddir)/thirdparty/libbrisk.a \
	$(top_builddir)/thirdparty/libagast.a
endif

if COND_LIBLAPACK
if COND_LIBBLAS
THIRDPARTY_EXTRA_LIBS += $(top_builddir)/thirdparty/liblevmar.a \
	$(top_builddir)/thirdparty/libsba.a
endif
endif

bin_PROGRAMS = mavhub
mavhub_SOURCES =  main.cpp

mavhub_LDADD = \
	$(top_builddir)/src/application/libapp.a \
	$(top_builddir)/src/module/libmodule.a \
	$(top_builddir)/src/sensor/libsensor.a \
	$(top_builddir)/src/protocol/libprotocol.a \
	$(top_builddir)/src/core/libcore.a \
	$(top_builddir)/src/io/libio.a \
	$(SRC_EXTRA_LIBS) \
	$(THIRDPARTY_EXTRA_LIBS) \
	$(GSTREAMER_LIBS) \
	$(OPENCV1_LIBS) \
	$(OPENCV2_LIBS) \
	$(AM_LDFLAGS)

#	$(top_builddir)/src/lib/*.a

