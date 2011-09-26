include $(top_srcdir)/am/am_incl_common.am

SUBDIRS = application core io lib module protocol sensor

# additional include paths necessary to compile the C++ program
# AM_CXXFLAGS = 

AM_CPPFLAGS += $(GSTREAMER_CFLAGS) -Wno-variadic-macros -Wno-long-long

bin_PROGRAMS = mavhub
mavhub_SOURCES =  main.cpp

if COND_USE_GSTREAMER
	EXTRA_SCR_LIBS += src/lib/libgstreamer.a
endif
if COND_OPENGL
	EXTRA_SCR_LIBS += src/lib/libopengl.a
endif

mavhub_LDADD = \
	$(top_builddir)/src/application/libapp.a \
	$(top_builddir)/src/module/libmodule.a \
	$(top_builddir)/src/sensor/libsensor.a \
	$(top_builddir)/src/protocol/libprotocol.a \
	$(top_builddir)/src/core/libcore.a \
	$(top_builddir)/src/io/libio.a \
	$(EXTRA_SRC_LIBS) \
	$(GSTREAMER_LIBS) \
	$(AM_LDFLAGS)
