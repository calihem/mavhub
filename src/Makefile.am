include $(top_srcdir)/am/am_incl_common.am

SUBDIRS = application core io module protocol sensor

# additional include paths necessary to compile the C++ program
# AM_CXXFLAGS = 

bin_PROGRAMS = mavhub
mavhub_SOURCES =  main.cpp

mavhub_LDADD = \
	$(top_builddir)/src/application/libapp.a \
	$(top_builddir)/src/module/libmodule.a \
	$(top_builddir)/src/sensor/libsensor.a \
	$(top_builddir)/src/protocol/libprotocol.a \
	$(top_builddir)/src/core/libcore.a \
	$(top_builddir)/src/io/libio.a \
	$(AM_LDFLAGS)
