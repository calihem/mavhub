include $(top_srcdir)/am/am_incl_common.am

# additional include paths necessary to compile the C++ program
# AM_CXXFLAGS = 

bin_PROGRAMS = test_mavhub
test_mavhub_SOURCES =  \
	test_mavhub.cpp \
	utility_test.cpp \
	application/acc_calibration_app_test.cpp

test_mavhub_LDADD = \
	$(top_builddir)/src/application/libapp.a \
	$(top_builddir)/src/core/libcore.a \
	$(top_builddir)/src/io/libio.a \
	$(top_builddir)/src/module/libmodule.a \
	$(top_builddir)/src/protocol/libprotocol.a \
	$(top_builddir)/src/sensor/libsensor.a \
	$(AM_LDFLAGS)

test_mavhub_LDFLAGS = -lboost_unit_test_framework
