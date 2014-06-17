include $(top_srcdir)/am/am_incl_common.am

# additional include paths necessary to compile the C++ program
# AM_CXXFLAGS = 

bin_PROGRAMS = test_mavhub
test_mavhub_SOURCES =  \
	brisk_test.cpp \
	camera_test.cpp \
	condition_test.cpp \
	feature_test.cpp \
	math_test.cpp \
	mavhub_test.cpp \
	pose_test.cpp \
	sba_test.cpp \
	slam_test.cpp \
	time_test.cpp \
	application/acc_calibration_app_test.cpp

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

test_mavhub_LDADD = \
	$(top_builddir)/src/application/libapp.a \
	$(top_builddir)/src/core/libcore.a \
	$(top_builddir)/src/io/libio.a \
	$(top_builddir)/src/module/libmodule.a \
	$(top_builddir)/src/protocol/libprotocol.a \
	$(top_builddir)/src/sensor/libsensor.a \
	$(SRC_EXTRA_LIBS) \
	$(THIRDPARTY_EXTRA_LIBS) \
	$(OPENCV1_LIBS) \
	$(OPENCV2_LIBS) \
	$(AM_LDFLAGS)

test_mavhub_LDFLAGS = -lboost_unit_test_framework
