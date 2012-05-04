include $(top_srcdir)/am/am_incl_common.am

# additional include paths necessary to compile the C++ library
AM_CXXFLAGS += $(GSTREAMER_CFLAGS)

noinst_LIBRARIES = libapp.a

libapp_a_SOURCES = \
	acc_calibration_app/acc_calibration_app.cpp \
	attitude_filter_app/attitude_filter_app.cpp \
	app_store.cpp \
	core_app.cpp \
	mavlink_mk_app.cpp \
	mavlink_mkhuch_app.cpp \
	mk_app.cpp \
	opengl_app.cpp \
	slam_app.cpp \
	vision/v_oflow_app.cpp \
	vision/ofdifferential.cpp \
	vision/OpticalFlow.cpp \
	vision/ofmodel.cpp
