include $(top_srcdir)/am/am_incl_common.am

noinst_LIBRARIES =

if COND_USE_GSTREAMER
AM_CPPFLAGS += $(GSTREAMER_CFLAGS) -Wno-variadic-macros -Wno-long-long

noinst_LIBRARIES += libgstreamer.a

libgstreamer_a_SOURCES = \
	gstreamer/video_server.cpp
endif

if COND_OPENGL
noinst_LIBRARIES += libopengl.a

libopengl_a_SOURCES = \
	opengl/map_2d.cpp
endif

if COND_OPENCV2
noinst_LIBRARIES += libslam.a

libslam_a_SOURCES = \
	slam/features.cpp \
	slam/bundleadjust.cpp \
	slam/tracker.cpp \
	slam/map.cpp

endif
