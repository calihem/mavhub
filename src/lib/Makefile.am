include $(top_srcdir)/am/am_incl_common.am

if COND_USE_GSTREAMER
AM_CPPFLAGS += $(GSTREAMER_CFLAGS) -Wno-variadic-macros -Wno-long-long

noinst_LIBRARIES = libgstreamer.a

libgstreamer_a_SOURCES = \
	gstreamer/video_server.cpp
endif

if COND_OPENGL
noinst_LIBRARIES = libopengl.a

libopengl_a_SOURCES = \
	opengl/map_2d.cpp
endif
