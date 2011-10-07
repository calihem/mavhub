include $(top_srcdir)/am/am_incl_common.am

noinst_LIBRARIES =

if COND_OPENCV
# AM_CPPFLAGS += $(SIMD_FLAGS)

noinst_LIBRARIES += libbrisk.a

libbrisk_a_SOURCES = \
	brisk/brisk.cpp
endif
