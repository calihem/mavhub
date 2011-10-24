include $(top_srcdir)/am/am_incl_common.am

noinst_LIBRARIES =

if COND_OPENCV
# AM_CPPFLAGS += $(SIMD_FLAGS)

noinst_LIBRARIES += libagast.a

libagast_a_SOURCES = \
	agast/agast5_8.cc \
	agast/agast5_8_nms.cc \
	agast/agast7_12d.cc \
	agast/agast7_12d_nms.cc \
	agast/agast7_12s.cc \
	agast/agast7_12s_nms.cc \
	agast/nonMaximumSuppression.cc \
	agast/oast9_16.cc \
	agast/oast9_16_nms.cc

noinst_LIBRARIES += libbrisk.a

libbrisk_a_SOURCES = \
	brisk/brisk.cpp
endif
