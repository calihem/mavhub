include $(top_srcdir)/am/am_incl_common.am

noinst_LIBRARIES =

noinst_LIBRARIES += libadept.a

libadept_a_SOURCES = \
	adept/adept.cpp

if COND_OPENCV2
# AM_CPPFLAGS += $(SIMD_FLAGS)
AM_CPPFLAGS += $(OPENCV2_CFLAGS)

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

if COND_LIBLAPACK
if COND_LIBBLAS

noinst_LIBRARIES += liblevmar.a libsba.a

liblevmar_a_SOURCES = \
	levmar/lm.c \
	levmar/Axb.c \
	levmar/misc.c \
	levmar/lmlec.c \
	levmar/lmbc.c \
	levmar/lmblec.c \
	levmar/lmbleic.c

libsba_a_SOURCES = \
	sba/sba_levmar.c \
	sba/sba_levmar_wrap.c \
	sba/sba_lapack.c \
	sba/sba_crsm.c \
	sba/sba_chkjac.c

endif
endif
