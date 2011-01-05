ACLOCAL_AMFLAGS = -I m4

# EXTRA_DIST = 

if COND_UNIT_TESTS
MAYBE_TESTS = tests
else
MABE_TESTS =
endif

SUBDIRS = src $(MAYBE_TESTS)

include $(top_srcdir)/am/am_incl_doxygen.am
