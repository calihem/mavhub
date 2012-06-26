#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([mavhub], [0.1], [mavhub@calihem.de])

AC_LANG([C++])
AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_SRCDIR([src/main.cpp])
AC_CONFIG_AUX_DIR([auxdir])
AC_CONFIG_MACRO_DIR([m4])

AC_CANONICAL_SYSTEM
AM_INIT_AUTOMAKE([foreign])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_CPP
AC_PROG_RANLIB

# Checks for SSE support
AX_EXT

# doxygen support
DX_HTML_FEATURE(ON)
DX_CHM_FEATURE(OFF)
DX_CHI_FEATURE(OFF)
DX_MAN_FEATURE(OFF)
DX_RTF_FEATURE(OFF)
DX_XML_FEATURE(OFF)
DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)
DX_INIT_DOXYGEN([$PACKAGE_NAME], [doc/Doxyfile], [doc])

# unit tests support
AC_ARG_ENABLE(tests,
	AC_HELP_STRING([--enable-tests=yes|no],
		[enable automated tests (default is no)]),
	[case "${enableval}" in
		yes)
			tests=true
			AX_BOOST_BASE
			AX_BOOST_UNIT_TEST_FRAMEWORK
			;;
		no)
			tests=false
			;;
		*)
			AC_MSG_ERROR(bad value ${enableval} for --enable-tests)
			;;
		esac],
	[tests=false])
AM_CONDITIONAL(COND_UNIT_TESTS, test x$tests = xtrue)

# Checks for pkg-config
PKG_PROG_PKG_CONFIG([0.25])

# Checks for OpenCV
PKG_CHECK_MODULES([OPENCV1], [opencv >= 1.0 opencv < 2.0],
	[AC_DEFINE(HAVE_OPENCV1, [], [Use OpenCV1]) AM_CONDITIONAL(COND_OPENCV1, true) AC_DEFINE(HAVE_OPENCV, [], [Use OpenCV])],
	[AM_CONDITIONAL(COND_OPENCV1, false)])
AC_SUBST([OPENCV1_CFLAGS])
AC_SUBST([OPENCV1_LIBS])
PKG_CHECK_MODULES([OPENCV2], [opencv >= 2.0],
	[AC_DEFINE(HAVE_OPENCV2, [], [Use OpenCV2]) AM_CONDITIONAL(COND_OPENCV2, true) AC_DEFINE(HAVE_OPENCV, [], [Use OpenCV])],
	[AM_CONDITIONAL(COND_OPENCV2, false)])
AC_SUBST([OPENCV2_CFLAGS])
AC_SUBST([OPENCV2_LIBS])

# Checks for GStreamer
PKG_CHECK_MODULES([GSTREAMER], [gstreamer-0.10 gstreamer-app-0.10],
	[AC_DEFINE(HAVE_GSTREAMER, [], [Use GStreamer]) gstreamer=true],
	[gstreamer=false])
AM_CONDITIONAL(COND_USE_GSTREAMER, test x$gstreamer = xtrue)

# Checks for libraries.
# To list the function names of a library run:
# nm -D /lib/libfoo.so
AC_CHECK_LIB([ncurses], [newwin])
AC_CHECK_LIB([pthread], [pthread_mutex_lock])
AC_CHECK_LIB([gps], [gps_clear_fix])
AC_CHECK_LIB([glut], [glutInit])
AC_CHECK_LIB([GL], [glBegin])
AC_CHECK_LIB([GLU], [gluPerspective])
AC_CHECK_LIB([ivy], [IvyInit])
AC_CHECK_LIB([lapack],
	[cgesvd_],
	[AM_CONDITIONAL(COND_LIBLAPACK, true) LIBS+="-llapack " AC_DEFINE(HAVE_LIBLAPACK, [1], [Define to 1 if you have the `lapack' library (-llapack).])],
	AM_CONDITIONAL(COND_LIBLAPACK, false) )
AC_CHECK_LIB([blas],
	[cblas_caxpy],
	[AM_CONDITIONAL(COND_LIBBLAS, true) LIBS+="-lblas " AC_DEFINE(HAVE_LIBBLAS, [1], [Define to 1 if you have the `blas' library (-lblas).])],
	AM_CONDITIONAL(COND_LIBBLAS, false))
AC_CHECK_LIB([fann], [fann_create_standard])
AC_CHECK_LIB([oscpack], [InterruptSignalHandler]) #_ZN3osc20OutboundPacketStreamlsERKNS_12BeginMessageE])

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h fcntl.h inttypes.h netdb.h netinet/in.h strings.h sys/ioctl.h sys/socket.h sys/time.h termios.h unistd.h])
AC_CHECK_HEADERS([mavlink.h], , [AC_MSG_WARN( [Please specify header directory for mavlink, e.g. configure CPPFLAGS="-I/home/user/mavlink/include/huch"] )])
AC_CHECK_HEADERS([mkhuchlink.h], , [AC_MSG_WARN( [Please specify header directory for mkhuchlink, e.g. configure CPPFLAGS="-I/home/user/mkhuchlink"] )])
AC_CHECK_HEADERS([mklink.h], , [AC_MSG_WARN( [Please specify header directory for mklink, e.g. configure CPPFLAGS="-I/home/user/mklink"] )])
AC_CHECK_HEADERS([libgpsmm.h], , [AC_MSG_WARN([libgps not installed in system - needed for ctrl_wifimeter_app])])
AC_CHECK_HEADERS([GL/glut.h GL/freeglut.h], AM_CONDITIONAL(COND_OPENGL, true) , [AM_CONDITIONAL(COND_OPENGL, false) AC_MSG_WARN([OpenGL headers not found])])
AC_CHECK_HEADERS([Ivy/ivy.h], , [AC_MSG_WARN( [Please specify header directory for ivy, e.g. configure CPPFLAGS="-I/home/user/ivy-c"] )])
AC_CHECK_HEADERS([osc/OscTypes.h], , [AC_MSG_WARN([Please specify header directory for oscpack, e.g. configure CPPFLAGS="-I/usr/include/oscpack"])])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_UINT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_INT8_T
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([bzero gethostbyname gettimeofday inet_ntoa pow select socket strerror])

AC_CONFIG_FILES([Makefile
                 src/Makefile
                 src/application/Makefile
                 src/core/Makefile
                 src/io/Makefile
                 src/lib/Makefile
                 src/module/Makefile
                 src/protocol/Makefile
                 src/sensor/Makefile
                 thirdparty/Makefile
                 tests/Makefile])

AC_OUTPUT

echo "
  ($PACKAGE_NAME) version $PACKAGE_VERSION
  Prefix.........: $prefix
  C++ Compiler...: $CXX $CXXFLAGS $CPPFLAGS $OPENCV1_CFLAGS $OPENCV2_CFLAGS
  Linker.........: $LD $LDFLAGS $GSTREAMER_LIBS $OPENCV1_LIBS $OPENCV2_LIBS $LIBS
  Unit tests.....: $tests
"
