# MAVHUB (Micro Air Vehicle HUB)

MAVHUB is a simple C++ framework to abstract the communication between multiple
components of a robotic system. Therefor it supports multiple protocols and
has some wrapper classes to access I/O devices or to realize your robotic
algorithm.

## Dependencies
### Required
* automake
* pkg-config

### Optional
* autotools
* [BLAS](http://www.netlib.org/blas/) (libblas-dev)
* [boost C++ library](http://www.boost.org/) (libboost-dev)
* [boost Unit Test Framework](http://www.boost.org/doc/libs/1_55_0/libs/test/doc/html/index.html) (libboost-test-dev)
* [doxygen](http://www.doxygen.org)
* [GStreamer](http://gstreamer.freedesktop.org/)
* [ivy](http://www.eei.cena.fr/products/ivy/)
* [LAPACK](http://www.netlib.org/lapack/) (liblapack-dev)
* [mavlink](http://qgroundcontrol.org/mavlink/start)
* mkhuchlink
* OpenCV (libopencv-dev)
* OpenGL (freeglut3-dev)

## Configuration

### Additional libraries
* MAVLink:

		./configure CPPFLAGS="-I$(FULL_PATH_TO_MAVLINK)/include/huch"
* MKHUCHLink:

		./configure CPPFLAGS="-I$(FULL_PATH_TO_MKHUCHLINK)"

### Compiler flags
To enable optimization level 2 and to get rid of the debug symbols, enter 

	./configure CXXFLAGS="-O2"

### Cross compilation
To cross compile for an arm platform with the angstrom SDK run

	./configure --build=`config.guess` --host arm-angstrom-linux-gnueabi 

Two variables for pkg-config need to be set for proper operation. The paths
given below are for the older oe directory layout. In recent versions this
has to be changed to

	build/tmp-angstrom_2008_1/sysroots/armv7a-angstrom-linux-gnueabi

for e.g. the beagle-board.

	PKG_CONFIG_PATH=PATH_TO_OEROOT/tmp/staging/armv5te-angstrom-linux-gnueabi/usr/lib/pkgconfig
	PKG_CONFIG_SYSROOT_DIR=PATH_TO_OEROOT/tmp/staging/armv5te-angstrom-linux-gnueabi

### Unit Test Framework
To compile the test program in the tests directory, enable test support via

	./configure --enable-tests=yes

If you get an error on a 64 bit machine saying that configure could not find the version of the library,
adding of

	--with-boost-libdir=/usr/lib/x86_64-linux-gnu/

might help.

### Examples
1. On x86 architecture you might want to enter

		mkdir build-i686
		cd build-i686
		../configure CXXFLAGS="-O2" \
			CPPFLAGS="-I/home/user/git/mavlink/include/huch \
			-I/home/user/git/mkhuchlink"

2. For a typical beagleboard or gumstix configuration run

		mkdir build-arm
		cd build-arm
		../configure --build=`../auxdir/config.guess` \
			--host arm-angstrom-linux-gnueabi \
			CXXFLAGS="-O2 \
			CPPFLAGS="-I/home/user/git/mavlink/include/huch \
			-I/home/user/git/mkhuchlink"

## Compilation
After configuration, just type

	make

and you will find the binary under src/mavhub.

### Documentation
Documentation can be generated using Doxygen. Please run

	make doxygen-doc

and have a look at doc/html/index.html.

## Usage
	mavhub [options]

### Options
	-c <file>, --config <file>    open config file <file>
	-h, --help                    print usage summary

### Settings
Settings can be stored in an ini-like file. Examples can be found in the `mavhub.d` directory.
To start, just enter

	./src/mavhub -c ../mavhub.d/mavhub.conf

### GStreamer plugins
To use own gstreamer plugins or plugins which aren't installed in the GStreamer search path, you
have to set the `GST_PLUGIN_PATH` to the directory containing the plugins, e.g.

	export GST_PLUGIN_PATH=/home/user/git/mavhub/lib/gstreamer/
	./src/mavhub -c ../mavhub.d/mavhub.conf

### Unit Test Framework
A complete list about the supported paremeters can be found in the [boost runtime parameter reference](http://www.boost.org/doc/libs/1_55_0/libs/test/doc/html/utf/user-guide/runtime-config/reference.html).
For example, a more verbatim output can be reached by

	./tests/test_mavhub --log_level=message

To run only a subset of tests use the `run_test` argument

	./tests/test_mavhub --log_level=test_suite --run_test=hub_camera_tests,hub_slam_tests/tracker_test

## FAQ
## Where to get the sourcecode?
	git clone https://github.com/calihem/mavhub.git

## How can I manually rebuild the autotools files?
	aclocal -I m4
	automake
	autoconf

