#include "pose.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <iomanip>	//setprecision
#include <limits>	//epsilon


#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout
#else
#define dout if(0) std::cout
#endif

namespace hub {
namespace slam {

using namespace std;
using namespace TooN;

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
