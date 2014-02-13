#include "map.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

namespace hub {
namespace slam {

const char *ply_header_start = "ply\nformat ascii 1.0\n";
const char *ply_vertex_element = "element vertex %u\n"
	"property float32 x\n"
	"property float32 y\n"
	"property float32 z\n"
	"property uint8 red\n"
	"property uint8 green\n"
	"property uint8 blue\n";
const char *ply_face_element = "element face %u\n"
	"property list uint8 int32 vertex_index\n";
const char *ply_header_end = "end_header\n";


} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
