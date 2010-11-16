#include "file.h"

namespace cpp_io {

using namespace std;

FStream::FStream(const std::string& filename, std::ios_base::openmode mode) throw(const std::invalid_argument&) :
		fstream(filename.c_str(), mode),
		file_name(filename),
		open_mode(mode) {

	if ( !(*this) ) {
		throw invalid_argument( string("File ") + file_name + string(" doesn't exist") );
	}
}

FStream::~FStream() {
}

} //namespace cpp_io
