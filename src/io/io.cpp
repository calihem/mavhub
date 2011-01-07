#include "io.h"

namespace cpp_io {

using namespace std;

// ----------------------------------------------------------------------------
// IOInterface
// ----------------------------------------------------------------------------
const std::string IOInterface::filename(const std::string& fullname) {
	size_t found = fullname.find_last_of("/\\");
	return fullname.substr(found+1);
}

const std::string IOInterface::path(const std::string& fullname) {
	size_t found = fullname.find_last_of("/\\");
	if(found == std::string::npos)
		return ".";
	return fullname.substr(0, found);
}

void IOInterface::name(const std::string& name) {
	bool was_open = is_open();
	if(is_open()) {
		close();
	}
	_name = name;

	if(was_open)
		open();
}

// ----------------------------------------------------------------------------
// FStream
// ----------------------------------------------------------------------------
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
