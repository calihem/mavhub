#ifndef _FILE_H_
#define _FILE_H_

#include <string>
#include <fstream>

#include <stdexcept>

namespace cpp_io {

class FStream : public std::fstream {
	public:
		FStream(const std::string& filename,
			std::ios_base::openmode mode = std::ios_base::in|std::ios_base::out)
			throw(const std::invalid_argument&);
		~FStream();

		const std::string& full_name() const;
		const std::string name() const;
		const std::string path() const;
		const std::ios_base::openmode omode() const;

	private:
		/// full name of file
		std::string file_name;
		/// open mode of file
		std::ios_base::openmode open_mode;

		FStream(const FStream &);
		void operator=(const FStream &);
};

// ----------------------------------------------------------------------------
// FStream
// ----------------------------------------------------------------------------
inline const std::string& FStream::full_name() const {
	return file_name;
}

inline const std::string FStream::name() const { //FIXME: do and use static function
	size_t found = file_name.find_last_of("/\\");
	return file_name.substr(found+1);
}

inline const std::string FStream::path() const { //FIXME: do and use static function
	size_t found = file_name.find_last_of("/\\");
	if(found == std::string::npos)
		return ".";
	return file_name.substr(0, found);
}

inline const std::ios_base::openmode FStream::omode() const {
	return open_mode;
}

} // namespace cpp_io

#endif
