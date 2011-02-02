#ifndef _IO_H_
#define _IO_H_

#include <string>
#include <fcntl.h>
#include <fstream>

#include <stdexcept>

namespace cpp_io {

class IOInterface {
	public:
		/// Get the human readable name of file, e.g. "Serial Port"
		const std::string& description() const;
		/// Enable/ disable blocking mode
		int enable_blocking_mode(bool enabled);
		/// Removes the path from filename
		static const std::string filename(const std::string& fullname);
		/// Returns file descriptor
		int handle() const;
		/// Get the system name of file (with path), e.g. /dev/ttyS0
		const std::string& name() const;
		/// Set (full) name of file
		void name(const std::string& name);
		/// Extracts the path from filename
		static const std::string path(const std::string& fullname);
		virtual bool is_open() const;
		virtual int open() = 0;
		virtual void close();
		/// Reads maximum nbyte from file to buf
		virtual ssize_t read(void *buf, size_t nbyte) const;
		/// Writes nbyte from file to buf
		virtual ssize_t write(const void *buf, size_t nbyte) const;
		friend std::ostream& operator <<(std::ostream &os, const IOInterface &file);

	protected:
		IOInterface(const std::string& name, const std::string& description);
		virtual ~IOInterface();

		/// file descriptor
		int fd;
		/// (full) file name in system context
		std::string _name;
		/// human readable file name
		std::string _description;
		
		virtual void print(std::ostream &os) const;

	private:
		IOInterface();
		IOInterface(const IOInterface &);
		void operator=(const IOInterface &);
};

class FStream : public std::fstream {
	public:
		FStream(const std::string& filename,
			std::ios_base::openmode mode = std::ios_base::in|std::ios_base::out)
			throw(const std::invalid_argument&);
		virtual ~FStream();

		const std::string& full_name() const;
		const std::string name() const;
		const std::string path() const;
		std::ios_base::openmode omode() const;

	private:
		/// full name of file
		std::string file_name;
		/// open mode of file
		std::ios_base::openmode open_mode;

		FStream(const FStream &);
		void operator=(const FStream &);
};

// ----------------------------------------------------------------------------
// IOInterface
// ----------------------------------------------------------------------------
inline IOInterface::IOInterface(const std::string& name, const std::string& description):
		fd(-1), _name(name), _description(description) {
}

inline IOInterface::~IOInterface() {
	close();
}

inline const std::string& IOInterface::description() const {
	return _description;
}

inline int IOInterface::enable_blocking_mode(bool enabled) {
	int mode = fcntl(fd, F_GETFL, 0);
	
	if(enabled) {
		mode &= ~O_NONBLOCK;
	} else {
		mode |= O_NONBLOCK;
	}
	return fcntl(fd, F_SETFL, mode);
}

inline int IOInterface::handle() const {
	return fd;
}

inline const std::string& IOInterface::name() const {
	return _name;
}

inline void IOInterface::print(std::ostream &os) const {
	os << _description << ": " << _name << std::endl;
}

inline bool IOInterface::is_open() const {
	return (fd >= 0);
}

inline void IOInterface::close() {
	if( is_open() ) {
		::close(fd);
		fd = -1;
	}
}

inline ssize_t IOInterface::read(void *buf, size_t nbyte) const {
	return ::read(fd, buf, nbyte);
}

inline ssize_t IOInterface::write(const void *buf, size_t nbyte) const {
	return ::write(fd, buf, nbyte);
}

inline std::ostream& operator <<(std::ostream &os, const IOInterface &file) {
	file.print(os);
	return os;
}

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

inline std::ios_base::openmode FStream::omode() const {
	return open_mode;
}

} // namespace cpp_io

#endif
