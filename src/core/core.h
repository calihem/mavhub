#ifndef _MAVHUB_CORE_H_
#define _MAVHUB_CORE_H_

#include <inttypes.h> // uint16_t
#include "core/thread.h"

namespace mavhub {

class Core {
	public:
		/// Get system ID
		static const uint16_t system_id();
		/// Set system ID
		static const uint16_t system_id(const uint16_t system_id);
		/// Pointer to number of arguments given to main routine
		static int *argc;
		/// Argument vector given to main routine
		static char** argv;

	private:
		static uint16_t _system_id; ///< \var System ID
		static pthread_mutex_t system_id_mutex; ///< \var Mutex to protect _system_id
};

// ----------------------------------------------------------------------------
// Core
// ----------------------------------------------------------------------------
inline const uint16_t Core::system_id() {
	return _system_id;
}
inline const uint16_t Core::system_id(const uint16_t system_id) {
	cpp_pthread::Lock sys_lock(system_id_mutex);
	_system_id = system_id;

	return _system_id;
}


} // namespace mavhub

#endif
