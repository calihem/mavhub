#ifndef _THREAD_H_
#define _THREAD_H_

#include <pthread.h>

namespace cpp_pthread {
	
/**  
* C++ wrapper class for the POSIX pthread library
*/
class PThread {
	public:
		PThread() {};
		~PThread() {};
		pthread_t start();
		static void *join(pthread_t &pthread);

	protected:
		virtual void run() = 0;
		 static void *start_routine_wrapper(void *);
};

/**
 * RAII-class for mutexes
 */
class Lock {
	public:
		explicit Lock(pthread_mutex_t& mutex);
		~Lock();

	private:
		Lock(const Lock&);
		Lock& operator=(const Lock&);
		pthread_mutex_t *mutex_ptr;
};

// ----------------------------------------------------------------------------
// PThread
// ----------------------------------------------------------------------------
inline pthread_t PThread::start() {
	pthread_t thread;
	pthread_create(&thread, NULL, start_routine_wrapper, (void *)(this));
	return thread;
}
inline void *PThread::join(pthread_t &thread) {
	void *rc;
	pthread_join(thread, &rc);
	return rc;
}
inline void *PThread::start_routine_wrapper(void *arg) {
	PThread *ptr = (PThread *)arg;
	ptr->run();
	return 0;
}

// ----------------------------------------------------------------------------
// Lock
// ----------------------------------------------------------------------------
inline Lock::Lock(pthread_mutex_t& mutex) : mutex_ptr(&mutex) {
	pthread_mutex_lock(mutex_ptr);
}

inline Lock::~Lock() {
	pthread_mutex_unlock(mutex_ptr);
}

} // namespace cpp_pthread

#endif
