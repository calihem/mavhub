#ifndef _THREAD_H_
#define _THREAD_H_

#include <pthread.h>

namespace cpp_pthread {
	
/**  
* C++ wrapper class for the POSIX pthread library
*/
class PThread {
	public:
		PThread() : _running(false) {};
		~PThread() {};
		const pthread_t& start();
		void *join();

	protected:
		pthread_t thread;
		bool _running;

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
inline const pthread_t& PThread::start() {
	if(_running) return thread;

	pthread_create(&thread, NULL, start_routine_wrapper, (void *)(this));
	_running = true;
	return thread;
}
inline void *PThread::join() {
	if(!_running) return NULL;

	_running = false;
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
