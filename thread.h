#ifndef _THREAD_H_
#define _THREAD_H_

#include <pthread.h>

namespace mavhub {
	
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

} // namespace mavhub

#endif
