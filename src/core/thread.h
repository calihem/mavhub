/****************************************************************************
** Copyright 2011 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file thread.h
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Wrapper for pthread library.
 */

#ifndef _THREAD_H_
#define _THREAD_H_

#include <pthread.h>

namespace cpp_pthread {

/**
 * \brief C++ wrapper class for the POSIX pthread library
 *
 * PThread is an abstract class to use the POSIX pthread
 * library in an object oriented way. A simple thread
 * class could look like the following:
 * \code
 * class MyThread : public PThread {
 *     public:
 *         MyThread() { run(); };
 *
 *     protected:
 *         virtual void run() {
 *             while( !interrupted() ) {
 *                 // do something
 *             }
 *         };
 * };
 * \endcode
 */
class PThread {
	public:
		 /// Constructor.
		PThread() : _running(false), _interrupt(false) { };
		 /// Destructor.
		~PThread() { };
		/**
		 * \brief Creates a new thread and starts the execution.
		 */
		const pthread_t& start();
		/**
		 * \brief Wait for thread termination.
		 * Suspends execution of calling thread until target thread
		 * terminates.
		 *
		 * \return Pointer to return value passed to pthread_exit
		 */
		void* join();
		/**
		 * \brief Interrupt current thread.
		 * Sets an interrupt flag for the current thread to terminate
		 * execution. 
		 *
		 * \warning This will only work if the implemenation of run()
		 * checks the flag by calling interrupted().
		 * \sa join() 
		 */
		void interrupt();
		/**
		 * \brief True if thread has to be interrupted or was interrupted.
		 */
		const bool interrupted() const;

	protected:
		/// C struct to represent thread.
		pthread_t thread;
		/**
		 * \brief Working method of the thread.
		 */
		virtual void run() = 0;
		/**
		 * \brief Static wrapper function to call pthread_create.
		 */
		static void* start_routine_wrapper(void*);

	private:
		/// True if current thread is running.
		bool _running;
		/// True if thread has to be interrupted or was interrupted.
		bool _interrupt;
};

/**
 * \brief RAII-class for mutexes
 *
 * The Lock class will automatically lock the given mutex in the constructor
 * and unlock it in the destructor. If you put your variables on the stack,
 * the destructor will be called at the end of the scope and the risk to run 
 * into a deadlock is minimized.
 * \code
 * pthread_mutex_t my_mutex;
 * {
 *     Lock my_lock(my_mutex); // locks my_mutex
 *     // put locked code here
 * }
 * // my_mutex is unlocked again
 * \endcode
 */
class Lock {
	public:
		explicit Lock(pthread_mutex_t &mutex);
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

	_interrupt = false;
	pthread_create( &thread, NULL, start_routine_wrapper, static_cast<void*>(this) );
	_running = true;
	return thread;
}
inline void* PThread::join() {
	if(!_running) return NULL;

	void *rc;
	interrupt();
	pthread_join(thread, &rc);
	_running = false;
	return rc;
}
inline void PThread::interrupt() {
	_interrupt = true;
}
inline const bool PThread::interrupted() const {
	return _interrupt;
}
inline void* PThread::start_routine_wrapper(void *arg) {
	PThread *ptr = static_cast<PThread*>(arg);
	if(ptr) ptr->run();
	return 0;
}

// ----------------------------------------------------------------------------
// Lock
// ----------------------------------------------------------------------------
inline Lock::Lock(pthread_mutex_t &mutex) : mutex_ptr(&mutex) {
	pthread_mutex_lock(mutex_ptr);
}

inline Lock::~Lock() {
	pthread_mutex_unlock(mutex_ptr);
}

} // namespace cpp_pthread

#endif
