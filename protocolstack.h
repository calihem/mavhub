#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "protocollayer.h"
#include <list>

namespace mavhub {

	typedef std::list<MediaLayer*> interface_list_t;

	class ProtocolStack : public PThread {
		public:
			ProtocolStack(uint8_t system_id);
			~ProtocolStack();

			void addInterface(MediaLayer *interface);
// 			void addLayer(ProtocolEntity *layer);
// // 			void addApplication(ApplicationLayer &app, int port);
// 			void join();
// 			int localhost() const;

		protected:
			virtual void run();


		private:
			uint8_t system_id;
			interface_list_t interface_list;

	};
	// ----------------------------------------------------------------------------
	// ProtocolStack
	// ----------------------------------------------------------------------------
// 	inline int ProtocolStack::localhost() const { return hostID; }


} // namespace mavhub

#endif
