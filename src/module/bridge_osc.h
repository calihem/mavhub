#ifndef _BRIDGE_OSC_H_
#define _BRIDGE_OSC_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_LIBOSCPACK
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#include <inttypes.h> //uint8_t
#include "protocol/protocollayer.h"

#define OSC_OUTPUT_BUFFER_SIZE 1024

namespace mavhub {
	// packet listener class
	class MavhubOscPacketListener : public osc::OscPacketListener {
	public:
    virtual void ProcessMessage(const osc::ReceivedMessage& m, 
																const IpEndpointName& remoteEndpoint);
	};



	class Bridge_Osc : public AppLayer<mavlink_message_t> {
	public:
		Bridge_Osc(const std::map<std::string, std::string> args);
		virtual ~Bridge_Osc();
		virtual void handle_input(const mavlink_message_t &msg);
		virtual void send_mavlink_msg(mavlink_message_t* msg);

	protected:
		virtual void run();

	private:
			uint16_t component_id;
			uint16_t port;

			MavhubOscPacketListener* lp;
			UdpListeningReceiveSocket* sp;
			
			virtual void read_conf(const std::map<std::string, std::string> args);
	};
	
	// ----------------------------------------------------------------------------
	// Bridge_Osc
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif // HAVE_LIBOSCPACK
#endif // HAVE_MAVLINK_H
#endif
