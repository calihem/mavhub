#include "bridge_osc.h"

#ifdef HAVE_LIBOSCPACK

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
#include <stdlib.h>

#define OSC_OUTPUT_BUFFER_SIZE 1024

using namespace std;

namespace mavhub {

	void MavhubOscPacketListener::ProcessMessage(const osc::ReceivedMessage& m,
																							 const IpEndpointName& remoteEndpoint)
	{
		try{
			// example of parsing single messages. osc::OsckPacketListener
			// handles the bundle traversal.
            
			if( strcmp( m.AddressPattern(), "/test1" ) == 0 ){
				// example #1 -- argument stream interface
				osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
				bool a1;
				osc::int32 a2;
				float a3;
				const char *a4;
				args >> a1 >> a2 >> a3 >> a4 >> osc::EndMessage;
                
				std::cout << "received '/test1' message with arguments: "
									<< a1 << " " << a2 << " " << a3 << " " << a4 << "\n";
                
			}
			else if( strcmp( m.AddressPattern(), "/test2" ) == 0 ){
				// example #2 -- argument iterator interface, supports
				// reflection for overloaded messages (eg you can call 
				// (*arg)->IsBool() to check if a bool was passed etc).
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				bool a1 = (arg++)->AsBool();
				int a2 = (arg++)->AsInt32();
				float a3 = (arg++)->AsFloat();
				const char *a4 = (arg++)->AsString();
				if( arg != m.ArgumentsEnd() )
					throw osc::ExcessArgumentException();
                
				std::cout << "received '/test2' message with arguments: "
									<< a1 << " " << a2 << " " << a3 << " " << a4 << "\n";
			}
			else if( strcmp( m.AddressPattern(), "/test3" ) == 0 ){
				// example #1 -- argument stream interface
				osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
				osc::int32 a1;
				args >> a1 >> osc::EndMessage;
                
				Logger::log("received '/test3' message with arguments: ", a1, Logger::LOGLEVEL_DEBUG);
                
			}
		}
		catch( osc::Exception& e ){
			// any parsing errors such as unexpected argument types, or 
			// missing arguments get thrown as exceptions.
			std::cout << "error while parsing message: "
                << m.AddressPattern() << ": " << e.what() << "\n";
		}
	}


	Bridge_Osc::Bridge_Osc(const map<string, string> args) : AppInterface("bridge_osc"), AppLayer("bridge_osc") {
		//, phi(0.0), theta(0.0), psi(0.0) 
		// initialize mod parameters from conf
		// read_conf(args);
		assign_variable_from_args(component_id);
		assign_variable_from_args(port);
		/* initializations */
		lp = new MavhubOscPacketListener;
		sp = new UdpListeningReceiveSocket(
																			 IpEndpointName( IpEndpointName::ANY_ADDRESS, port),
																			 lp );
	}

	Bridge_Osc::~Bridge_Osc() {}

	void Bridge_Osc::send_mavlink_msg(mavlink_message_t* msg) {
		printf("callbackcallback: %i\n", msg->sysid);
		AppLayer<mavlink_message_t>::send(*msg);
	}

	void Bridge_Osc::handle_input(const mavlink_message_t &msg) {
		mavlink_huch_sensor_array_t sa;

    char buffer[OSC_OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OSC_OUTPUT_BUFFER_SIZE);
    
    // p << osc::BeginBundleImmediate
    //     << osc::BeginMessage( "/test1" ) 
    //         << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
    //     << osc::BeginMessage( "/test2" ) 
    //         << true << 24 << (float)10.8 << "world" << osc::EndMessage
    //     << osc::EndBundle;
    
    // transmitSocket.Send( p.Data(), p.Size() );

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			// Logger::log("Bridge_Osc got mavlink heartbeat: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_DEBUG);
			//OscSendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", msg.sysid);
			// OscSendMsg("%d ALIVE 50,162,250,192,221,27,111,57,63,249,122,206,139,11,197,244", msg.sysid);
			p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/heartbeat" ) 
				<< static_cast<int>(msg.sysid) << osc::EndMessage
				<< osc::EndBundle;

			sp->SendTo(
								 IpEndpointName( "127.0.0.1", 7002),
								 p.Data(),
								 p.Size()
								 );
			
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			// phi = mavlink_msg_attitude_get_roll(&msg);
			// theta = mavlink_msg_attitude_get_pitch(&msg);
			// psi = mavlink_msg_attitude_get_yaw(&msg);
			// //Logger::log("Bridge_Osc got ml attitude", phi, theta, Logger::LOGLEVEL_INFO);
			//OscSendMsg("%d ATTITUDE %f %f %f", 155, phi, theta, psi);
			//OscSendMsg("%d ATTITUDE %f %f %f", msg.sysid, 0., 0., 0.);
			break;
		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			mavlink_msg_huch_sensor_array_decode(&msg, &sa);
			//OscSendMsg("%d PRESSURE 0.0 0.0 %f %f", msg.sysid, sa.data[0], sa.data[1]);
			// OscSendMsg("%d HUCH_SENSOR_RAW %f %f %f %f %f %f", msg.sysid,
			// 					 sa.data[0], sa.data[1], sa.data[2], sa.data[3],
			// 					 sa.data[4], sa.data[5]
			// 					 );
			break;
		case MAVLINK_MSG_ID_PARAM_VALUE:
			Logger::log("Bridge_Osc got ml param value: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			// static int8_t param_id[15];
			// float param_value;
			// int setting_id;
			// mavlink_msg_param_value_get_param_id(&msg, param_id);
			// param_value = mavlink_msg_param_value_get_param_value(&msg);
			// Logger::log("Bridge_Osc got ml param value: (param_id, param_value)", (char*)param_id, param_value, Logger::LOGLEVEL_INFO);
			// if(!strcmp((char*)param_id, (const char*)"ctl_mode")) {
			// 	setting_id = 38;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"ctl_bump")) {
			// 	setting_id = 39;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"bump_thr_low")) {
			// 	setting_id = 40;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"bump_thr_high")) {
			// 	setting_id = 41;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"ac_pid_bias")) {
			// 	setting_id = 42;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"ac_pid_Kc")) {
			// 	setting_id = 43;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"ac_pid_Ki")) {
			// 	setting_id = 44;
			// }
			// else if(!strcmp((char*)param_id, (const char*)"ac_pid_Kd")) {
			// 	setting_id = 45;
			// }
			// else
			// 	setting_id = -1;
			// if(setting_id >= 0) {
			// OscSendMsg("%d DL_VALUE %d %f",
			// 					 system_id(),
			// 					 setting_id,
			// 					 param_value);
			// }
			// // 					 , 38, 2.0);
			// // OscSendMsg("%d DL_VALUE %d %f", 42, 39, 0.0);
			break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			// Logger::log("Bridge_Osc got mavlink local position: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/local_position" ) 
				<< static_cast<int>(msg.sysid)
				<< mavlink_msg_local_position_ned_get_x(&msg)
				<< mavlink_msg_local_position_ned_get_y(&msg)
				<< mavlink_msg_local_position_ned_get_z(&msg)
				<< osc::EndMessage
				<< osc::EndBundle;

			sp->SendTo(
								 IpEndpointName( "127.0.0.1", 7002),
								 p.Data(),
								 p.Size()
								 );
			
			break;

#ifdef MAVLINK_ENABLED_HUCH
		case MAVLINK_MSG_ID_HUCH_VISUAL_FLOW:
			p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/huch_visual_flow" ) 
				<< static_cast<int>(msg.sysid)
				<< mavlink_msg_huch_visual_flow_get_u(&msg)
				<< mavlink_msg_huch_visual_flow_get_v(&msg)
				<< mavlink_msg_huch_visual_flow_get_u_i(&msg)
				<< mavlink_msg_huch_visual_flow_get_v_i(&msg)
				<< osc::EndMessage
				<< osc::EndBundle;

			sp->SendTo(
								 IpEndpointName( "127.0.0.1", 7002),
								 p.Data(),
								 p.Size()
								 );
			
			break;
		case MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL:
			p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/huch_generic_channel" ) 
				<< static_cast<int>(msg.sysid)
				<< mavlink_msg_huch_generic_channel_get_index(&msg)
				<< mavlink_msg_huch_generic_channel_get_value(&msg)
				<< osc::EndMessage
				<< osc::EndBundle;

			sp->SendTo(
								 IpEndpointName( "127.0.0.1", 7002),
								 p.Data(),
								 p.Size()
								 );
			
			break;
#endif

		default:
			break;
		}
		// AppLayer<mavlink_message_t>::send(msg);
	}

	void Bridge_Osc::run() {
		if(!owner()) {
			Logger::log("Owner of Bridge_Osc not set", Logger::LOGLEVEL_WARN);
			return;
		}

		// init oscpack
    // MavhubOscPacketListener listener;
    // UdpListeningReceiveSocket s(
    //         IpEndpointName( IpEndpointName::ANY_ADDRESS, port),
    //         &listener );

		//int system_type = MAV_QUADROTOR;
		// mavlink_message_t msg_heartbeat;
		// mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_heartbeat, system_type, MAV_AUTOPILOT_HUCH);

		Logger::log("Bridge_Osc started", Logger::LOGLEVEL_INFO);

		// // how to add before/after select hook
		// pt2Function = &afterSelect_cb;
		// // OscSetBeforeSelectHook(pt2Function, (void*)0);
		// OscSetAfterSelectHook(pt2Function, (void*)0);

		// with local member function, defunct
		// pt2Member = &mavhub::Bridge_Osc::afterSelect_cb;
		// OscSetBeforeSelectHook(pt2Member, (void*)0);


		// int delay = 1000;
		// tid = TimerRepeatAfter (0, delay, handle_timer, 0);
		// tid = TimerRepeatAfter (0, delay, cb_DlValue, this);

		// libosc mainloop
		// OscMainLoop();
		// s.RunUntilSigInt();
		// s.Run();
		sp->Run();

		// local mainloop
		// while(1) {
		// 	Logger::log("bridge_osc: system_id", static_cast<int>(system_id()), Logger::LOGLEVEL_INFO);
		// 	//AppLayer<mavlink_message_t>::send(msg_heartbeat);
		// 	//send(msg_heartbeat);
		// 	//OscSendMsg("%d MAVHUB_ALIVE %f", 155, 1.0);
		// 	sleep(1);
		// }
	}

	void Bridge_Osc::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Bridge_Osc::read_conf", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		Logger::log("Bridge_Osc::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
	}

	// void Bridge_Osc::handle_timer (TimerId id, void *data, unsigned long delta) {
	// 	//if (wakeup(3) && send_loop()) decode_and_send_to_osc();
	// 	//if (wakeup(3))
	// 	//do_something();
	// 	printf("handle_timer\n");
	// 	OscSendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 155);
	// }

	// void Bridge_Osc::afterSelect_cb(void* arg) {
	// 	//OscHookPtr* a;
	// 	//std::cout << arg->a ", " << arg->b << std::endl;
	// 	//printf("%d, %f\n", arg->a, arg->b);
	// 	printf("Bridge_Osc::afterSelect_cb: 1,2,3\n");
	// 	//return a;
	// 	//return 0;
	// }

} // namespace mavhub

#endif // HAVE_LIBOSCPACK
