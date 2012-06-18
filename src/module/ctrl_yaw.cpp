// control yaw component

#include "ctrl_yaw.h"

#ifdef HAVE_MAVLINK_H

#include "core/datacenter.h"
#include "protocol/protocolstack.h"

#include <cstdlib>
#include <sstream>

#include <math.h>

#define PI 3.1415926535897931
#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Yaw::Ctrl_Yaw(const map<string, string> args,
										 const Logger::log_level_t loglevel) :
		AppInterface("ctrl_yaw", loglevel),
		ModuleBase(args, "ctrl_yaw"),
		sensor_type(1)
	{
		read_conf(args);
		param_request_list = 0;
		// pid_yaw = new PID(0, params["yaw_Kc"], params["yaw_Ti"],
		// 									params["yaw_Td"]);
	}

	Ctrl_Yaw::~Ctrl_Yaw() {
	}

  void Ctrl_Yaw::handle_input(const mavlink_message_t &msg) {
		static char param_id[16];
		int rc2;
		int rc5;
		//Logger::log("Ctrl_Yaw got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {

#ifdef MAVLINK_ENABLED_HUCH

		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			// Logger::log("Ctrl_Yaw: got visual_navigation msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_navigation_decode(&msg, (mavlink_huch_visual_navigation_t *)&huch_visual_navigation);
			break;

		case MAVLINK_MSG_ID_HUCH_VISUAL_FLOW:
			//Logger::log("Ctrl_Yaw: got visual_flow msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_flow_decode(&msg, (mavlink_huch_visual_flow_t *)&huch_visual_flow);
			//Logger::log("psi_est:", huch_visual_flow.psi_estimate, Logger::LOGLEVEL_INFO);
			break;

		// case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
		// 	// FIXME: implement this instead of datacenter
		// 	break;

#endif // MAVLINK_ENABLED_HUCH	

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Yaw::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			// FIXME: make this a function
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Yaw::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Yaw::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Yaw::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Yaw::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

					// // update PID controllers
					// pid_pitch->setKc(params["yaw_Kc"]);
					// pid_pitch->setBias(params["yaw_bias"]);

				}
			}
			break;

		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			rc2 = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
			rc5 = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
			if (rc2 > 1700 && rc5 > 1700) 
				params["reset_sp"] = 2.0;
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			// Logger::log(name(), "rx msg attitude", Logger::LOGLEVEL_DEBUG);
			mavlink_msg_attitude_decode(&msg, &attitude);
			break;

		default:
			break;
		}		
	}

  void Ctrl_Yaw::run() {
		// generic
		static mavlink_message_t msg;
		//static mavlink_debug_t dbg;
		// timing
		// uint64_t dt = 0;
		// double dtf = 0.0;
		struct timeval tk, tkm1; // timevals
		int update_rate = 60; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		// body variables
		int yaw_meas;
		int sp;
		int compass_res, compass_res_half, compass_res_three_half;
		double yaw, yaw1;
		// vector<int16_t> v(3);

		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);

		yaw = yaw1 = 0;
		sp = 0;
		yaw_meas = 0;
		compass_res = 256;
		compass_res_half = 128;
		compass_res_three_half = 384;

		Logger::log("Ctrl_Yaw started:", name(), Logger::LOGLEVEL_INFO);
		while(true) {
			/* wait time */
			usec = get_time_us();
			uint64_t end = usec;
			wait_time = wait_freq - (end - start);
			wait_time = (wait_time < 0)? 0: wait_time;
		
			/* wait */
			usleep(wait_time);
			//usleep(10);

			/* calculate frequency */
			end = get_time_us();
			frequency = (15 * frequency + end - start) / 16;
			start = end;

			// Logger::log("Ctrl_Yaw slept for", wait_time, component_id, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			// dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time
			// dt fractional in seconds
			// dtf = (double)dt * 1e-6;

			// param handling
			if(param_request_list) {
				Logger::log("Ctrl_Yaw::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const char*) p->first.data(), MAVLINK_TYPE_FLOAT, p->second, 1, 0);
					send(msg);
				}

				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_test_pitch", prm_test_pitch, 1, 0);
				// send(msg);
				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_yaw_P", prm_yaw_P, 1, 0);
				// send(msg);
			}

			if(params["reset_sp"] > 0.0) {
				params["reset_sp"] = 0.0;
				sp = yaw_meas;
			}

			// // test huch_visual_navigation
			// huch_visual_navigation.psi_estimate = 1.234;
			// mavlink_msg_huch_visual_navigation_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &huch_visual_navigation);
			// send(msg);

#ifdef MAVLINK_ENABLED_HUCH

			// get magnetic 2D compass measurement
			if(sensor_type == 1) { // hmc5843
				yaw_meas = 128 * ((calcYaw() / PI) + 1.);
				// Logger::log("Ctrl_Yaw yaw_meas", yaw_meas, Logger::LOGLEVEL_DEBUG);
			}
			else { // cmp02
				yaw_meas = DataCenter::get_sensor(6);
			}
			// calculate controller output
			yaw = (((sp - yaw_meas) + compass_res_three_half) % compass_res) - compass_res_half;
			//yaw = ((sp - yaw_meas) + compass_res_three_half) % compass_res; //
			// apply gain
			yaw *= params["yaw_Kc"];
			// //comp = (/255.0) * 6.28;
			// yaw = (int16_t)(prm_yaw_P * (0.0 + (comp/255.0) * 6.28));
			// Logger::log("Ctrl_Yaw (psi_est, yaw)", DataCenter::get_sensor(6), yaw, Logger::LOGLEVEL_INFO);

			// FIXME: optical compass
			// yaw1 = (int16_t)(params["yaw_Kc"] * (0.0 - huch_visual_navigation.psi_vc));
			// pid_yaw->setSp(0.0);
			// yaw = pid_yaw->calc(dtf, (0.0 - huch_visual_navigation.visual_compass));
			// Logger::log("Ctrl_Yaw (psi_est, yaw1)", huch_visual_navigation.psi_vc, yaw1, Logger::LOGLEVEL_INFO);
			//yaw = 0;

#endif // MAVLINK_ENABLED_HUCH	
			uint32_t time_boot_ms = get_time_ms();
			mavlink_msg_debug_pack( system_id(), component_id, &msg, time_boot_ms, 108, yaw);
			AppLayer<mavlink_message_t>::send(msg);
			mavlink_msg_debug_pack( system_id(), component_id, &msg, time_boot_ms, 109, sp);
			AppLayer<mavlink_message_t>::send(msg);
	
			// write output to shared store
			DataCenter::set_extctrl_yaw(yaw);
			// Logger::log("Ctrl_Yaw (n,r,y)", v, Logger::LOGLEVEL_INFO);

			chan.usec = get_time_us();
			chan.index = CHAN_YAW;    
			chan.value = yaw;
			mavlink_msg_huch_generic_channel_encode(system_id(),
																							component_id,
																							&msg,
																							&chan);
			AppLayer<mavlink_message_t>::send(msg);

		}
	}

	float Ctrl_Yaw::mapfromto(float x, float minf, float maxf, float mint, float maxt) {
		float rf, rt;
		rf = maxf - minf;
		rt = maxt - mint;
		return (((x - minf)/rf) * rt) + mint;
	}

	float Ctrl_Yaw::calcYaw() {
		static float xmag_max = 0.0, ymag_max = 0.0, zmag_max = 0.0;
		static float xmag_min = 0.0, ymag_min = 0.0, zmag_min = 0.0;
		float norm;
		float xmag_map, ymag_map, zmag_map;
		float yaw;
		huch_magnetic_kompass = DataCenter::get_huch_magnetic_kompass();

		// Logger::log(name(), "calc kompass(x,y,z): ",
		// 						huch_magnetic_kompass.data_x,
		// 						huch_magnetic_kompass.data_y, 
		// 						huch_magnetic_kompass.data_z,
		// 						Logger::LOGLEVEL_DEBUG);
		// Logger::log(name(), "calc attitude(pitch,roll): ",
		// 						attitude.pitch,
		// 						attitude.roll, 
		// 						Logger::LOGLEVEL_DEBUG);

		// taken from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267295038
		// Arhtur Spooner

		//this part is required to normalize the magnetic vector
		if (huch_magnetic_kompass.data_x > xmag_max) {xmag_max = huch_magnetic_kompass.data_x;}
		if (huch_magnetic_kompass.data_y > ymag_max) {ymag_max = huch_magnetic_kompass.data_y;}
		if (huch_magnetic_kompass.data_z > zmag_max) {zmag_max = huch_magnetic_kompass.data_z;}

		if (huch_magnetic_kompass.data_x < xmag_min) {xmag_min = huch_magnetic_kompass.data_x;}
		if (huch_magnetic_kompass.data_y < ymag_min) {ymag_min = huch_magnetic_kompass.data_y;}
		if (huch_magnetic_kompass.data_z < zmag_min) {zmag_min = huch_magnetic_kompass.data_z;}
    
		xmag_map = mapfromto(huch_magnetic_kompass.data_x, xmag_min, xmag_max, -1., 1.);
		ymag_map = mapfromto(huch_magnetic_kompass.data_y, ymag_min, ymag_max, -1., 1.);
		zmag_map = mapfromto(huch_magnetic_kompass.data_z, zmag_min, zmag_max, -1., 1.);

		//normalize the magnetic vector
		norm = sqrtf( powf(xmag_map, 2) + powf(ymag_map, 2) + powf(zmag_map, 2));
		xmag_map /= norm;
		ymag_map /= norm;
		zmag_map /= norm;
  
		//compare Applications of Magnetic Sensors for Low Cost Compass Systems by Michael J. Caruso
		//for the compensated Yaw equations...
		//http://www.ssec.honeywell.com/magnetic/datasheets/lowcost.pdf
		// yaw = atan2f((ymag_map * cosf(attitude.pitch) + zmag_map * sinf(attitude.pitch)),
		// 						 (xmag_map * cosf(attitude.roll)) + (ymag_map * sinf(attitude.roll) * sinf(attitude.pitch)) + (zmag_map * sinf(attitude.roll) * cosf(attitude.pitch))
		// 						 );
		yaw = atan2f((ymag_map * cosf(attitude.roll) + zmag_map * sinf(attitude.roll)),
								 (xmag_map * cosf(attitude.pitch)) + (ymag_map * sinf(attitude.pitch) * sinf(attitude.roll)) + (zmag_map * sinf(attitude.pitch) * cosf(attitude.roll))
								 );
		// YawU=atan2(-ymag_map, xmag_map) *180/PI;

		return yaw;
	}

	void Ctrl_Yaw::default_conf() {
		params["yaw_Kc"] = 100.0;
		params["yaw_Ti"] = 0.0;
		params["yaw_Td"] = 0.0;
		params["reset_sp"] = 0.0;
	}

	void Ctrl_Yaw::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		// controller params yaw
		iter = args.find("yaw_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["yaw_Kc"];
		}
		iter = args.find("yaw_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["yaw_Ti"];
		}
		iter = args.find("yaw_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["yaw_Td"];
		}

		iter = args.find("reset_sp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["reset_sp"];
		}

		Logger::log("ctrl_yaw::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: yaw_Kc", params["yaw_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: yaw_Ti", params["yaw_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: yaw_Td", params["yaw_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: reset_sp", params["reset_sp"], Logger::LOGLEVEL_DEBUG);
	}
}

#endif // HAVE_MAVLINK_H

