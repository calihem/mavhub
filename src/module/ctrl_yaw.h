// control yaw component

#ifndef _CTRL_YAW_H_
#define _CTRL_YAW_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "modulebase.h"
#include "debug_channels.h"
#include "core/logger.h"
#include "protocol/protocollayer.h"
// #include "PID.h"

namespace mavhub {
	class Ctrl_Yaw : public ModuleBase {
	public:
		/// Ctor: yaw controller
		Ctrl_Yaw(const std::map<std::string, std::string>,
						 const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		/// Dtor
		virtual ~Ctrl_Yaw();
		/// protocol stack input handling
		virtual void handle_input(const mavlink_message_t &msg);

	protected:
		/// this thread's main method
		virtual void run();

	private:
		uint16_t component_id;
		int16_t yaw;
#ifdef MAVLINK_ENABLED_HUCH
		mavlink_huch_visual_navigation_t huch_visual_navigation;
		mavlink_huch_visual_flow_t huch_visual_flow;
		mavlink_huch_magnetic_kompass_t huch_magnetic_kompass;
		mavlink_huch_generic_channel_t chan;
#endif // MAVLINK_ENABLED_HUCH	
		mavlink_attitude_t attitude;

		int sensor_type;
		// params
		// request
		int param_request_list;
		// container
		std::map<std::string, double>	params;
		double prm_yaw_P;
		// controller instances
		// PID* pid_yaw;

		/// set reasonable config defaults
		virtual void default_conf();
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
		/// calculate yaw (heading) from 3-axis magnetometer and attitude
		virtual float calcYaw();
		/// util function to map x from from-range into to-range
		virtual float mapfromto(float x, float minf, float maxf, float mint, float maxt);
	};
}

#endif // HAVE_MAVLINK_H

#endif
