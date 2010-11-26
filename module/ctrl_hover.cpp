// talk to FC with mkpackage
#include "ctrl_hover.h"

#include <mavlink.h>
#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <stdio.h>

#include "main.h" //system_id
#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include "mkpackage.h"

using namespace std;

namespace mavhub {
  Ctrl_Hover::Ctrl_Hover() {
		kal = new Kalman_CV();
  }

  Ctrl_Hover::~Ctrl_Hover() {
		if(kal)
			delete kal;
	}

  void Ctrl_Hover::handle_input(const mavlink_message_t &msg) {
		vector<int> v(16);
		// hover needs:
		// huch_attitude
		// huch_altitude
		// huch_ranger
		// Logger::log("Ctrl_Hover got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

		if(msg.msgid == MAVLINK_MSG_ID_HUCH_ATTITUDE) {
			// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
			//Logger::log("Ctrl_Hover got huch_attitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_attitude_decode(&msg, &attitude);
			//Logger::log("Ctrl_Hover", attitude.xacc, Logger::LOGLEVEL_INFO);
		}
		else if(msg.msgid == MAVLINK_MSG_ID_HUCH_ALTITUDE) {
			// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
			//Logger::log("Ctrl_Hover got huch_altitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_altitude_decode(&msg, &altitude);
			//Logger::log("Ctrl_Hover", altitude.baro, altitude.baroref, Logger::LOGLEVEL_INFO);
		}

		if(msg.sysid == owner->system_id() && msg.msgid == 0) {//FIXME: set right msgid
			//TODO
		}
  }

  void Ctrl_Hover::run() {
		// int buf[1];
		uint8_t flags = 0;
		uint64_t dt = 0;
		struct timeval tk, tkm1; // timevals
		//mavlink_message_t msg_i;

		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);
		
		// rel
		//flags |= (APFLAG_GENERAL_ON | APFLAG_KEEP_VALUES | APFLAG_HEIGHT_CTRL1 );
		// abs
		flags |= (APFLAG_GENERAL_ON | APFLAG_KEEP_VALUES | APFLAG_FULL_CTRL );
		extctrl.remote_buttons = 0;	/* for lcd menu */
		extctrl.nick = 0; //nick;
		extctrl.roll = 0; //roll;
		extctrl.yaw = 0; //yaw;
		extctrl.gas = 0; //gas;	/* MotorGas = min(ExternControl.Gas, StickGas) */
		extctrl.height = 0; //height;
		/* for autopilot */
		extctrl.AP_flags = flags;
		extctrl.frame = 'E';	/* get ack from flightctrl */
		extctrl.config = 0;	/* activate external control via serial iface in FlightCtrl */

		Logger::debug("Ctrl_Hover started");
		// MKPackage msg_setneutral(1, 'c');
		while(true) {
			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time
			
			// owner->send(msg_setneutral);
			kal->update_F_dt(dt * 1e-6);
			kal->setMeasAt(0, 0, 0.1);
			// Kalman_CV::cvPrintMat(kal->getTransMat(), 3, 3, "F");
			Kalman_CV::cvPrintMat(kal->getMeas(), 5, 1, (char *)"meas");
			kal->eval();
			extctrl.gas = 255 * (double)rand()/RAND_MAX;
			// Logger::log("Ctrl_Hover run", extctrl.gas, Logger::LOGLEVEL_INFO);
			Logger::log("Ctrl_Hover dt", dt, altitude.baro, Logger::LOGLEVEL_INFO);
			MKPackage msg_extctrl(1, 'b', (uint8_t *)&extctrl, sizeof(extctrl));
			owner->send(msg_extctrl);
			// XXX: usleep call takes ~5000 us?
			usleep(1);
		}
		return;
  }
}
