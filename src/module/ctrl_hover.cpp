// talk to FC with mkpackage
#include "ctrl_hover.h"

#include <mavlink.h>
#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <sstream>

#include "core/logger.h"
#include "utility.h"
#include "core/protocolstack.h"
#include "protocol/mkpackage.h"
#include "datacenter.h"

using namespace std;

namespace mavhub {
	// Ctrl_Hover::Ctrl_Hover(int component_id_, int numchan_, const list<pair<int, int> > chanmap_, const map<string, string> args) {
  Ctrl_Hover::Ctrl_Hover(const map<string, string> args) {
		read_conf(args);
		// component_id = component_id_;
		app_id = 3;
		app_name = "ctrl_hover";
		kal = new Kalman_CV();
		pid_alt = new PID(ctl_bias, ctl_Kc, ctl_Ti, ctl_Td);
		//numchan = numchan_;
		chanmap.reserve(numchan);
		raw.reserve(numchan);
		pre.reserve(numchan);
		premod.reserve(numchan);
		baro_ref = 0.0;
		param_request_list = 0;
		param_count = 2;
		list<pair<int, int> >::const_iterator iter;
		//iter = chanmap_.begin();
		iter = chanmap_pairs.begin();
		for(int i = 0; i < numchan; i++) {
			// XXX: iter vs. index
			chanmap[i] = iter->second;
			Logger::log("Ctrl_Hover chantype", chanmap[i], Logger::LOGLEVEL_INFO);
			// assign preprocessors according to sensor type
			switch(chanmap[i]) {
			case USS:
				premod[i] = new PreProcessorUSS();
				break;
			case BARO:
 				premod[i] = new PreProcessorBARO();
				break;
			case ACC:
				premod[i] = new PreProcessorACC();
				break;
			case IR_SHARP_30_3V:
				// sigma, beta, k
				premod[i] = new PreProcessorIR(1.1882e-06, 1.9528e-05, 0.42);
				break;
			case IR_SHARP_150_3V:
				// sigma, beta, k
				premod[i] = new PreProcessorIR(9.3028e-09, 4.2196e-06, 0.42);
				break;
			}	
			iter++;
		}
  }

  Ctrl_Hover::~Ctrl_Hover() {
		if(kal)
			delete kal;
	}

  void Ctrl_Hover::handle_input(const mavlink_message_t &msg) {
		vector<int> v(16);
		char* param_id;
		// hover needs:
		// huch_attitude
		// huch_altitude
		// huch_ranger
		Logger::log("Ctrl_Hover got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HUCH_ATTITUDE:	
			// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
			//Logger::log("Ctrl_Hover got huch_attitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_attitude_decode(&msg, &attitude);
			//Logger::log("Ctrl_Hover", attitude.xacc, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_HUCH_FC_ALTITUDE:
			// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
			//Logger::log("Ctrl_Hover got huch_altitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_fc_altitude_decode(&msg, &altitude);
			//Logger::log("Ctrl_Hover", altitude.baro, altitude.baroref, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_MANUAL_CONTROL:
			// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
			//Logger::log("Ctrl_Hover got huch_altitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
			mavlink_msg_manual_control_decode(&msg, &manual_control);
			// Logger::log("Ctrl_Hover", (int)manual_control.target, manual_control.thrust, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Hover::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == owner->system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == owner->system_id()) {
				Logger::log("Ctrl_Hover::handle_input: PARAM_SET for this system", (int)owner->system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Hover::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, (int8_t*)param_id);
					Logger::log("Ctrl_Hover::handle_input: PARAM_SET for param_id", std::string(param_id), Logger::LOGLEVEL_INFO);
					if(!strcmp("setpoint_stick", param_id)) {
						ctl_sticksp = (int)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for ctl_sticksp", ctl_sticksp, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("setpoint_value", param_id)) {
						ctl_sp = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for ctl_sp", ctl_sp, Logger::LOGLEVEL_INFO);
					}	
				}
			}
			break;
		default:
			break;
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
		ostringstream o;
		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		// strapdown matrix
		CvMat* C;
		CvMat *accel_b; // body accel vector
		CvMat *accel_g; // global accel vector (derotated)
		mavlink_local_position_t pos;
		int run_cnt = 0;
		double gas;

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

		// init position stuff
		C = cvCreateMat(3,3, CV_32FC1);
		cvSetIdentity(C, cvRealScalar(1));
		accel_b = cvCreateMat(3, 1, CV_32FC1);
		accel_g = cvCreateMat(3,1, CV_32FC1);
		pos.usec = get_time_us();
		pos.x = 0.0;
		pos.y = 0.0;
		pos.z = 0.0;
		pos.vx = 0.0;
		pos.vy = 0.0;
		pos.vz = 0.0;

		Logger::debug("Ctrl_Hover started");
		// MKPackage msg_setneutral(1, 'c');
		// owner->send(msg_setneutral);
		while(true) {
			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time

			// return params list
			if(param_request_list) {
				Logger::log("Ctrl_Hover::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;
				mavlink_msg_param_value_pack(owner->system_id(), component_id, &msg, (int8_t *)"setpoint_value", ctl_sp, 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner->system_id(), component_id, &msg, (int8_t *)"setpoint_stick", ctl_sticksp, 1, 0);
				send(msg);
			}

			// 1. collect data

			// get ranger data
			ranger = DataCenter::get_huch_ranger();
			// Logger::log(ranger.ranger1, ranger.ranger2, ranger.ranger3, Logger::LOGLEVEL_INFO);

			raw[0] = ranger.ranger1; // USS
			raw[1] = altitude.baro; // barometer
			raw[2] = attitude.zacc; // z-acceleration
			raw[3] = ranger.ranger2; // ir ranger 1
			raw[4] = ranger.ranger3; // ir ranger 2

			o.str("");
			o << "ctrl_hover raw: dt: " << dt << ", uss: " << raw[0] << ", baro: " << raw[1] << ", zacc: " << raw[2] << ", ir1: " << raw[3] << ", ir2: " << raw[4];
			//Logger::log(o.str(), Logger::LOGLEVEL_INFO);

			// 2. preprocess: linearize, common units, heuristic filtering
			preproc();

			// 2.a. validity / kalman H

			// USS
			if(in_range(pre[0].first, 100.0, 2000.0))
				pre[0].second = 1;
			else
				pre[0].second = 0;
			// BARO
			pre[1].first -= (0.05 * pre[2].first);
			// IR1: 0 = uss, 4 = ir2
			if(pre[0].first <= 320.0 && pre[4].first <= 320.0)
				pre[3].second = 1;
			else	
				pre[3].second = 0;
			// IR2
			if(in_range(pre[0].first, 300.0, 1000.0))
				pre[4].second = 1;
			else
				pre[4].second = 0;

			// XXX: pack this into preprocessing proper
			if(pre[0].second > 0 && run_cnt % 100 == 0)
				reset_baro_ref(pre[0].first);
			pre[1].first = pre[1].first - baro_ref;
			
			// 2.b set kalman measurement transform matrix H
			for(int i = 0; i < numchan; i++) {
				switch(chanmap[i]) {
				case ACC:
					kal->setMeasTransAt(i, 2, pre[i].second);
					break;
				default:
					kal->setMeasTransAt(i, 0, pre[i].second);
					break;
				}
			}
			//Kalman_CV::cvPrintMat(kal->getMeasTransMat(), 5, 3, (char *)"H");

			// debug out
			o.str("");
			o << "ctrl_hover pre: dt: " << dt << ", uss: " << pre[0] << ", baro: " << pre[1] << ", zacc: " << pre[2] << ", ir1: " << pre[3] << ", ir2: " << pre[4];
			//Logger::log(o.str(), Logger::LOGLEVEL_INFO);

			// Logger::log("baro_ref:", baro_ref, Logger::LOGLEVEL_INFO);

			// XXX: this does not work well because i don't have proper yaw angle
			// // 2.a position
			// // strapdown
			// sd_comp_C(&attitude, C);
			// Kalman_CV::cvPrintMat(C, 3, 3, "C");
			// cvmSet(accel_b, 0,0, attitude.xacc * MKACC2MM);
			// cvmSet(accel_b, 1,0, attitude.yacc * MKACC2MM);
			// cvmSet(accel_b, 2,0, (attitude.zaccraw - 512) * MKACC2MM);
			// cvMatMul(C, accel_b, accel_g);
			// Kalman_CV::cvPrintMat(accel_b, 3, 1, "accel_b");
			// Kalman_CV::cvPrintMat(accel_g, 3, 1, "accel_g");
			// // cvmSet(accel_g, 2, 0, cvmGet(accel_g, 2, 0) - 1.0);
			// // integrate
			// // pos.vx += cvmGet(accel_g, 0, 0) * dt * 1e-6;
			// // pos.vy += cvmGet(accel_g, 1, 0) * dt * 1e-6;
			// // pos.vz += cvmGet(accel_g, 2, 0) * dt * 1e-6;
			// pos.vx = cvmGet(accel_g, 0, 0); // * dt * 1e-6;
			// pos.vy = cvmGet(accel_g, 1, 0); // * dt * 1e-6;
			// pos.vz = cvmGet(accel_g, 2, 0); // * dt * 1e-6;
			// pos.x += pos.vx * dt * 1e-6;
			// pos.y += pos.vy * dt * 1e-6;
			// pos.z += pos.vz * dt * 1e-6;
			// mavlink_msg_local_position_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &pos);
			// send(msg);

			// set attitude
			ml_attitude.usec = get_time_us();
			ml_attitude.roll  = attitude.xgyroint * MKGYRO2RAD;
			ml_attitude.pitch = attitude.ygyroint * MKGYRO2RAD;
			ml_attitude.yaw   = attitude.zgyroint * MKGYRO2RAD;
			ml_attitude.rollspeed  = attitude.xgyro * MKGYRO2RAD;
			ml_attitude.pitchspeed = attitude.ygyro * MKGYRO2RAD;
			ml_attitude.yawspeed   = attitude.zgyro * MKGYRO2RAD;
			mavlink_msg_attitude_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &ml_attitude);
			send(msg);

			// 3. kalman filter
			// update timestep
			kal->update_F_dt(dt * 1e-6);
			// copy measurements
			for(int i = 0; i < numchan; i++) {
				kal->setMeasAt(i, 0, pre[i].first);
			}
			// Kalman_CV::cvPrintMat(kal->getTransMat(), 3, 3, "F");
			// Kalman_CV::cvPrintMat(kal->getMeas(), 5, 1, (char *)"meas");
			// Kalman_CV::cvPrintMat(kal->getStatePost(), 3, 1, (char *)"meas");
			// evaluate filter: predict + update
			kal->eval();
			// Logger::log("Ctrl_Hover run", extctrl.gas, Logger::LOGLEVEL_INFO);
			// Logger::log("Ctrl_Hover dt", dt, Logger::LOGLEVEL_INFO);

			ctrl_hover_state.uss = pre[0].first;
			ctrl_hover_state.baro = pre[1].first;
			ctrl_hover_state.accz = pre[2].first;
			ctrl_hover_state.ir1 = pre[3].first;
			ctrl_hover_state.ir2 = pre[4].first;
			ctrl_hover_state.kal_s0 = cvmGet(kal->getStatePost(), 0, 0);
			ctrl_hover_state.kal_s1 = cvmGet(kal->getStatePost(), 1, 0);
			ctrl_hover_state.kal_s2 = cvmGet(kal->getStatePost(), 2, 0);
			mavlink_msg_huch_ctrl_hover_state_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &ctrl_hover_state);
			send(msg);

			// 4. run controller
			// setpoint on stick
			if(ctl_sticksp) {
				ctl_sp = (int)(manual_control.thrust * 15.0);
			}

			pid_alt->setSp(ctl_sp);
			gas = pid_alt->calc((double)dt * 1e-6, ctrl_hover_state.kal_s0);

			// enforce more limits
			if(!ctl_sticksp) {
				if(gas > (manual_control.thrust * 4)) { // 4 <- stick_gain
					pid_alt->setIntegralM1();
					gas = manual_control.thrust * 4;
				}
			}

			if(manual_control.thrust < 5) // reset below threshold
				pid_alt->setIntegral(0.0);

			extctrl.gas = (int16_t)gas;

			// gas out
			dbg.ind = 0;
			dbg.value = gas;
			mavlink_msg_debug_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			send(msg);

			// PID error
			dbg.ind = 1;
			dbg.value = pid_alt->getErr();
			mavlink_msg_debug_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			send(msg);

			// PID integral
			dbg.ind = 2;
			dbg.value = pid_alt->getPv_int();
			mavlink_msg_debug_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			send(msg);

			// PID derivative
			dbg.ind = 3;
			dbg.value = pid_alt->getDpv();
			mavlink_msg_debug_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			send(msg);

			// PId setpoint
			dbg.ind = 4;
			dbg.value = pid_alt->getSp();
			mavlink_msg_debug_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			send(msg);

			// attitude_controller_output.thrust = extctrl.gas / 4 - 128;
			// mavlink_msg_attitude_controller_output_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg, &attitude_controller_output);
			// send(msg);

			//extctrl.gas = 255 * (double)rand()/RAND_MAX;

			//Logger::log("Ctrl_Hover: ctl out", extctrl.gas, Logger::LOGLEVEL_INFO);
			MKPackage msg_extctrl(1, 'b', (uint8_t *)&extctrl, sizeof(extctrl));
			send(msg_extctrl);
			
			// stats
			run_cnt += 1;
			// XXX: usleep call takes ~5000 us?
			usleep(10000);
		}
		return;
  }

	void Ctrl_Hover::preproc() {
		// Logger::log("preprocessing :)", Logger::LOGLEVEL_INFO);
		for(int i = 0; i < numchan; i++) {
			pre[i] = premod[i]->calc(i, raw[i]);
		}
	}

	// XXX: pack this into preprocessing proper
	void Ctrl_Hover::reset_baro_ref(double ref) {
		// baro_ref = ctrl_hover_state.baro - ref;
		baro_ref = pre[1].first - ref;
		//Logger::log("Resetting baro ref", Logger::LOGLEVEL_INFO);
	}

	void Ctrl_Hover::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("hier", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		iter = args.find("numsens");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> numchan;
		}

		iter = args.find("inmap");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> chanmap_pairs;
		}

		iter = args.find("ctl_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_bias;
		}

		iter = args.find("ctl_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_Kc;
		}

		iter = args.find("ctl_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_Ti;
		}

		iter = args.find("ctl_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_Td;
		}

		iter = args.find("ctl_sp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_sp;
		}

		iter = args.find("ctl_bref");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_bref;
		}

		iter = args.find("ctl_sticksp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_sticksp;
		}

		// XXX
		Logger::log("ctrl_hover::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: numchan", numchan, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: inmap", chanmap_pairs, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_bias", ctl_bias, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_Kc", ctl_Kc, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_Ti", ctl_Ti, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_Td", ctl_Td, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_sp", ctl_sp, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_bref", ctl_bref, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_hover::read_conf: ctl_sticksp", ctl_sticksp, Logger::LOGLEVEL_INFO);

		return;
	}

	// compute strapdown matrix C from attitude
	int Ctrl_Hover::sd_comp_C(mavlink_huch_attitude_t *a, CvMat *C) {
		double phi, theta, psi;
		double sphi, stheta, spsi;
		double cphi, ctheta, cpsi;
		
		// roll is phi
		phi = a->xgyroint * MKGYRO2RAD;
		// roll is theta
		theta = a->ygyroint * MKGYRO2RAD;
		// yaw is psi
		psi = 0.0; // a->zgyroint * MKGYRO2RAD;

		sphi = sin(phi);
		stheta = sin(theta);
		spsi = sin(psi);

		cphi = cos(phi);
		ctheta = cos(theta);
		cpsi = cos(psi);

		cvmSet(C, 0, 0, ctheta * cpsi);
		cvmSet(C, 0, 1, -cphi * spsi + sphi * stheta * cpsi);
		cvmSet(C, 0, 2, sphi * spsi + cphi * stheta * cpsi);
		cvmSet(C, 1, 0, ctheta * spsi);
		cvmSet(C, 1, 1, cphi * cpsi + sphi * stheta * spsi);
		cvmSet(C, 1, 2, -sphi * cpsi + cphi * stheta * spsi);
		cvmSet(C, 2, 0, -stheta);
		cvmSet(C, 2, 1, sphi * ctheta);
		cvmSet(C, 2, 2, cphi * ctheta);
		// cvPrintMat(C, 3, 3, "C");
		return 0;
	}

}
