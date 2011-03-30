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
#include "core/datacenter.h"

#define PI2 6.2831853071795862

using namespace std;

namespace mavhub {
	// Ctrl_Hover::Ctrl_Hover(int component_id_, int numchan_, const list<pair<int, int> > chanmap_, const map<string, string> args) {
  Ctrl_Hover::Ctrl_Hover(const map<string, string> args) : AppLayer("ctrl_hover") {
		// sensible pre-config defaults
		ctl_mingas = 5.0;
		ctl_maxgas = 800;
		set_neutral_rq = 0;
		read_conf(args);
		// component_id = component_id_;
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
		mk_debugout_digital_offset = 2;
		list<pair<int, int> >::const_iterator iter;
		//iter = chanmap_.begin();
		iter = chanmap_pairs.begin();
		for(int i = 0; i < numchan; i++) {
			// XXX: iter vs. index
			chanmap[i] = iter->second;
			Logger::log("Ctrl_Hover chantype", chanmap[i], Logger::LOGLEVEL_DEBUG);
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
				premod[i] = new PreProcessorIR(40.0, 320.0, 1.1882e-06, 1.9528e-05, 0.42);
				break;
			case IR_SHARP_150_3V:
				// sigma, beta, k
				premod[i] = new PreProcessorIR(300.0, 1000.0, 9.3028e-09, 4.2196e-06, 0.42);
				break;
			case IR_SHARP_30_5V:
				// sigma, beta, k
				premod[i] = new PreProcessorIR(40.0, 350.0, 7.2710e-08, 3.7778e-5, 0.42);
				break;
			case IR_SHARP_150_5V:
				// sigma, beta, k
				premod[i] = new PreProcessorIR(250.0, 1500.0, 1.5369e-07, 8.8526e-06, 0.42);
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
		static vector<int> v(16);
		static int8_t param_id[15];
		// hover needs:
		// huch_attitude
		// huch_altitude
		// huch_ranger
		Logger::log("Ctrl_Hover got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_MK_DEBUGOUT:
			// Logger::log("Ctrl_Hover got MK_DEBUGOUT", Logger::LOGLEVEL_INFO);
			mavlink_msg_mk_debugout_decode(&msg, (mavlink_mk_debugout_t *)&mk_debugout);
			// MK FlightCtrl IMU data
			debugout2attitude(&mk_debugout);
			// MK FlightCtrl Barometric sensor data
			debugout2altitude(&mk_debugout);
			// MK huch-FlightCtrl I2C USS data
			// XXX: this should be in kopter config, e.g. if settings == fc_has_uss
			debugout2ranger(&mk_debugout, &ranger);
			// real debugout data
			debugout2status(&mk_debugout, &mk_fc_status);

			// put into standard pixhawk structs
			// raw IMU
			//set_pxh_raw_imu();
			set_pxh_attitude();
			set_pxh_manual_control();

			// publish_data(get_time_us());
			// deadlock problem
			// mavlink_msg_huch_attitude_encode(42, 23, &msg_j, &huch_attitude);
			// send(msg_j);
			break;

		// case MAVLINK_MSG_ID_HUCH_ATTITUDE:	
		// 	// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
		// 	//Logger::log("Ctrl_Hover got huch_attitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
		// 	mavlink_msg_huch_attitude_decode(&msg, &attitude);
		// 	//Logger::log("Ctrl_Hover", attitude.xacc, Logger::LOGLEVEL_INFO);
		// 	break;
		// case MAVLINK_MSG_ID_HUCH_FC_ALTITUDE:
		// 	// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
		// 	//Logger::log("Ctrl_Hover got huch_altitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
		// 	mavlink_msg_huch_fc_altitude_decode(&msg, &altitude);
		// 	//Logger::log("Ctrl_Hover", altitude.baro, altitude.baroref, Logger::LOGLEVEL_INFO);
		// 	break;
		// case MAVLINK_MSG_ID_MANUAL_CONTROL:
		// 	// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
		// 	//Logger::log("Ctrl_Hover got huch_altitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
		// 	mavlink_msg_manual_control_decode(&msg, &manual_control);
		// 	// Logger::log("Ctrl_Hover", (int)manual_control.target, manual_control.thrust, Logger::LOGLEVEL_INFO);
		// 	break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Hover::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == owner()->system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == owner()->system_id()) {
				Logger::log("Ctrl_Hover::handle_input: PARAM_SET for this system", (int)owner()->system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Hover::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Hover::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);
					if(!strcmp("setpoint_stick", (const char *)param_id)) {
						ctl_sticksp = (int)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for ctl_sticksp", ctl_sticksp, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("setpoint_value", (const char *)param_id)) {
						ctl_sp = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for ctl_sp", ctl_sp, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("output_enable", (const char *)param_id)) {
						output_enable = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for output_enable", output_enable, Logger::LOGLEVEL_INFO);
						// action requests
					}	else if(!strcmp("set_neutral_rq", (const char *)param_id)) {
						set_neutral_rq = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for set_neutral_rq", set_neutral_rq, Logger::LOGLEVEL_INFO);
						// enable groundstation talk
					}	else if(!strcmp("gs_enable", (const char *)param_id)) {
						gs_enable = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for gs_enable", gs_enable, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("gs_enable_dbg", (const char *)param_id)) {
						gs_enable_dbg = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for gs_enable_dbg", gs_enable_dbg, Logger::LOGLEVEL_INFO);
						// PID params
					}	else if(!strcmp("PID_bias", (const char *)param_id)) {
						pid_alt->setBias((double)mavlink_msg_param_set_get_param_value(&msg));
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for bias", pid_alt->getBias(), Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("PID_Kc", (const char *)param_id)) {
						pid_alt->setKc((double)mavlink_msg_param_set_get_param_value(&msg));
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for Kc", pid_alt->getKc(), Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("PID_Ti", (const char *)param_id)) {
						pid_alt->setTi((double)mavlink_msg_param_set_get_param_value(&msg));
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for Ti", pid_alt->getTi(), Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("PID_Td", (const char *)param_id)) {
						pid_alt->setTd((double)mavlink_msg_param_set_get_param_value(&msg));
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for Td", pid_alt->getTd(), Logger::LOGLEVEL_INFO);
					}	
				}
			}
			break;
		default:
			break;
		}
		if(msg.sysid == owner()->system_id() && msg.msgid == 0) {//FIXME: set right msgid
			//TODO
		}
  }

  void Ctrl_Hover::run() {
		int buf[1]; // to MK buffer
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
		int run_cnt_cyc = 0;
		double gas;

		// heartbeat
		int system_type = MAV_QUADROTOR;
		mavlink_message_t msg_hb;
		mavlink_msg_heartbeat_pack(owner()->system_id(), component_id, &msg_hb, system_type, MAV_AUTOPILOT_HUCH);

		// XXX: infrared valid FIRs
		double tmp_c[] = {0.125, 0.25, 0.25, 0.25, 0.125};
		FIR filt_valid_ir1(5, tmp_c);
		FIR filt_valid_ir2(5, tmp_c);

		// more timing
		int update_rate = 100; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

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

		//extctrl.height = 0; //height;
		/* for autopilot */
		extctrl.AP_flags = flags;
		extctrl.frame = 'E';	/* get ack from flightctrl */
		extctrl.config = 0;	/* activate external control via serial iface in FlightCtrl */

		// init position stuff
		C = cvCreateMat(3,3, CV_32FC1);
		cvSetIdentity(C, cvRealScalar(1));
		accel_b = cvCreateMat(3, 1, CV_32FC1);
		accel_g = cvCreateMat(3,1, CV_32FC1);
		pos.usec = 0; // get_time_us(); XXX: qgc bug
		pos.x = 0.0;
		pos.y = 0.0;
		pos.z = 0.0;
		pos.vx = 0.0;
		pos.vy = 0.0;
		pos.vz = 0.0;

		Logger::log("Ctrl_Hover started:", name(), Logger::LOGLEVEL_INFO);
		
		// request debug msgs
		buf[0] = 10;
		MKPackage msg_debug_on(1, 'd', 1, buf, 1);
		sleep(1);
		send(msg_debug_on);
		Logger::log("Ctrl_Hover debug request sent to FC", Logger::LOGLEVEL_INFO);

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

			// Logger::log("Ctrl_Hover slept for", wait_time, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time
			
			// continue;

			// 1. collect data

			// get ranger data
			ranger = DataCenter::get_huch_ranger();
			// Logger::log(ranger.ranger1, ranger.ranger2, ranger.ranger3, Logger::LOGLEVEL_INFO);

			raw[0] = ranger.ranger1; // USS
			raw[1] = altitude.baro; // barometer
			raw[2] = attitude.zacc; // z-acceleration
			raw[3] = ranger.ranger2; // ir ranger 1
			raw[4] = ranger.ranger3; // ir ranger 2

			// o.str("");
			// o << "ctrl_hover raw: dt: " << dt << ", uss: " << raw[0] << ", baro: " << raw[1] << ", zacc: " << raw[2] << ", ir1: " << raw[3] << ", ir2: " << raw[4];
			//Logger::log(o.str(), Logger::LOGLEVEL_INFO);

			// 2. preprocess: linearize, common units, heuristic filtering
			preproc();

			// 2.a0: range correction from attitude for infrared rangers
			att2dist(3);
			att2dist(4);

			// 2.a. validity / kalman H

			// gone to pp_uss
			// // USS
			// if(in_range(pre[0].first, 100.0, 2000.0))
			// 	pre[0].second = 1;
			// else
			// 	pre[0].second = 0;

			// BARO: correct from accel-z
			pre[1].first -= (0.05 * pre[2].first);

			// IR1: 0 = uss, 4 = ir2
			// FIXME: use IR1 own's measurement
			if(pre[0].first <= 300.0) // && pre[4].first <= 300.0)
				pre[3].second = 1;
			else	
				pre[3].second = 0;
			pre[3].second = (filt_valid_ir1.calc(pre[3].second) >= 1.0) ? 1 : 0;

			// IR2
			// FIXME: use IR2 own's measurement
			if(in_range(pre[0].first, 300.0, 700.0))
				pre[4].second = 1;
			else
				pre[4].second = 0;
			pre[4].second = (filt_valid_ir2.calc(pre[4].second) >= 1.0) ? 1 : 0;

			// XXX: pack this into preprocessing proper
			if(pre[0].second > 0 && run_cnt_cyc == 0)
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
			// o.str("");
			// o << "ctrl_hover pre: dt: " << dt << ", uss: " << pre[0] << ", baro: " << pre[1] << ", zacc: " << pre[2] << ", ir1: " << pre[3] << ", ir2: " << pre[4];
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
			// mavlink_msg_local_position_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &pos);
			// send(msg);

			// set attitude
			ml_attitude.usec = 0; // get_time_us(); XXX: qgc bug
			ml_attitude.roll  = attitude.xgyroint * MKGYRO2RAD;
			ml_attitude.pitch = attitude.ygyroint * MKGYRO2RAD;
			ml_attitude.yaw   = attitude.zgyroint * MKGYRO2RAD;
			ml_attitude.rollspeed  = attitude.xgyro * MKGYRO2RAD;
			ml_attitude.pitchspeed = attitude.ygyro * MKGYRO2RAD;
			ml_attitude.yawspeed   = attitude.zgyro * MKGYRO2RAD;

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

			// reset integral
			if(manual_control.thrust < ctl_mingas) // reset below threshold
				pid_alt->setIntegral(0.0);

			// limit integral
			if(gas > ctl_maxgas)
				pid_alt->setIntegralM1();

			// min/max limits
			gas = limit_gas(gas);

			extctrl.gas = (int16_t)gas;
			extctrl.nick = (int16_t)DataCenter::get_extctrl_nick();
			extctrl.roll = (int16_t)DataCenter::get_extctrl_roll();
			extctrl.yaw = (int16_t)DataCenter::get_extctrl_yaw();
			// Logger::log("Ctrl_Hover nick", , Logger::LOGLEVEL_INFO);
			// extctrl.gas = 255 * (double)rand()/RAND_MAX;

			// send control output to FC
			// Logger::log("Ctrl_Hover: ctl out", extctrl.gas, Logger::LOGLEVEL_INFO);
			if(output_enable > 0) {
				MKPackage msg_extctrl(1, 'b', (uint8_t *)&extctrl, sizeof(extctrl));
				send(msg_extctrl);
			}

			// send data to groundstation
			// parameters if requested: parameter list and data
			if(param_request_list) {
				Logger::log("Ctrl_Hover::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"setpoint_value", ctl_sp, 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"setpoint_stick", ctl_sticksp, 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"output_enable", output_enable, 1, 0);
				send(msg);
				// trigger setneutral
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"set_neutral_rq", set_neutral_rq, 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"gs_enable", gs_enable, 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"gs_enable_dbg", gs_enable_dbg, 1, 0);
				send(msg);
				// PID params
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"PID_bias", pid_alt->getBias(), 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"PID_Kc", pid_alt->getKc(), 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"PID_Ti", pid_alt->getTi(), 1, 0);
				send(msg);
				mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"PID_Td", pid_alt->getTd(), 1, 0);
				send(msg);
			}

			// set neutral?
			if(set_neutral_rq > 0) {
				Logger::log("Requesting setneutral from flightcontrol", Logger::LOGLEVEL_INFO);
				MKPackage msg_setneutral(1, 'c');
				send(msg_setneutral);
				set_neutral_rq = 0;
			}

			// typed message forwarding
			// huch attitude
			mavlink_msg_huch_attitude_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &attitude);
			send(msg);
			// huch_fc_altitude
			// mavlink_msg_huch_fc_altitude_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &altitude);
			// send(msg);
			// mk_fc_status
			mavlink_msg_mk_fc_status_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &mk_fc_status);
			send(msg);
			// huch ctrl_hover
			mavlink_msg_huch_ctrl_hover_state_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &ctrl_hover_state);
			send(msg);
			// send pixhawk std structs to groundstation
			if(gs_enable) {
				// manual control
				mavlink_msg_manual_control_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &manual_control);
				send(msg);
				// attitude
				mavlink_msg_attitude_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &ml_attitude);
				send(msg);
			}

			// send debug signals to groundstation
			if(gs_enable_dbg) {
				// gas out
				send_debug(&msg, &dbg, ALT_GAS, gas);
				// // PID error
				// send_debug(&msg, &dbg, ALT_PID_ERR, pid_alt->getErr());
				// // PID integral
				// send_debug(&msg, &dbg, ALT_PID_INT, pid_alt->getPv_int());
				// // PID derivative
				// send_debug(&msg, &dbg, ALT_PID_D, pid_alt->getDpv());
				// // PID setpoint
				// send_debug(&msg, &dbg, ALT_PID_SP, pid_alt->getSp());
				// VALID USS
				send_debug(&msg, &dbg, DBG_VALID_USS, pre[0].second * 256);
				// VALID IR1
				send_debug(&msg, &dbg, DBG_VALID_IR1, pre[3].second * 256);
				// VALID IR2
				send_debug(&msg, &dbg, DBG_VALID_IR2, pre[4].second * 256);
				// dt
				send_debug(&msg, &dbg, ALT_DT_S, dt * 1e-6);
			}

			// XXX: does this make problems with time / qgc?
			// XXX: see qgc/src/uas/UAS.cc -> attitude_control
			// attitude_controller_output.thrust = extctrl.gas / 4 - 128;
			// mavlink_msg_attitude_controller_output_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &attitude_controller_output);
			// send(msg);
			
			// stats
			run_cnt += 1;
			run_cnt_cyc = run_cnt % 100;
			// send heartbeat
			if(run_cnt_cyc == 0)
				send(msg_hb);
			// XXX: usleep call takes ~5000 us?
			//usleep(10000);
		}
		return;
  }

	// send debug
	void Ctrl_Hover::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
		dbg->ind = index;
		dbg->value = value;
		mavlink_msg_debug_encode(owner()->system_id(), static_cast<uint8_t>(component_id), msg, dbg);
		send(*msg);
	}

	void Ctrl_Hover::preproc() {
		// Logger::log("preprocessing :)", Logger::LOGLEVEL_INFO);
		for(int i = 0; i < numchan; i++) {
			// pre[i] = premod[i]->calc(i, raw[i]);
			premod[i]->calc(pre, i, raw[i]);
		}
	}

	// XXX: pack this into preprocessing proper
	void Ctrl_Hover::reset_baro_ref(double ref) {
		// baro_ref = ctrl_hover_state.baro - ref;
		baro_ref = pre[1].first - ref;
		//Logger::log("Resetting baro ref", Logger::LOGLEVEL_INFO);
	}

	// pre-processing: use attitude for ranger correction
	void Ctrl_Hover::att2dist(int chan) {
		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		//float alpha, beta, reading_f;
		double z;
		float z_corr;
		//long long int reading_new;

		// attitude.xgyroint;
		// attitude.ygyroint;
		// attitude.zgyroint;

		z = pre[chan].first;

		// alpha = PI2 * (float)gyrnick / (GYR_DEG * 360);
		// beta = PI2 * (float)gyrroll / (GYR_DEG * 360);
		// reading_f = (float)reading;
		// reading_f = cosf(alpha) * cosf(beta) * reading_f;

		//reading_new = (long long int)cosf(alpha) * cosf(beta) * reading_f;

		// printf("spp_dist2att: %d, %f, %d, %d, %f, %f, %f\n", reading, reading_f, gyrnick, gyrroll,
		// 			 alpha, beta, reading_f);

		z_corr = cosf(ml_attitude.roll) \
			* cosf(ml_attitude.pitch) \
			* z;

		send_debug(&msg, &dbg, ALT_CORR_BASE + ((chan - 3) * 2), z);
		send_debug(&msg, &dbg, ALT_CORR_BASE + ((chan - 3) * 2) + 1, z_corr);

		// Logger::log("att2dist", ml_attitude.roll, ml_attitude.pitch, Logger::LOGLEVEL_INFO);
		// Logger::log("att2dist", z, z_corr, Logger::LOGLEVEL_INFO);
		// reading_new = (long long int)reading_f;
		// return(reading_new);
		pre[chan].first = z_corr;
		return;
	}
	

	void Ctrl_Hover::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

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

		iter = args.find("ctl_mingas");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_mingas;
		}

		iter = args.find("ctl_maxgas");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_maxgas;
		}

		// output enable
		iter = args.find("output_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> output_enable;
		}

		// mavlink to groundstation enable
		iter = args.find("gs_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> gs_enable;
		}

		// mavlink to groundstation debug data enable
		iter = args.find("gs_enable_dbg");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> gs_enable_dbg;
		}

		// XXX
		Logger::log("ctrl_hover::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: numchan", numchan, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: inmap", chanmap_pairs, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_bias", ctl_bias, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_Kc", ctl_Kc, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_Ti", ctl_Ti, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_Td", ctl_Td, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_sp", ctl_sp, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_bref", ctl_bref, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_sticksp", ctl_sticksp, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_mingas", ctl_mingas, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_maxgas", ctl_maxgas, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: output_enable", output_enable, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: gs_enable", gs_enable, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: gs_enable_dbg", gs_enable_dbg, Logger::LOGLEVEL_DEBUG);

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

  void Ctrl_Hover::debugout2attitude(mavlink_mk_debugout_t* dbgout) {
		// int i;
		static vector<int> v(16);
		// for mkcom mapping compatibility:
		// xaccmean, yaccmean, zacc
		attitude.xacc     = v[0]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_accnick);
		attitude.yacc     = v[1]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_accroll);
		attitude.zacc     = v[2]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_acctop);
		attitude.zaccraw  = v[3]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_acctopraw);
		attitude.xaccmean = v[4]  = Ctrl_Hover::debugout_getval_s(dbgout, ATTmeanaccnick);
		attitude.yaccmean = v[5]  = Ctrl_Hover::debugout_getval_s(dbgout, ATTmeanaccroll);
		attitude.zaccmean = v[6]  = Ctrl_Hover::debugout_getval_s(dbgout, ATTmeanacctop);
		attitude.xgyro    = v[7]  = -Ctrl_Hover::debugout_getval_s(dbgout, ADval_gyrroll);
		attitude.ygyro    = v[8]  = -Ctrl_Hover::debugout_getval_s(dbgout, ADval_gyrnick);
		attitude.zgyro    = v[9]  = -Ctrl_Hover::debugout_getval_s(dbgout, ADval_gyryaw);
		attitude.xgyroint = v[10] = -Ctrl_Hover::debugout_getval_s32(dbgout, ATTintrolll, ATTintrollh);
		attitude.ygyroint = v[11] = -Ctrl_Hover::debugout_getval_s32(dbgout, ATTintnickl, ATTintnickh);
		attitude.zgyroint = v[12] = -Ctrl_Hover::debugout_getval_s32(dbgout, ATTintyawl, ATTintyawh);
		attitude.xmag = v[13] = 0.0;
		attitude.ymag = v[14] = 0.0;
		attitude.zmag = v[15] = 0.0;

		// Logger::log("debugout2attitude:", v, Logger::LOGLEVEL_INFO);

		// despair debug
		// printf("blub: ");
		// printf("%u,", (uint8_t)dbgout->debugout[0]);
		// printf("%u,", (uint8_t)dbgout->debugout[1]);
		// for(i = 0; i < 64; i++) {
		// 	printf("%u,", (uint8_t)dbgout->debugout[i+2]);
		// }
		// printf("\n");
  }

  void Ctrl_Hover::debugout2altitude(mavlink_mk_debugout_t* dbgout) {
		static vector<int16_t> v(2);
		// XXX: use ADval_press
		altitude.baro = v[0] = debugout_getval_s(dbgout, ATTabsh);
		// altitude->baro = v[0] = debugout_getval_u(dbgout, ADval_press);
		// Logger::log("debugout2altitude:", v, altitude->baro, Logger::LOGLEVEL_INFO);
  }

	// this puts USS signal into ranger1 (hardware with USS sensor on
	// FC i2c
  void Ctrl_Hover::debugout2ranger(mavlink_mk_debugout_t* dbgout, mavlink_huch_ranger_t* ranger) {
		static vector<uint16_t> v(3);
		ranger->ranger1 = v[0] = debugout_getval_u(dbgout, USSvalue);
		DataCenter::set_huch_ranger_at(*ranger, 0);
		// Logger::log("debugout2ranger:", v, Logger::LOGLEVEL_INFO);
  }

  void Ctrl_Hover::debugout2status(mavlink_mk_debugout_t* dbgout, mavlink_mk_fc_status_t* status) {
		static vector<int16_t> v(6);
		status->rssi = v[0] = debugout_getval_s(dbgout, RC_rssi);
		status->batt = v[1] = debugout_getval_s(dbgout, ADval_ubat);
		status->gas  = v[2] = debugout_getval_s(dbgout, GASmixfrac2);
		status->nick  = v[3] = debugout_getval_s(dbgout, StickNick);
		status->roll  = v[4] = debugout_getval_s(dbgout, StickRoll);
		status->yaw  = v[5] = debugout_getval_s(dbgout, StickYaw);
		//Logger::log("debugout2status:", v, Logger::LOGLEVEL_INFO);
  }

	// copy huch data into std pixhawk raw_imu
	void Ctrl_Hover::set_pxh_raw_imu() {
		raw_imu.usec = 0; // get_time_us(); XXX: qgc bug
		raw_imu.xacc = attitude.xacc;
		raw_imu.yacc = attitude.yacc;
		raw_imu.zacc = attitude.zaccraw;
		raw_imu.xgyro = attitude.xgyro;
		raw_imu.ygyro = attitude.ygyro;
		raw_imu.zgyro = attitude.zgyro;
		raw_imu.xmag = attitude.xmag;
		raw_imu.ymag = attitude.ymag;
		raw_imu.zmag = attitude.zmag;
	}

	// copy huch data into std pixhawk attitude
	void Ctrl_Hover::set_pxh_attitude() {
		ml_attitude.usec = 0; // get_time_us(); XXX: qgc bug
		ml_attitude.roll  = attitude.xgyroint * MKGYRO2RAD;
		ml_attitude.pitch = attitude.ygyroint * MKGYRO2RAD;
		ml_attitude.yaw   = attitude.zgyroint * MKGYRO2RAD;
		ml_attitude.rollspeed  = attitude.xgyro * MKGYRO2RAD;
		ml_attitude.pitchspeed = attitude.ygyro * MKGYRO2RAD;
		ml_attitude.yawspeed   = attitude.zgyro * MKGYRO2RAD;
	}

	// copy huch data into std pixhawk attitude
	void Ctrl_Hover::set_pxh_manual_control() {
		manual_control.target = owner()->system_id();
		manual_control.thrust = (float)debugout_getval_u(&mk_debugout, CTL_stickgas);
	}

  // fetch unsigned int from mk_debugout
  uint16_t Ctrl_Hover::debugout_getval_u(mavlink_mk_debugout_t* dbgout, int index) {
		int i;
		i = 2 * index + mk_debugout_digital_offset;
		return (uint16_t)dbgout->debugout[i+1] << 8 |
			(uint8_t)dbgout->debugout[i];
  }

  // fetch signed int from mk_debugout
  int16_t Ctrl_Hover::debugout_getval_s(mavlink_mk_debugout_t* dbgout, int index) {
		int i;
		i = 2 * index + mk_debugout_digital_offset;
		return (int16_t)dbgout->debugout[i+1] << 8 |
			(uint8_t)dbgout->debugout[i]; // unsigned part
  }

  // fetch signed int32 from mk_debugout
  int32_t Ctrl_Hover::debugout_getval_s32(mavlink_mk_debugout_t* dbgout, int indexl, int indexh) {
		int il, ih;
		uint16_t ul, uh;
		il = 2 * indexl + mk_debugout_digital_offset;
		ih = 2 * indexh + mk_debugout_digital_offset;
		ul = (uint16_t)dbgout->debugout[il+1] << 8 |
			(uint8_t)dbgout->debugout[il];
		uh = (uint16_t)dbgout->debugout[ih+1] << 8 |
			(uint8_t)dbgout->debugout[ih];

		return (int32_t) uh << 16 | ul;
	}

}
