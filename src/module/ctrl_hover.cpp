// talk to FC with mkpackage
#include "ctrl_hover.h"

#include <mavlink.h>
#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <sstream>

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include "protocol/mkpackage.h"
#include "core/datacenter.h"

#define PI2 6.2831853071795862
#define WINSIZE 9
#define WINSIZE_HALF WINSIZE/2

using namespace std;

namespace mavhub {
	// Ctrl_Hover::Ctrl_Hover(int component_id_, int numchan_, const list<pair<int, int> > chanmap_, const map<string, string> args) {
  Ctrl_Hover::Ctrl_Hover(const map<string, string> args) : 
		AppInterface("ctrl_hover"), 
		AppLayer<mavlink_message_t>("ctrl_hover"),
		AppLayer<mk_message_t>("ctrl_hover"),
		uss_win(WINSIZE, 0.0),
		uss_win_sorted(WINSIZE, 0.0),
		uss_win_idx(0),
		uss_win_idx_m1(0),
		uss_med(0.0),
		d_uss(0.0),
		uss_plaus(0)
	{
		// sensible pre-config defaults
		conf_defaults();
		read_conf(args);
		//Logger::log("ctrl_hover: param numsens", params["numsens"], Logger::LOGLEVEL_INFO);
		numchan = params["numsens"];
		kal = new Kalman_CV(); // FIXME: should be parameterised
		kal_setRFromParams();
		pid_alt = new PID(ctl_bias, ctl_Kc, ctl_Ti, ctl_Td);

		typemap.reserve(numchan);
		chanmap.reserve(numchan);
		raw.reserve(numchan);
		pre.reserve(numchan);
		premean.resize(numchan);
		precov.resize(numchan);
		premod.reserve(numchan);

		// stats = cvCreateMat(numchan, 2, CV_32FC1);
		// stats_data = cvCreateMat(numchan, 16, CV_32FC1);
		stats.resize(numchan);

		baro_ref = 0.0;
		param_request_list = 0;
		param_count = 2;
		mk_debugout_digital_offset = 2;
		list<pair<int, int> >::const_iterator iter;
		//iter = chanmap_.begin();
		iter = typemap_pairs.begin();
		for(int i = 0; i < numchan; i++) {
			// XXX: iter vs. index
			typemap[i] = iter->second;
			Logger::log("Ctrl_Hover chantype", typemap[i], Logger::LOGLEVEL_DEBUG);
			// assign preprocessors according to sensor type
			switch(typemap[i]) {
			case USS_FC:
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
			premean[i] = 0.0;
			precov[i] = 1.0;
			stats[i] = new Stat_MeanVar(64);
			iter++;
		}
		// channel mapping
		iter = chanmap_pairs.begin();
		for(int i = 0; i < numchan; i++) {
			// XXX: iter vs. index
			chanmap[i] = iter->second;
			Logger::log("Ctrl_Hover chanmap", i, chanmap[i], Logger::LOGLEVEL_DEBUG);
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
		//Logger::log("Ctrl_Hover got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);

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
			if(typemap[0] == USS_FC)
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
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Hover::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Hover::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Hover::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Hover::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

					// if(!strcmp("ctl_sticksp", (const char *)param_id)) {
					// 	params["ctl_sticksp"] = (int)mavlink_msg_param_set_get_param_value(&msg);
					// 	Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for params[\"ctl_sticksp\"]", params["ctl_sticksp"], Logger::LOGLEVEL_INFO);
					// }	else if(!strcmp("ctl_setpoint", (const char *)param_id)) {
					// 	params["ctl_setpoint"] = (double)mavlink_msg_param_set_get_param_value(&msg);
					// 	Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for ctl_setpoint", params["ctl_setpoint"], Logger::LOGLEVEL_INFO);
					// }	else
					if(!strcmp("output_enable", (const char *)param_id)) {
						output_enable = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for output_enable", output_enable, Logger::LOGLEVEL_INFO);
						// action requests
					}	else if(!strcmp("set_neutral_rq", (const char *)param_id)) {
						set_neutral_rq = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for set_neutral_rq", set_neutral_rq, Logger::LOGLEVEL_INFO);
						// enable groundstation talk
					}	else if(!strcmp("gs_en", (const char *)param_id)) {
						params["gs_en"] = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for gs_en", params["gs_en"], Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("gs_en_dbg", (const char *)param_id)) {
						params["gs_en_dbg"] = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for gs_en_dbg", params["gs_en_dbg"], Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("gs_en_stats", (const char *)param_id)) {
						params["gs_en_stats"] = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for gs_en_stats", params["gs_en_stats"], Logger::LOGLEVEL_INFO);
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
					// update kalman
					kal_setRFromParams();
				}
			}
			break;
		default:
			break;
		}
		if(msg.sysid == system_id() && msg.msgid == 0) {//FIXME: set right msgid
			//TODO
		}
  }

  void Ctrl_Hover::handle_input(const mk_message_t &msg) { }

  void Ctrl_Hover::run() {
		int buf[1]; // to MK buffer
		uint8_t flags = 0;
		uint64_t dt = 0; // in microseconds
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
		mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_hb, system_type, MAV_AUTOPILOT_HUCH);

		// XXX: infrared valid FIRs
		//double tmp_d[] = {0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05};
		double tmp_d[] = {0.125, 0.25, 0.25, 0.25, 0.125};
		FIR filt_valid_uss(5, tmp_d);
		// FIR filt_valid_uss(11, tmp_c);
		//double tmp_c[] = {0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05};
		FIR filt_valid_ir1(5, tmp_d);
		FIR filt_valid_ir2(5, tmp_d);

		// USS smoothing
		// CvMat* fmu_src = cvCreateMat(1, 8, CV_32FC1);
		// CvMat* fmu_dst = cvCreateMat(1, 8, CV_32FC1);
		// cvSmooth filt_median_uss1(fmu_src, fmu_dst, CV_MEDIAN, 1, 0, 0.0, 0.0);
		

		// more timing
		// 100 Hz is too much at the moment, especially with more than one
		// UDP interface
		// int update_rate = 50; // 100 Hz
		int wait_freq = ctl_update_rate? 1000000 / ctl_update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		vector<double> covvec(2);

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
		mk_message_t msg_debug_on;
		mklink_msg_pack(&msg_debug_on, MK_FC_ADDRESS, MK_MSG_TYPE_POLL_DEBUG, buf, 1);
		sleep(1);
		AppLayer<mk_message_t>::send(msg_debug_on);
		Logger::log("Ctrl_Hover debug request sent to FC", Logger::LOGLEVEL_INFO);

		while(true) {
			/* wait time */
			usec = get_time_us();
			uint64_t end = usec;
			wait_time = wait_freq - (end - start);
			// wait_time = (wait_time < 0)? 0: wait_time;
			if(wait_time < 0) {
				Logger::log("ALARM: time", Logger::LOGLEVEL_INFO);
				wait_time = 0;
			}
		
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

			// send pixhawk std structs to groundstation
			// FIXME: moved here 2011-05-25
			// manual control
			mavlink_msg_manual_control_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &manual_control);
			AppLayer<mavlink_message_t>::send(msg);
			if(params["gs_en"]) {
				// attitude
				mavlink_msg_attitude_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &ml_attitude);
				AppLayer<mavlink_message_t>::send(msg);
			}
			
			if(params["en_skipmain"] > 0.0)
				continue;

			// continue;

			// 1. collect data

			// get ranger data
			//ranger = DataCenter::get_huch_ranger();
			// Logger::log(ranger.ranger1, ranger.ranger2, ranger.ranger3, Logger::LOGLEVEL_INFO);

			// double tmp = tmp;
			raw[0] = (int)DataCenter::get_sensor(chanmap[0]); //ranger.ranger1; // USS
			raw[1] = altitude.baro; // barometer
			raw[2] = attitude.zacc; // z-acceleration
			raw[3] = (int)DataCenter::get_sensor(chanmap[3]); //ranger.ranger2; // ir ranger 1
			raw[4] = (int)DataCenter::get_sensor(chanmap[4]); // ranger.ranger3; // ir ranger 2

			// record raw altitude readings
			// Logger::log("hc_raw_en", params["hc_raw_en"], Logger::LOGLEVEL_INFO);
			if(params["ch_raw_en"] >= 1.0) {
				// copy data
				//for(int i = 0; i < params["numsens"]; i++) {
				ch_raw.raw0 = raw[0];
				ch_raw.raw1 = raw[1];
				ch_raw.raw2 = raw[2];
				ch_raw.raw3 = raw[3];
				ch_raw.raw4 = raw[4];
					//}
				// send
				mavlink_msg_huch_hc_raw_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &ch_raw);
				AppLayer<mavlink_message_t>::send(msg);
			}

			// o.str("");
			// o << "ctrl_hover raw: dt: " << dt << ", uss: " << raw[0] << ", baro: " << raw[1] << ", zacc: " << raw[2] << ", ir1: " << raw[3] << ", ir2: " << raw[4];
			// Logger::log(o.str(), Logger::LOGLEVEL_INFO);

			// 2. preprocess: linearize, common units, heuristic filtering
			preproc();

			// 2.a0: range correction from attitude for infrared rangers
			//att2dist(3);
			//att2dist(4);

			// 2.z post pre-processing running statistics
			for(int i = 0; i < numchan; i++) {
				stats[i]->update(pre[i].first);
				// Logger::log("ctrl_hover: stats:", stats[i]->get_mean(),
				// 						stats[i]->get_var(), Logger::LOGLEVEL_INFO);
			}

			// 2.a. validity / kalman H, kalman R
			double tmp_valid;

			////////////////////////////////////////////////////////////
			// USS
			// USS smoothing / median filtering
			uss_win_idx_m1 = uss_win_idx;
			uss_win_idx = (uss_win_idx + 1) % WINSIZE;
			uss_win[uss_win_idx] = pre[0].first;
			// check max z velocity
			d_uss = uss_win[uss_win_idx] - uss_win[uss_win_idx_m1];
			uss_plaus = (fabs(d_uss) <= (params["uss_max_vz"] * dt * 1e-6));
			send_debug(&msg, &dbg, DBG_USS_MAX_VZ, (params["uss_max_vz"] * dt * 1e-6));
			send_debug(&msg, &dbg, DBG_USS_D, d_uss);

			uss_win_sorted = uss_win;
			sort(uss_win_sorted.begin(), uss_win_sorted.end());
			//Logger::log("ctrl_hover: vector size", uss_win.size(), Logger::LOGLEVEL_INFO);
			// Logger::log("ctrl_hover: median window:", uss_win, Logger::LOGLEVEL_INFO);
			// Logger::log("ctrl_hover: median window:", uss_win_sorted, Logger::LOGLEVEL_INFO);
			uss_med = uss_win_sorted[WINSIZE_HALF];
			//Logger::log("ctrl_hover: uss_plaus", d_uss, uss_plaus, Logger::LOGLEVEL_INFO);

			// if(params["uss_med_en"] >= 1.0) {
			// 	Logger::log("ctrl_hover: median window:", uss_med, d_uss, Logger::LOGLEVEL_INFO);
			// 	pre[0].first = uss_med;
			// }

			// if(!uss_plaus) {
			// 	// low pass
			// 	uss_win[uss_win_idx] = uss_win[uss_win_idx_m1]; // + (0.01 * d_uss);
			// 	pre[0].first = uss_win[uss_win_idx];
			// }

			if(in_range(pre[0].first, params["uss_llim"], params["uss_hlim"])
				 && pre[3].first > params["uss_llim"]
				 && uss_plaus)
				tmp_valid = 1.0;
			else {
				// pre[0].first = uss_win[uss_win_idx_m1];
				tmp_valid = 0.0;
				// retain last valid
			}
				// }
			// filter valid
			precov[0] = filt_valid_uss.calc(tmp_valid);
			pre[0].second = (precov[0] >= 0.99) ? 1 : 0;

			////////////////////////////////////////////////////////////
			// BARO: correct from accel-z
			pre[1].first -= (0.05 * pre[2].first);
			// BARO: disable below 30.0 cm
			// FIXME
			if((pre[3].first < 300.0 && pre[4].first < 600) || stats[0]->get_mean() < 1600.0) {
				precov[1] = 0.0;
				//pre[1].second = 0;
			} else {
				precov[1] = 1.0;
				//pre[1].second = 1;
			}
			pre[1].second = precov[1];

			////////////////////////////////////////////////////////////
			// ACCZ
			precov[2] = 1.0;
			pre[2].second = precov[2];

			////////////////////////////////////////////////////////////
			// IR1: 0 = uss, 4 = ir2
			// FIXME: use IR1 own's measurement
			// if(pre[0].first <= 300.0) //
			//if(in_range(pre[3].first, 100.0, 200.0))
			// 39, 300
			if(in_range(pre[3].first, params["ir1_llim"], params["ir1_ulim"])
				 && pre[4].first < 600.0) // carpet problem: ir1 freaks out on carpet FIXME
				tmp_valid = 1.0;
			else
				tmp_valid = 0.0;
			precov[3] = filt_valid_ir1.calc(tmp_valid);
			pre[3].second = (precov[3] >= 0.99) ? 1 : 0;

			 ////////////////////////////////////////////////////////////
			 // IR2
			 // FIXME: use IR2 own's measurement
			 // if(in_range(pre[0].first, 300.0, 700.0))
			 // if(in_range(pre[4].first, 300.0, 560.0))
			// 300, 600
			 if(in_range(pre[4].first, params["ir2_llim"], params["ir2_ulim"]) &&
					pre[3].first > 300.0)
				 tmp_valid = 1;
			 else
				 tmp_valid = 0;
			 precov[4] = filt_valid_ir2.calc(tmp_valid);
			 pre[4].second = (precov[4] >= 0.99) ? 1 : 0;

			 // FIXME: pack this into preprocessing proper
			 // enable / disable via parameter
			 if((pre[0].second > 0 || pre[3].second > 0) &&
					run_cnt_cyc == 0 && params["ctl_bref"] != 0.0) {
				 reset_baro_ref(pre[0].first);
				 //reset_baro_ref(cvmGet(kal->getStatePost(), 0, 0));
			 }
			 pre[1].first = pre[1].first - baro_ref;

			 // // 2.b set kalman measurement transform matrix H
			 for(int i = 0; i < numchan; i++) {
				 switch(typemap[i]) {
				 case ACC:
					 kal->setMeasTransAt(i, 2, pre[i].second);
					 // kal->setMeasTransAt(i, 2, 1.0);
					 break;
				 case BARO:
					 kal->setMeasTransAt(i, 0, pre[i].second);
					 // kal->setMeasTransAt(i, 0, 1.0);
					 break;
				 default:
					 kal->setMeasTransAt(i, 0, pre[i].second);
					 // kal->setMeasTransAt(i, 0, 0.0);
					 break;
				 }
			 }
			 // //Kalman_CV::cvPrintMat(kal->getMeasTransMat(), 5, 3, (char *)"H");

			 // start cov
			 // // 2.c set kalman measurement noise covariance matrix R
			 // for(int i = 0; i < numchan; i++) {
			 // 	switch(typemap[i]) {
			 // 	case ACC:
			 // 		covvec[0] = 20.0;
			 // 		covvec[1] = 0.01;
			 // 		break;
			 // 	case USS_FC:
			 // 	case USS:
			 // 		covvec[0] = 40.0;
			 // 		covvec[1] = 0.2;
			 // 		break;
			 // 	case BARO:
			 // 		covvec[0] = 40.0;
			 // 		covvec[1] = 0.5;
			 // 		break;
			 // 	case IR_SHARP_30_3V:
			 // 	case IR_SHARP_30_5V:
			 // 		covvec[0] = 40.0;
			 // 		covvec[1] = 0.1;
			 // 		break;
			 // 	case IR_SHARP_150_3V:
			 // 	case IR_SHARP_150_5V:
			 // 		covvec[0] = 40.0;
			 // 		covvec[1] = 0.3;
			 // 		break;
			 // 	default:
			 // 		// kal->setMeasTransAt(i, 0, pre[i].second);
			 // 		break;
			 // 	}
			 // 	// determine heuristic "variance"
			 // 	stats[i]->update_heur(integrate_and_saturate(stats[i]->get_var_e(),
			 // 																							 pre[i].second,
			 // 																							 covvec[1],
			 // 																							 covvec[0]));
			 // 	// Logger::log("ctrl_hov:",
			 // 	// 						stats[i]->get_var_e(),
			 // 	// 						calc_var_nonlin(stats[i]->get_var_e(), covvec[1], covvec[0]), Logger::LOGLEVEL_INFO);

			 // 	//kal->setMeasNoiseCovAt(i, i, covvec[pre[i].second]);
			 // 	kal->setMeasNoiseCovAt(i, i, calc_var_nonlin(stats[i]->get_var_e(), covvec[1], covvec[0]));
			 // 	// stats[i]->update_heur(calc_var_nonlin(stats[i]->get_var_e(), covvec[1], covvec[0]));
			 // }
			 // end cov

			 //	Kalman_CV::cvPrintMat(kal->getMeasNoiseCov(), 5, 5, (char *)"R");

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
			 //Logger::log("Ctrl_Hover dt", dt, params["uss_max_vz"], Logger::LOGLEVEL_INFO);

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
			if(params["ctl_sticksp"]) {
				params["ctl_setpoint"] = (int)(manual_control.thrust * 15.0);
			}

			// FIXME: christian gps-logging
			//Logger::log("alt:", ctrl_hover_state.kal_s0, Logger::LOGLEVEL_INFO);
			//cout << ctrl_hover_state.kal_s0 << endl;

			pid_alt->setSp(params["ctl_setpoint"]);
			gas = pid_alt->calc((double)dt * 1e-6, ctrl_hover_state.kal_s0);
			// printf("gas 1: %f\n", gas);

			// enforce more limits
			if(params["ctl_sticksp"] < 1.0 &&
				 params["physical"] > 0) {
				if(gas > (manual_control.thrust * 4)) { // 4 <- stick_gain
					pid_alt->setIntegralM1();
					gas = manual_control.thrust * 4;
				}
			}
			//printf("gas 2: %f, man: %f\n", gas, manual_control.thrust);

			// reset integral
			if(manual_control.thrust < params["ctl_mingas"]) // reset below threshold
				pid_alt->setIntegral(0.0);

			// limit integral
			if(gas > params["ctl_maxgas"])
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
				mk_message_t msg_extctrl;
				mklink_msg_pack(&msg_extctrl, MK_FC_ADDRESS, MK_MSG_TYPE_EXT_CTRL, &extctrl, sizeof(extctrl));
				AppLayer<mk_message_t>::send(msg_extctrl);
			}

			// send data to groundstation
			// parameters if requested: parameter list and data
			if(param_request_list) {
				Logger::log("Ctrl_Hover::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				// update params from kal meas noise covmat
				kal_getParamsFromR();

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					AppLayer<mavlink_message_t>::send(msg);
				}

				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"setpoint_value", ctl_sp, 1, 0);
				// send(msg);
				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"setpoint_stick", params["ctl_sticksp"], 1, 0);
				// send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"output_enable", output_enable, 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				// trigger setneutral
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"set_neutral_rq", set_neutral_rq, 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"gs_en", params["gs_en"], 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"gs_en_dbg", params["gs_en_dbg"], 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"gs_en_stats", params["gs_en_stats"], 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				// PID params
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"PID_bias", pid_alt->getBias(), 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"PID_Kc", pid_alt->getKc(), 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"PID_Ti", pid_alt->getTi(), 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"PID_Td", pid_alt->getTd(), 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
			}

			// set neutral?
			if(set_neutral_rq > 0) {
				Logger::log("Requesting setneutral from flightcontrol", Logger::LOGLEVEL_INFO);
				mk_message_t msg_setneutral;
				mklink_msg_pack(&msg_setneutral, MK_FC_ADDRESS, MK_MSG_TYPE_SETNEUTRAL_REQ, NULL, 0);
				AppLayer<mk_message_t>::send(msg_setneutral);
				set_neutral_rq = 0;
			}

			// typed message forwarding

			// // huch attitude
			mavlink_msg_huch_attitude_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &attitude);
			AppLayer<mavlink_message_t>::send(msg);

			// huch_fc_altitude
			// mavlink_msg_huch_fc_altitude_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &altitude);
			// send(msg);
			// mk_fc_status
			mavlink_msg_mk_fc_status_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &mk_fc_status);
			AppLayer<mavlink_message_t>::send(msg);
			// huch ctrl_hover
			mavlink_msg_huch_ctrl_hover_state_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &ctrl_hover_state);
			AppLayer<mavlink_message_t>::send(msg);

			// send debug signals to groundstation
			if(params["gs_en_dbg"]) {
				// gas out
				send_debug(&msg, &dbg, ALT_GAS, gas);
				// // PID error
				// send_debug(&msg, &dbg, ALT_PID_ERR, pid_alt->getErr());
				// PID integral
				send_debug(&msg, &dbg, ALT_PID_INT, pid_alt->getPv_int());
				// // PID derivative
				// send_debug(&msg, &dbg, ALT_PID_D, pid_alt->getDpv());
				// PID setpoint
				send_debug(&msg, &dbg, ALT_PID_SP, pid_alt->getSp());
				// VALID USS
				send_debug(&msg, &dbg, DBG_VALID_USS, pre[0].second * 256);
				// VALID IR1
				send_debug(&msg, &dbg, DBG_VALID_IR1, pre[3].second * 256);
				// VALID IR2
				send_debug(&msg, &dbg, DBG_VALID_IR2, pre[4].second * 256);
			}
			// timing
			if(params["dbg_alt_dt_s"]) {
				// dt
				send_debug(&msg, &dbg, ALT_DT_S, dt * 1e-6);
			}

			// send debug signals to groundstation
			if(params["gs_en_stats"]) {
				int i, offs;
				for(i = 0; i < numchan; i++) {
					offs = i * 3;
					send_debug(&msg, &dbg, CH1_MEAN + offs, stats[i]->get_mean());
					send_debug(&msg, &dbg, CH1_VAR + offs, stats[i]->get_var());
					send_debug(&msg, &dbg, CH1_VAR_E + offs, calc_var_nonlin(stats[i]->get_var_e(), covvec[1], covvec[0]));
					//stats[i]->get_var_e());
				}
			}

			if(params["dbg_kal_vars"]) {
				send_debug(&msg, &dbg, DBG_KAL_VAR0, ((1.0 - precov[0]) * 10.0 + 1) * params["kal_R0"]);
				send_debug(&msg, &dbg, DBG_KAL_VAR1, ((1.0 - precov[1]) * 10.0 + 1) * params["kal_R1"]);
				send_debug(&msg, &dbg, DBG_KAL_VAR2, ((1.0 - precov[2]) * 10.0 + 1)  * params["kal_R2"]);
				send_debug(&msg, &dbg, DBG_KAL_VAR3, ((1.0 - precov[3]) * 10.0 + 1)  * params["kal_R3"]);
				send_debug(&msg, &dbg, DBG_KAL_VAR4, ((1.0 - precov[4]) * 10.0 + 1) * params["kal_R4"]);
			}

			// XXX: does this make problems with time / qgc?
			// XXX: see qgc/src/uas/UAS.cc -> attitude_control
			// attitude_controller_output.thrust = extctrl.gas / 4 - 128;
			// mavlink_msg_attitude_controller_output_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &attitude_controller_output);
			// send(msg);
			
			// stats
			run_cnt += 1;
			run_cnt_cyc = run_cnt % 10;
			// send heartbeat
			if(run_cnt_cyc == 0)
				AppLayer<mavlink_message_t>::send(msg_hb);
			// XXX: usleep call takes ~5000 us?
			//usleep(10000);
		}
		return;
  }

	// send debug
	void Ctrl_Hover::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
		dbg->ind = index;
		dbg->value = value;
		mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
		AppLayer<mavlink_message_t>::send(*msg);
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
		// use average
		// baro_ref = pre[1].first - ref;
		baro_ref = stats[1]->get_mean() - ref;
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
	
	void Ctrl_Hover::conf_defaults() {
		set_neutral_rq = 0;
		params["ctl_mingas"] = 5.0;
		params["ctl_maxgas"] = 800;
		params["kal_R0"] = 0.3;
		params["kal_R1"] = 2.0;
		params["kal_R2"] = 0.01;
		params["kal_R3"] = 0.3;
		params["kal_R4"] = 0.3;
		params["physical"] = 1.0; // safe?
		params["ir1_llim"] = 40.0;
		params["ir1_ulim"] = 300.0;
		params["ir2_llim"] = 300.0;
		params["ir2_ulim"] = 600.0;
	}

	void Ctrl_Hover::kal_setRFromParams() {
		Logger::log("setrfromparams:", params["kal_R0"], params["kal_R1"], Logger::LOGLEVEL_INFO);
		Logger::log("setrfromparams:", params["kal_R2"], params["kal_R3"], Logger::LOGLEVEL_INFO);
		Logger::log("setrfromparams:", params["kal_R4"], Logger::LOGLEVEL_INFO);
		kal->setMeasNoiseCovAt(0, 0, (double)params["kal_R0"]);
		kal->setMeasNoiseCovAt(1, 1, params["kal_R1"]);
		kal->setMeasNoiseCovAt(2, 2, params["kal_R2"]);
		kal->setMeasNoiseCovAt(3, 3, params["kal_R3"]);
		kal->setMeasNoiseCovAt(4, 4, params["kal_R4"]);
	}

	void Ctrl_Hover::kal_getParamsFromR() {
		params["kal_R0"] = kal->getMeasNoiseCovAt(0, 0);
		params["kal_R1"] = kal->getMeasNoiseCovAt(1, 1);
		params["kal_R2"] = kal->getMeasNoiseCovAt(2, 2);
		params["kal_R3"] = kal->getMeasNoiseCovAt(3, 3);
		params["kal_R4"] = kal->getMeasNoiseCovAt(4, 4);
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
			// s >> numchan;
			s >> params["numsens"];
			// numchan = 5;
		}

		iter = args.find("inmap");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> typemap_pairs;
		}

		iter = args.find("chmap");
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
			// s >> ctl_sp;
			s >> params["ctl_setpoint"];
		}

		iter = args.find("ctl_bref");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_bref"];
		}

		iter = args.find("ctl_sticksp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			// s >> ctl_sticksp;
			s >> params["ctl_sticksp"];
		}

		iter = args.find("ctl_mingas");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			//s >> ctl_mingas;
			s >> params["ctl_mingas"];
		}

		iter = args.find("ctl_maxgas");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			// s >> ctl_maxgas;
			s >> params["ctl_maxgas"];
		}

		// output enable
		iter = args.find("output_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> output_enable;
		}

		// mavlink to groundstation enable
		iter = args.find("gs_en");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["gs_en"];
		}

		// mavlink to groundstation debug data enable
		iter = args.find("gs_en_dbg");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["gs_en_dbg"];
		}

		// mavlink to groundstation debug data enable
		iter = args.find("dbg_alt_dt_s");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["dbg_alt_dt_s"];
		}

		// mavlink to groundstation debug data enable
		iter = args.find("dbg_kal_vars");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["dbg_kal_vars"];
		}

		// mavlink to groundstation debug statistics data
		iter = args.find("gs_en_stats");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["gs_en_stats"];
		}

		// main controller update rate
		iter = args.find("ctl_update_rate");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_update_rate;
		}
		else
			ctl_update_rate = 100;

		// mavlink to groundstation debug statistics data
		iter = args.find("uss_med_en");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["uss_med_en"];
		}

		// enable hover ctrl raw sensor value dump
		iter = args.find("ch_raw_en");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ch_raw_en"];
		}

		// uss sensor conf
		iter = args.find("uss_llim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["uss_llim"];
		}

		// uss sensor conf
		iter = args.find("uss_hlim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["uss_hlim"];
		}

		// uss sensor conf
		iter = args.find("uss_max_vz");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["uss_max_vz"];
		}

		////////////////////////////////////////////////////////////
		// kalman conf: meas cov
		iter = args.find("kal_R0");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["kal_R0"];
		}
		// kalman conf: meas cov
		iter = args.find("kal_R1");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["kal_R1"];
		}
		// kalman conf: meas cov
		iter = args.find("kal_R2");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["kal_R2"];
		}
		// kalman conf: meas cov
		iter = args.find("kal_R3");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["kal_R3"];
		}
		// kalman conf: meas cov
		iter = args.find("kal_R4");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["kal_R4"];
		}
		
		// kalman conf: measurements via H or R
		iter = args.find("kal_use_var");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["kal_use_var"];
		}
		
		////////////////////////////////////////////////////////////


		// skip main loop?
		iter = args.find("en_skipmain");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["en_skipmain"];
		}

		// IR params
		iter = args.find("ir1_llim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ir1_llim"];
		}
		// IR params
		iter = args.find("ir1_ulim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ir1_ulim"];
		}
		// IR params
		iter = args.find("ir2_llim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ir2_llim"];
		}
		// IR params
		iter = args.find("ir2_ulim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ir2_ulim"];
		}



		// XXX
		Logger::log("ctrl_hover::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: numchan", numchan, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: inmap", typemap_pairs, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: chmap", chanmap_pairs, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_bias", ctl_bias, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_Kc", ctl_Kc, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_Ti", ctl_Ti, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_Td", ctl_Td, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: params[\"ctl_setpoint\"]", params["ctl_setpoint"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: params[\"ctl_bref\"]", params["ctl_bref"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: params[\"ctl_sticksp\"]", params["ctl_sticksp"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: params[\"ctl_mingas\"]", params["ctl_mingas"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: params[\"ctl_maxgas\"]", params["ctl_maxgas"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: output_enable", output_enable, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: gs_en", params["gs_en"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: gs_en_dbg", params["gs_en_dbg"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: gs_en_stats", params["gs_en_stats"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: ctl_update_rate", ctl_update_rate, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_hover::read_conf: usS_max_vz", params["uss_max_vz"], Logger::LOGLEVEL_DEBUG);

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
		// FIXME: optimize away the vector assignment
		attitude.xacc     = v[0]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_accnick);
		attitude.yacc     = v[1]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_accroll);
		attitude.zacc     = v[2]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_acctop);
		attitude.zaccraw  = v[3]  = Ctrl_Hover::debugout_getval_s(dbgout, ADval_acctopraw);
		attitude.xaccmean = v[4]  = Ctrl_Hover::debugout_getval_s(dbgout, ATTmeanaccnick);
		attitude.yaccmean = v[5]  = Ctrl_Hover::debugout_getval_s(dbgout, ATTmeanaccroll);
		attitude.zaccmean = v[6]  = Ctrl_Hover::debugout_getval_s(dbgout, ATTmeanacctop);
		// watch for the minus sign
		attitude.xgyro    = v[7]  = -Ctrl_Hover::debugout_getval_s(dbgout, ADval_gyrroll);
		attitude.ygyro    = v[8]  = -Ctrl_Hover::debugout_getval_s(dbgout, ADval_gyrnick);
		attitude.zgyro    = v[9]  = -Ctrl_Hover::debugout_getval_s(dbgout, ADval_gyryaw);
		attitude.xgyroint = v[10] = -Ctrl_Hover::debugout_getval_s32(dbgout, ATTintrolll, ATTintrollh);
		attitude.ygyroint = v[11] = -Ctrl_Hover::debugout_getval_s32(dbgout, ATTintnickl, ATTintnickh);
		attitude.zgyroint = v[12] = -Ctrl_Hover::debugout_getval_s32(dbgout, ATTintyawl, ATTintyawh);
		attitude.xmag = v[13] = 0.0;
		attitude.ymag = v[14] = 0.0;
		attitude.zmag = v[15] = 0.0;

		//Logger::log("debugout2attitude:", v, Logger::LOGLEVEL_INFO);

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
		// FIXME: typemap[0] is ultrasonic sensors
		// DataCenter::set_huch_ranger_at(*ranger, 0);
		DataCenter::set_sensor(chanmap[0], (double)v[0]);
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
		manual_control.target = system_id();
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
