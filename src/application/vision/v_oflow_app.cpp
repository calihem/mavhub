#include "v_oflow_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

#include <sstream> //istringstream

using namespace std;
using namespace cpp_pthread;
//using namespace hub::slam;

namespace mavhub {

	V_OFLOWApp::V_OFLOWApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
		AppInterface("v_oflow_app", loglevel),
		MavlinkAppLayer("v_oflow_app", loglevel),
		hub::gstreamer::VideoClient(),
		with_out_stream(false),
		take_new_image(1),
		target_system(7),
		target_component(1),
		imu_rate(10),
		of_u(0.0),
		of_v(0.0),
		of_u_i(0.0),
		of_v_i(0.0),
		param_request_list(0),
		cam_matrix(3, 3, CV_32FC1),
		dist_coeffs( cv::Mat::zeros(4, 1, CV_32FC1) ),
		feature_detector(60, 3), //threshold, octaves
		rotation_vector( cv::Mat::zeros(3, 1, CV_64FC1) ),
		translation_vector( cv::Mat::zeros(3, 1, CV_64FC1))
	{
		Logger::log(name(), "Ctor", Logger::LOGLEVEL_DEBUG);

		pthread_mutex_init(&sync_mutex, NULL);
		// invalidate attitude
		attitude.usec = 0;
		attitude.roll = 0.;
		attitude.pitch = 0.;
		attitude.yaw = 0.;

		// // same for mk_imu
		// huch_mk_imu.usec = 0;
		// huch_mk_imu.xacc = 0;
		// huch_mk_imu.yacc = 0;
		// huch_mk_imu.zacc = 0;
		// huch_mk_imu.xgyro = 0;
		// huch_mk_imu.ygyro = 0;
		// huch_mk_imu.zgyro = 0;

		// set sink name
		std::map<std::string,std::string>::const_iterator iter = args.find("sink");
		if( iter != args.end() ) {
			sink_name.assign(iter->second);
		} else {
			log(name(), ": sink argument missing", Logger::LOGLEVEL_DEBUG);
			sink_name.assign("sink0");
		}

		get_value_from_args("out_stream", with_out_stream);

		//TODO: pipe_in, pipe_out

		assign_variable_from_args(component_id);
		assign_variable_from_args(target_system);
		assign_variable_from_args(target_component);
		assign_variable_from_args(imu_rate);
		assign_variable_from_args(en_heartbeat);
		
		// read config
		read_conf(args);

		// get calibration data of camera
		//FIXME: use image dimensions for default camera matrix
		cam_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, 160.0, 0.0, 1.0, 120.0, 0.0, 0.0, 1.0);
		string calib_filename;
		get_value_from_args("calibration_data", calib_filename);
		if(!calib_filename.empty())
			load_calibration_data(calib_filename);

	}

	V_OFLOWApp::~V_OFLOWApp() {}

	void V_OFLOWApp::extract_features() {
		//FIXME: remove benchmark
		//uint64_t start_time = get_time_ms();
		// cv::Mat flip_image;



		// get change of attitude
		//TODO: time check of attitude
		// mavlink_attitude_t attitude_change;
		// attitude_change.roll = new_attitude.roll - old_attitude.roll;
		// attitude_change.pitch = new_attitude.pitch - old_attitude.pitch;
		// attitude_change.yaw = new_attitude.yaw - old_attitude.yaw;
		// Logger::log(name(), "Changed attitude:",
		// 	rad2deg(attitude_change.roll),
		// 	rad2deg(attitude_change.pitch),
		// 	rad2deg(attitude_change.yaw),
		// 	Logger::LOGLEVEL_DEBUG, _loglevel);
	
		// calculate transformation matrix
		// 	double distance = 1.0; //FIXME: use altitude information
		// 	double factor = 300.0;
		// 	float delta_x = factor*2.0*distance*sin(attitude_change.roll/2);
		// 	float delta_y = factor*2.0*distance*sin(attitude_change.pitch/2);
		// rotate image
		cv::Point center(new_image.cols/2, new_image.rows/2);
		// 	cv::Mat rotation_matrix = getRotationMatrix2D(center, -rad2deg(attitude_change.yaw), 1.0);
		// 	cv::Mat rotated_image;
		//FIXME: warp features (not image)
		// 	cv::warpAffine(new_image, rotated_image, rotation_matrix, new_image.size());
		// shift image
		// 	double m[2][3] = {{1, 0, -delta_x}, {0, 1, -delta_y}};
		// 	cv::Mat transform_matrix(2, 3, CV_64F, m);
		// 	cv::Mat transformed_image;
		// 	cv::warpAffine(rotated_image, transformed_image, transform_matrix, rotated_image.size());

		// match descriptors
		// std::vector<std::vector<cv::DMatch> > matches;
		// 	std::vector<std::vector<cv::DMatch> > forward_matches;
		// 	std::vector<std::vector<cv::DMatch> > backward_matches;
		// 	matcher.radiusMatch(old_descriptors, new_descriptors, forward_matches, 100.0);	//0.21 for L2
		// 	matcher.radiusMatch(new_descriptors, old_descriptors, backward_matches, 100.0);	//0.21 for L2
		// 	fusion_matches(forward_matches, backward_matches, matches);

		// matcher.radiusMatch(old_descriptors, new_descriptors, matches, 100.0);	//0.21 for L2
		// if(matches.empty()) {
		// 	Logger::log("no matches were found", Logger::LOGLEVEL_DEBUG, _loglevel);
		// 	return;
		// }

		//TODO: check for ambigous matches

		// TODO: use RANSAC instead of IMU filter?
		// 	std::vector<uint8_t> filter;
		// 	int valid_matches = filter_matches_by_imu< cv::L1<float> >(old_features,
		// 		new_features,
		// 		matches,
		// 		center,
		// 		attitude_change.roll, attitude_change.pitch, attitude_change.yaw,
		// 		delta_x, delta_y,
		// 		filter);
		// 	float valid_rate = (float)valid_matches/filter.size();
		// 	Logger::log(name(), ": Valid match rate is", valid_rate, filter.size(), Logger::LOGLEVEL_INFO, _loglevel);

		// 	cv::Mat H = find_homography(old_features, new_features, matches, CV_RANSAC);
		// 	cv::Mat H = find_homography(old_features, new_features, matches, 0);
		// 	std::cout << H << std::endl;
		// 	double yaw = acos( (H.at<double>(0,0) + H.at<double>(1,1))/2.0 );
		// 	std::cout << "yaw = " << rad2deg(yaw) << " (" << H.at<double>(0,2) << ", " << H.at<double>(1,2) << std::endl;

		// 	cv::Mat rotation_vector;
		// 	cv::Mat translation_vector;

		// determine_egomotion(old_features,
		// 	new_features,
		// 	matches,
		// 	cam_matrix,
		// 	dist_coeffs,
		// 	rotation_vector,
		// 	translation_vector);

		// 	log("rotation vector", rotation_vector, Logger::LOGLEVEL_INFO);
		//	log("translation_vector", translation_vector, Logger::LOGLEVEL_INFO);

		// {
		// Lock tx_lock(tx_mav_mutex);
		// mavlink_msg_attitude_pack(system_id(),
		// 	component_id,
		// 	&tx_mav_msg,
		// 	get_time_us(),
		// 	rotation_vector.at<double>(0, 1),
		// 	rotation_vector.at<double>(0, 0),
		// 	rotation_vector.at<double>(0, 2),
		// 	0, 0, 0);
		// AppLayer<mavlink_message_t>::send(tx_mav_msg);
		// }

		//cv::flip(new_image, flip_image, 0);

		//Logger::log(name(), ": r,c", flip_image.rows, flip_image.cols, Logger::LOGLEVEL_DEBUG, _loglevel);
		//Logger::log(name(), ": data", flip_image, Logger::LOGLEVEL_DEBUG, _loglevel);

		getFirstOrderOpticalFlow(new_image.cols, new_image.rows);
		// ctrlLateral();

		if(with_out_stream && Core::video_server) {
			visualize(new_image);
			// cv::Mat match_img;
			// cv::drawMatches(old_image, old_features,
			// 	new_image, new_features,
			// 	matches,
			// 	match_img,
			// 	cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
			// 	std::vector<std::vector<char> >(),
			// 	cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
			// );
			//FIXME:
			GstAppSrc *appsrc = GST_APP_SRC( Core::video_server->element("source", -1) );
			if(appsrc) {
				//log(name(), ": appsrc found", Logger::LOGLEVEL_DEBUG);
				//Core::video_server->push(appsrc, match_img.data, match_img.cols, match_img.rows, 24);
				Core::video_server->push(appsrc, new_image.data, new_image.cols, new_image.rows, 8);
				//Core::video_server->push(appsrc, old_image.data, old_image.cols, old_image.rows, 24);
			}
			else
				log(name(), ": no appsrc found", Logger::LOGLEVEL_DEBUG);
		}
		// cv::imshow("oflow",new_image);
		//uint64_t stop_time = get_time_ms();
		//Logger::log(name(), ": needed extract_features", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG, _loglevel);
	}

	void V_OFLOWApp::handle_input(const mavlink_message_t &msg) {
		static int8_t param_id[15];
		int rc2;
		int rc5;

		Logger::log(name(), "got mavlink_message", static_cast<int>(msg.msgid),
								"from", static_cast<int>(msg.sysid),
								static_cast<int>(msg.compid),
								Logger::LOGLEVEL_DEBUG, _loglevel);

		switch(msg.msgid) {

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Hover::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;

		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log(name(), "handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_DEBUG);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log(name(), "handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_DEBUG);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log(name(), "handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_DEBUG);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log(name(), "handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_DEBUG);
						}
					}
				}
			}
			break;

		case MAVLINK_MSG_ID_ACTION:
			if( (mavlink_msg_action_get_target(&msg) == system_id()) ) {
				// 			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
				uint8_t action_id = mavlink_msg_action_get_action(&msg);
				if(action_id == MAV_ACTION_GET_IMAGE) {
					Lock sync_lock(sync_mutex);
					// new image with ACK
					take_new_image = 3;
				}
			}
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			if( (msg.sysid == system_id()) ) {
				Lock sync_lock(sync_mutex);
				mavlink_msg_attitude_decode(&msg, &attitude);
				// take system time for attitude
				attitude.usec = get_time_us();
			}
			break;

		// case MAVLINK_MSG_ID_HUCH_MK_IMU:
		// 	if( (msg.sysid == system_id()) ) {
		// 		Lock sync_lock(sync_mutex);
		// 		mavlink_msg_huch_mk_imu_decode(&msg, &huch_mk_imu);
		// 		// take system time for attitude
		// 		attitude.usec = get_time_us();
		// 	}
		// 	break;

		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			rc2 = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
			rc5 = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
			if (rc2 > 1700 && rc5 > 1700) 
				params["reset_i"] = 2.0;
			break;

		default: break;
		}
	}

	void V_OFLOWApp::visualize(cv::Mat img) {
		cv::Scalar color = CV_RGB(255,255,255);
		const double scaleFactor = 2*img.cols/100;
		const int lineThickness = max(2, img.cols/300);
	
		cv::Point p,q;
		//const double pi = 3.14159265358979323846;
		//double angle;
	
		p.x = 320 >> 1;
		p.y = 240 >> 1;
		q.x = p.x - scaleFactor*of_u;
		q.y = p.y - scaleFactor*of_v;
	
		cv::line(img, p, q, color, lineThickness, CV_AA, 0 );
		cv::line(img, q, q, color, lineThickness*3, CV_AA, 0 );
	}

	void V_OFLOWApp::getFirstOrderOpticalFlow(int w, int h) {
		int8_t sgn_u, sgn_v;
		int16_t diff;
		uint32_t abs_grad_x, abs_grad_y, abs_grad_t, grad_magn;
		int32_t num_u, num_v, sum_u, sum_v;
		int32_t u,v;
		int step = new_image.step1();
		//cout << "steps: " << step << ", " << new_image.step1() << endl;
	
		num_u = num_v = sum_u = sum_v = 0;
		register uint index = 0;
	
		for(register int y = 0; y < h; y++) {
		 	for(register int x = 0; x < w; x++) {
			
				index = y*step+x;
		 		//I_t
				diff = new_image.data[index] - old_image.data[index];
				//cout << "diff: " << new_image.data[index] - old_image.data[index] << endl;
				//cout << "index: " << index << ", " << (int)old_image.data[index] << endl;
				abs_grad_t = ABS(diff);
			
				if(abs_grad_t) {
					//consider signum form I_t 
					//cout << "sgn: " << SGN(diff) << endl;
					sgn_u = sgn_v = SGN(diff);
					//I_x
					//diff = (new_image.data[y*step+x+1] - new_image.data[y*step+x-1] + old_image.data[y*step+x+1] - old_image.data[y*step+x-1]) >> 1;
					diff = (new_image.data[index+1] - new_image.data[index-1]);
					abs_grad_x = ABS(diff);
					sgn_u *= SGN(diff);
					//I_y
					//diff = (new_image.data[(y+1)*step+x] - new_image.data[(y-1)*step+x] + old_image.data[(y+1)*step+x] - old_image.data[(y-1)*step+x]) >> 1;
					diff = (new_image.data[index+step] - new_image.data[index-step]);
					abs_grad_y = ABS(diff);
					sgn_v *= SGN(diff);

					if(abs_grad_x || abs_grad_y) {
						grad_magn = (abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y);
						if( grad_magn > 0) {
							u = (-sgn_u * (((abs_grad_x * abs_grad_t) << 4) / grad_magn));
							v = (-sgn_v * (((abs_grad_y * abs_grad_t) << 4) / grad_magn));
							if(ABS(u) > 0) {
								sum_u += u;
								num_u++;
							}
							if(ABS(v) > 0) {
								sum_v += v;
								num_v++;
							}
						} 
					} 
				} // end if abs_grad_t
				// cout << "x: " << x << endl;
			}
			// cout << "y: " << y << endl;
		}
	
		of_u = (num_u == 0 ? iirFilter(of_u,0.) : iirFilter(of_u,(float)sum_u/num_u)); 
		of_v = (num_v == 0 ? iirFilter(of_v,0.) : iirFilter(of_v,(float)sum_v/num_v)); 
		// of_u = (num_u == 0 ? 0. : (float)sum_u/num_u); 
		// of_v = (num_v == 0 ? 0. : (float)sum_v/num_v); 
	
		//cout << "dx: " << of_u << ", dy:" << of_v << endl;
		// cout << "dy : " << of_v << endl;
	
		// cvCopy(frame1, frame0, NULL);
	}

	void V_OFLOWApp::ctrlLateral() {
		
	}

	void V_OFLOWApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
		if(!data) return;

		//uint64_t start_time = get_time_ms();

		// Logger::log(name(), ": got new video data of size", width, "x", height, Logger::LOGLEVEL_DEBUG, _loglevel);
		if(bpp != 8) {
			log(name(), ": unsupported video data with bpp =", bpp, Logger::LOGLEVEL_WARN);
			return;
		}

		//FIXME: dirty hack
		//throw first frames away
		static int counter = 0;
		if(counter < 10) {
			counter++;
			return;
		}

		// make a matrix header for captured data
		unsigned char *image_data = const_cast<unsigned char*>(data);
		cv::Mat video_data(height, width, CV_8UC1, image_data);

		Lock sync_lock(sync_mutex);
		// if(take_new_image) {
		// 	// make a new reference image
		// 	video_data.copyTo(old_image);
		// 	old_features.clear();
		// 	memcpy(&old_attitude, &attitude, sizeof(mavlink_attitude_t));
		// 	if(take_new_image & (1 << 1)) {
		// 		//TODO: send ACK
		// 	}
		// 	take_new_image = 0;
		// 	log(name(), ": took new image", Logger::LOGLEVEL_DEBUG);
		// } else {

		if (counter == 10) {
			counter++;
			video_data.copyTo(old_image);
			video_data.copyTo(new_image);
			return;
		}


		new_image.copyTo(old_image);
		video_data.copyTo(new_image);

		// log(name(), ": sizeof new_image", sizeof(new_image),
		// 		Logger::LOGLEVEL_DEBUG);
		// log(name(), ": new_image: num chan", new_image.channels(),
		// 		Logger::LOGLEVEL_DEBUG);
		// log(name(), ": new_image: type", new_image.type(),
		// 		Logger::LOGLEVEL_DEBUG);

		//new_features.clear();
		// memcpy(&new_attitude, &attitude, sizeof(mavlink_attitude_t));
		// Logger::log(name(), "attitude of new image",
		// 						rad2deg(new_attitude.roll),
		// 						rad2deg(new_attitude.pitch),
		// 						rad2deg(new_attitude.yaw),
		// 						Logger::LOGLEVEL_DEBUG, _loglevel);
		// }
		new_video_data = true;
		//uint64_t stop_time = get_time_ms();
		// Logger::log(name(), ": needed handle_video_data", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG, _loglevel);
	}

	void V_OFLOWApp::load_calibration_data(const std::string &filename) {
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if( !fs.isOpened() ) {
			log("Can't open calibration data", filename, Logger::LOGLEVEL_DEBUG);
			return;
		}

		fs["camera_matrix"] >> cam_matrix;
		fs["distortion_coefficients"] >> dist_coeffs;
	}

	void V_OFLOWApp::print(std::ostream &os) const {
		AppLayer<mavlink_message_t>::print(os);
	}

	void V_OFLOWApp::run() {
		log(name(), ": running", Logger::LOGLEVEL_DEBUG);

		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		static mavlink_huch_visual_flow_t flow;
		double pitch, roll;
		double imu_pitch, imu_roll, imu_pitchm1, imu_rollm1;
		double imu_pitch_speed, imu_roll_speed;

		if(Core::video_server) {
			int rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), sink_name.c_str());
			Logger::log(name(), ": binded to", sink_name, rc, Logger::LOGLEVEL_DEBUG, _loglevel);
		} else {
			log(name(), ": video server not running", Logger::LOGLEVEL_WARN);
			return;
		}

		Logger::log(name(), ": request data stream extra1 from",
								target_system, ":", target_component,
								Logger::LOGLEVEL_DEBUG, _loglevel);
		request_data_stream(target_system,
												target_component,
												MAV_DATA_STREAM_EXTRA1,
												imu_rate);

		// double leak_f;
		// leak_f = 0.995;
		flow.u = flow.v = flow.u_i = flow.v_i = 0.0f;
		pitch = roll = 0.0;
		imu_pitch = imu_roll = imu_pitchm1 = imu_rollm1 = 0.0;
		imu_pitch_speed = imu_roll_speed = 0.0;

		// doesnt work for some unknown reason
		// log(name(), ": creating cv window", Logger::LOGLEVEL_DEBUG);
		// cv::namedWindow("oflow", CV_WINDOW_NORMAL | CV_GUI_NORMAL);

		while( !interrupted() ) {
			//log(name(), "enter main loop", Logger::LOGLEVEL_DEBUG);

			if(param_request_list) {
				Logger::log("V_OFLOWApp::run: param request", Logger::LOGLEVEL_DEBUG);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					AppLayer<mavlink_message_t>::send(msg);
				}
			}

			if(params["reset_i"] > 0.0) {
				params["reset_i"] = 0.0;
				of_u_i = 0.0;
				of_v_i = 0.0;
			}

			{ Lock sync_lock(sync_mutex);
				if(new_video_data) {
					//NULL;
					extract_features();

					imu_pitch = attitude.pitch;
					imu_roll = attitude.roll;
					imu_pitch_speed = iirFilter(imu_pitch_speed, imu_pitch - imu_pitchm1);
					imu_roll_speed = iirFilter(imu_roll_speed, imu_roll - imu_rollm1);
					imu_pitchm1 = imu_pitch;
					imu_rollm1 = imu_roll;

					// write back into attitude struct
					// attitude.pitchspeed = imu_pitch_speed;
					// attitude.rollspeed = imu_roll_speed;

					if(params["dbg_en"] > 0.0) {
						send_debug(&msg, &dbg, 0, of_u);
						send_debug(&msg, &dbg, 1, of_v);
						send_debug(&msg, &dbg, 2, params["derot_pit_g"] * imu_pitch_speed);
						send_debug(&msg, &dbg, 3, params["derot_rol_g"] * imu_roll_speed);
						// send_debug(&msg, &dbg, 2, imu_pitch_speed);
						// send_debug(&msg, &dbg, 3, imu_roll_speed);
					}

					// derotate flow
					of_u = of_u + (params["derot_pit_g"] * imu_pitch_speed);
					of_v = of_v + (params["derot_rol_g"] * imu_roll_speed);

					if(params["dbg_en"] > 0.0) {
						send_debug(&msg, &dbg, 4, of_u);
						send_debug(&msg, &dbg, 5, of_v);
					}

					// leaky integral / position estimate
					of_u_i = (of_u_i * params["leak_f"]) + (of_u * 0.033); /// one over framerate
					of_v_i = (of_v_i * params["leak_f"]) + (of_v * 0.033);

					// calculate control signals
					// pitch
					pitch = of_u_i * params["pitch_gain"];
					// limit
					if(pitch > params["pitch_limit"])
						pitch = params["pitch_limit"];
					if(pitch < -params["pitch_limit"])
						pitch = -params["pitch_limit"];
					// roll
					roll = of_v_i * params["roll_gain"];
					if(roll > params["roll_limit"])
						roll = params["roll_limit"];
					if(roll < -params["roll_limit"])
						roll = -params["roll_limit"];


					// Logger::log(name(), ": of_u, of_v", of_u, of_v, Logger::LOGLEVEL_DEBUG);
					// Logger::log(name(), ": pitch, roll", huch_mk_imu.xgyro, huch_mk_imu.ygyro, Logger::LOGLEVEL_DEBUG);
					// Logger::log(name(), ": pitch, roll", imu_pitch_speed, imu_roll_speed, Logger::LOGLEVEL_DEBUG);
					DataCenter::set_extctrl_pitch(pitch);
					DataCenter::set_extctrl_roll(roll);

					// flow.u = of_u;
					// flow.v = of_v;
					// flow.u_i = of_u_i;
					// flow.v_i = of_v_i;
					// mavlink_msg_huch_visual_flow_encode(system_id(),
					// 																		component_id,
					// 																		&msg,
					// 																		&flow);

					mavlink_msg_huch_visual_flow_pack(system_id(),
																						component_id,
																						&msg,
																						get_time_us(),
																						of_u,
																						of_v,
																						of_u_i,
																						of_v_i);

					AppLayer<mavlink_message_t>::send(msg);
				}
				new_video_data = false;
			}
			//FIXME: remove usleep
			usleep(1000);
		}

		//unbind from video server
		Core::video_server->release( dynamic_cast<VideoClient*>(this) );
		log(name(), ": stop running", Logger::LOGLEVEL_DEBUG);
	}

	// send debug
	void V_OFLOWApp::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
		dbg->ind = index;
		dbg->value = value;
		mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
		AppLayer<mavlink_message_t>::send(*msg);
	}

	// read config
	void V_OFLOWApp::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("leak_f");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["leak_f"];
		}
		else {
			params["leak_f"] = 0.995;
		}

		iter = args.find("pitch_gain");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_gain"];
		}
		else {
			params["pitch_gain"] = -10.0;
		}

		iter = args.find("roll_gain");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_gain"];
		}
		else {
			params["roll_gain"] = 10.0;
		}

		iter = args.find("pitch_limit");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_limit"];
		}
		else {
			params["pitch_limit"] = 200.0;
		}

		iter = args.find("roll_limit");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_limit"];
		}
		else {
			params["roll_limit"] = 200.0;
		}

		iter = args.find("reset_i");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["reset_i"];
		}
		else {
			params["reset_i"] = 0.0;
		}

		iter = args.find("derot_pit_g");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["derot_pit_g"];
		}
		else {
			params["derot_pit_g"] = 0.0;
		}

		iter = args.find("derot_rol_g");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["derot_rol_g"];
		}
		else {
			params["derot_rol_g"] = 0.0;
		}

		iter = args.find("dbg_en");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["dbg_en"];
		}
		else {
			params["dbg_en"] = 0.0;
		}

	}

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
