#include "v_camctrl_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

#include <sstream> //istringstream

#define PI 3.14159265358979323846
#define NO_INTERPOLATION        0
#define BILINEAR_INTERPOLATION  1
#define BICUBIC_INTERPOLATION   2

#define ACTION_TOGGLE_LC 2

using namespace std;
using namespace cpp_pthread;
using namespace cv;
//using namespace hub::slam;

namespace mavhub {

	V_CAMCTRLApp::V_CAMCTRLApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
		AppInterface("v_camctrl_app", loglevel),
		AppLayer<mavlink_message_t>("v_camctrl_app", loglevel),
		hub::gstreamer::VideoClient(),
		with_out_stream(false),
		take_new_image(1),
		target_system(7),
		target_component(1),
		imu_rate(10),
		is_width(320),
		is_height(240),
		param_request_list(0),
		histSize(256),
		uniform(true),
		accumulate(false),
		hist_w(512),
		hist_h(400),
		bin_w(0),
		exposure(0)
	{
		// cout << loglevel << endl;
		// Logger::log(name(), "Ctor", Logger::LOGLEVEL_DEBUG);
		
		pthread_mutex_init(&sync_mutex, NULL);

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

		// // get calibration data of camera
		// //FIXME: use image dimensions for default camera matrix
		// cam_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, 160.0, 0.0, 1.0, 120.0, 0.0, 0.0, 1.0);
		// string calib_filename;
		// get_value_from_args("calibration_data", calib_filename);
		// if(!calib_filename.empty())
		// 	load_calibration_data(calib_filename);

		Logger::log(name(), "ctl_update_rate", ctl_update_rate, Logger::LOGLEVEL_DEBUG);

		// init execution timer
		exec_tmr = new Exec_Timing(ctl_update_rate);

		// neural network
		// ann = fann_create_from_file("n_lateral_control.net");

		// setup histogram stuff
		// range[0] = 0; range[1] = 256;
		// histRange = { range };
		bin_w = round( (double) hist_w/histSize);
		histImage = Mat( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );
		img_display = Mat (hist_h, hist_w, CV_8UC1, Scalar(0,0,0));

		// open v4l2 device
		if(cc_v4l2_open())
			Logger::log(name(), "Couldn't open video device", Logger::LOGLEVEL_DEBUG);
		cc_v4l2_query();
		cc_list_controls();
		// printf("unknown control: '%d'\n", cam_ctrls["blub"]);
		printf("Auto Exposure: %d\n", cc_v4l2_get(cam_ctrls["Auto Exposure"]));
		cc_v4l2_set(cam_ctrls["Auto Exposure"], 0);
		printf("Auto Exposure: %d\n", cc_v4l2_get(cam_ctrls["Auto Exposure"]));
		exposure = cc_v4l2_get(cam_ctrls["Exposure"]);
		printf("Exposure: %d\n", exposure);
	}

	V_CAMCTRLApp::~V_CAMCTRLApp() {}

	int V_CAMCTRLApp::cc_v4l2_open() {
		const char devicefile[] = "/dev/video0";
		printf("device: %s\n", devicefile);
		fd = v4l2_open(devicefile, O_RDWR, 0);
    if(fd < 0)
			return 1;
		return 0;
	}

	int V_CAMCTRLApp::cc_v4l2_query() {
    struct v4l2_queryctrl ctrl;
#ifdef V4L2_CTRL_FLAG_NEXT_CTRL
    /* Try the extended control API first */
    ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    if(0 == v4l2_ioctl (fd, VIDIOC_QUERYCTRL, &ctrl)) {
			do {
				// mw->add_control(ctrl, fd, grid, gridLayout);
				// printf();
				printf("add_control: ctrl.id: %d, ctrl.name = %s\n", ctrl.id, ctrl.name);
				cam_ctrls[(char *)ctrl.name] = ctrl.id;
				//printf("add_control: ctrl.id: %d, ctrl.name = %s\n", cam_ctrls[(char *)ctrl.name], (char *)ctrl.name);
				ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
			} while(0 == v4l2_ioctl (fd, VIDIOC_QUERYCTRL, &ctrl));
    } else
#endif
			{
				/* Fall back on the standard API */
				/* Check all the standard controls */
				for(int i=V4L2_CID_BASE; i<V4L2_CID_LASTP1; i++) {
					ctrl.id = i;
					if(v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0) {
						// mw->add_control(ctrl, fd, grid, gridLayout);
						printf("add_control x: ctrl.id: %d, ctrl.name = %s\n", ctrl.id, ctrl.name);
						cam_ctrls[(char *)ctrl.name] = ctrl.id;
						printf("add_control: ctrl.id: %d, ctrl.name = %s\n", cam_ctrls[(char *)ctrl.name], (char *)ctrl.name);
					}
				}

				/* Check any custom controls */
				for(int i=V4L2_CID_PRIVATE_BASE; ; i++) {
					ctrl.id = i;
					if(v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0) {
						// mw->add_control(ctrl, fd, grid, gridLayout);
						printf("add_control priv: ctrl.id: %d, ctrl.name = %s\n", ctrl.id, ctrl.name);
						cam_ctrls[(char *)ctrl.name] = ctrl.id;
					} else {
						break;
					}
				}
			}
		return 0;
	}

	void V_CAMCTRLApp::cc_list_controls() {
		typedef map<string, int>::const_iterator ci;
		for(ci p = cam_ctrls.begin(); p!=cam_ctrls.end(); ++p) {
			// printf("blub\n");
			// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
			// if(!strcmp(p->first.data(), (const char *)param_id)) {
				// params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
				Logger::log(name(), "cam_ctrls", p->first, cam_ctrls[p->first], Logger::LOGLEVEL_DEBUG);
				// }
		}
	}

	void V_CAMCTRLApp::cc_v4l2_set(int id, int value)
	{
    struct v4l2_control c;
		if(id == 0) return;
    c.id = id;
    c.value = value;
		printf("updateHardware: c.id = %d, c.value = %d\n", c.id, c.value);
    if(v4l2_ioctl(fd, VIDIOC_S_CTRL, &c) == -1) {
			// QString msg;
			// msg.sprintf("Unable to set %s\n%s", name, strerror(errno));
			// QMessageBox::warning(this, "Unable to set control", msg, "OK");
			Logger::log(name(), "Couldn't set v4l2 control", id, Logger::LOGLEVEL_DEBUG);
    }
    // updateStatus();
	}   

	int V_CAMCTRLApp::cc_v4l2_get(int id)
	{   
    struct v4l2_control c;
		if(id == 0) return -1;
    c.id = id;
		printf("updateHardware: c.id = %d\n", c.id);
    if(v4l2_ioctl(fd, VIDIOC_G_CTRL, &c) == -1) {
			// QString msg;
			// msg.sprintf("Unable to get %s\n%s", name,
			// 						strerror(errno));
			// QMessageBox::warning(this, "Unable to get control", msg, "OK");
			Logger::log(name(), "Couldn't get v4l2 control", id, Logger::LOGLEVEL_DEBUG);
		}
		return c.value;
	}

	void V_CAMCTRLApp::calcCamCtrl() {
		//FIXME: remove benchmark
		//uint64_t start_time = get_time_ms();
		// cv::Mat flip_image;

		// uint64_t start, end;
		// start = 0; end = 0;

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

		// start = get_time_us(); // bench

		// end = get_time_us();
		// Logger::log(name(), "bench: ", end - start, Logger::LOGLEVEL_DEBUG);

		

		if(with_out_stream && Core::video_server) {
			// img_display = new_image.clone();
			visualize(new_image, img_display);
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
			GstAppSrc *appsrc = GST_APP_SRC( Core::video_server->element("histo", -1) );
			if(appsrc) {
				//log(name(), ": appsrc found", Logger::LOGLEVEL_DEBUG);
				//Core::video_server->push(appsrc, match_img.data, match_img.cols, match_img.rows, 24);
				Core::video_server->push(appsrc, img_display.data, img_display.cols, img_display.rows, 8);
				//Core::video_server->push(appsrc, old_image.data, old_image.cols, old_image.rows, 24);
			}
			else
				log(name(), ": no appsrc found", Logger::LOGLEVEL_DEBUG);

			
			// appsrc = GST_APP_SRC( Core::video_server->element("source", -1) );
			// if(appsrc) {
			// 	//log(name(), ": appsrc found", Logger::LOGLEVEL_DEBUG);
			// 	//Core::video_server->push(appsrc, match_img.data, match_img.cols, match_img.rows, 24);
			// 	Core::video_server->push(appsrc, new_image.data, new_image.cols, new_image.rows, 8);
			// 	//Core::video_server->push(appsrc, old_image.data, old_image.cols, old_image.rows, 24);
			// }
			// else
			// 	log(name(), ": no appsrc found", Logger::LOGLEVEL_DEBUG);
		}

		// cv::imshow("camctrl",new_image);
		//uint64_t stop_time = get_time_ms();
		//Logger::log(name(), ": needed calcFlow", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG, _loglevel);
	}

	void V_CAMCTRLApp::handle_input(const mavlink_message_t &msg) {
		static char param_id[16];
		int rc2;
		int rc5;

		// Logger::log(name(), "got mavlink_message", static_cast<int>(msg.msgid),
		// 						"from", static_cast<int>(msg.sysid),
		// 						static_cast<int>(msg.compid),
		// 						Logger::LOGLEVEL_DEBUG, _loglevel);

		switch(msg.msgid) {

		// case MAVLINK_MSG_ID_HEARTBEAT:
		// 	Logger::log(name(), "got mavlink heartbeat: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_DEBUG);
		// 	break;

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

						// algo = (of_algorithm)(params["of_algo"]);
					}
				}
			}
			break;

		case MAVLINK_MSG_ID_HUCH_ACTION:
			Logger::log(name(), "handle_input: action request", (int)mavlink_msg_huch_action_get_target(&msg), system_id(), Logger::LOGLEVEL_DEBUG);
			if( (mavlink_msg_huch_action_get_target(&msg) == system_id()) ) {
				// 			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
				// uint8_t action_id = mavlink_msg_huch_action_get_action(&msg);
				// if(action_id == MAV_ACTION_GET_IMAGE) {
				// 	Lock sync_lock(sync_mutex);
				// 	// new image with ACK
				// 	take_new_image = 3;
				// }
				// switch(action_id) {
				// case ACTION_TOGGLE_LC:
				// 	// lc_active = !lc_active;
				// 	// params["reset_i"] = (float)lc_active;
				// 	Logger::log(name(), "action done: lc_active, reset_i", (int)lc_active, params["reset_i"], Logger::LOGLEVEL_DEBUG);
				// default:
				// 	break;
				// }
			}
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			if( (msg.sysid == system_id()) ) {
				Lock sync_lock(sync_mutex);
				// mavlink_msg_attitude_decode(&msg, &attitude);
				// take system time for attitude
				//attitude.time_boot_ms = get_time_ms();
			}
			break;

		case MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC:
			if( (msg.sysid == system_id()) ) {
				// mavlink_msg_huch_imu_raw_adc_decode(&msg, &raw_adc_imu);
			}
			break;

		case MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE:
			if( (msg.sysid == system_id()) ) {
				// mavlink_msg_huch_ctrl_hover_state_decode(&msg, &hover_state);
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

	void V_CAMCTRLApp::visualize(cv::Mat img_src, cv::Mat img) {
		cv::Scalar color = CV_RGB(255,255,255);
		const double scaleFactor = 2*img.cols/100;
		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		//const int lineThickness = max(2, img.cols/300);
		Mat cap_hist;
		// cv::Point p,q;
		//const double pi = 3.14159265358979323846;
		//double angle;
		float mean, var, std, skew, skew_scale;
		int i;

		/// Set the ranges ( for B,G,R) )
		float lrange[] = { 0, 256 } ;
		const float* lhistRange = { lrange };

		int N, Ntenth;
		int cnt_lower = 0, cnt_upper = 0;
		// int exposure;
		// p.x = (img.cols) >> 1;
		// p.y = (img.rows) >> 1;
		// q.x = p.x - scaleFactor*0.0;
		// q.y = p.y - scaleFactor*0.0;

		calcHist(&img_src, 1, 0, Mat(), cap_hist, 1, &histSize, &lhistRange, uniform, accumulate);

		N = is_width * is_height;
		Ntenth = (int)(N * 0.01);
		mean = 0.;
		var = 0.;
		std = 0.;
		skew = 0.;
		for (i = 0; i < histSize; i++) {
			mean += i * cap_hist.at<float>(i);
		}
		mean /= N;
		for (i = 0; i < histSize; i++) {
			var += powf(i - mean, 2) * cap_hist.at<float>(i);
		}
		std = sqrtf(var/(N-1));

		skew_scale = (float)N / ((N-1)*(N-2));
		for (i = 0; i < histSize; i++) {
			skew += powf((i - mean)/std, 3) * cap_hist.at<float>(i);
		}
		skew *= skew_scale;

		// send statistics
		send_debug(&msg, &dbg, 0, mean);
		send_debug(&msg, &dbg, 1, std);
		send_debug(&msg, &dbg, 2, skew);


		// set camera
		// check lower
		for(i = 0; i < 10; i++) {
			cnt_lower += (int)cap_hist.at<float>(i);
		}
		// check upper
		for(i = histSize-1; i > histSize-10; i--) {
			cnt_upper += (int)cap_hist.at<float>(i);
		}

		if(cnt_lower > Ntenth)
			exposure++;
		if(cnt_upper > Ntenth)
			exposure--;

		cc_v4l2_set(cam_ctrls["Exposure"], exposure);
		exposure = cc_v4l2_get(cam_ctrls["Exposure"]);

		Logger::log(name(), "lower:", cnt_lower, "upper:", cnt_upper, Logger::LOGLEVEL_DEBUG);
		Logger::log(name(), "exp:", exposure, Logger::LOGLEVEL_DEBUG);
		
		/// Normalize the result to [ 0, histImage.rows ]
		histImage = 0.;
		// normalize(cap_hist, cap_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

		/// Draw for each channel
		for(i = 1; i < histSize; i++ )
			{
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(cap_hist.at<float>(i-1)) ) ,
							Point( bin_w*(i), hist_h - cvRound(cap_hist.at<float>(i)) ),
							Scalar( 255, 0, 0), 1, 8, 0  );
			}	
		
		histImage.copyTo(img);
		// // cv::line(img, p, q, color, lineThickness, CV_AA, 0 );
		// // cv::line(img, q, q, color, lineThickness*3, CV_AA, 0 );
	}

	void V_CAMCTRLApp::preprocessImage(cv::Mat img) {
		int smoothSize = static_cast<int>(params["smoothSize"]);
		// fix use of smoothSize argument
		if(smoothSize > 0)
			// cvSmooth(&new_image, &new_image, CV_GAUSSIAN, smoothSize, 0);
			cv::GaussianBlur(img, img,
											 cv::Size(smoothSize,smoothSize),
											 0., 0.);
		// return 0;
	}

	void V_CAMCTRLApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
		if(!data) return;

		// uint64_t start_time = get_time_ms();

		// Logger::log(name(), ": got new video data of size", width, "x", height, Logger::LOGLEVEL_DEBUG);

		if(bpp != 8) {
			log(name(), ": unsupported video data with bpp =", bpp, Logger::LOGLEVEL_WARN);
			return;
		}

		//FIXME: dirty hack
		//throw first frames away
		static int counter = 0;
		if(counter < 10) {
			counter++;
			if(width != is_width) is_width = width;
			if(height != is_height) is_height = height;
			return;
		}

		// make a matrix header for captured data
		unsigned char *image_data = const_cast<unsigned char*>(data);
		// // set ROI, no, we do cropping in gstreamer
		// cv::Mat video_data_l(height, width, CV_8UC1, image_data);
		// cv::Mat video_data(video_data_l, cv::Rect(80, 60, 160, 120));
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

		// cvSetImageROI(video_data, cv::Rect(80, 60, 160, 120));

		if (counter == 10) {
			counter++;
			video_data.copyTo(old_image);
			video_data.copyTo(new_image);
			return;
		}

		// if (static_cast<int>(params["cam_type"]) == CAM_TYPE_PLANAR) {
		new_image.copyTo(old_image);
		video_data.copyTo(new_image);
		// }
		// else if (static_cast<int>(params["cam_type"]) == CAM_TYPE_OMNI) {
		// 	new_image.copyTo(old_image);
		// 	unwrapImage(&video_data, &new_image, defaultSettings());
		// }

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

	// void V_CAMCTRLApp::load_calibration_data(const std::string &filename) {
	// 	cv::FileStorage fs(filename, cv::FileStorage::READ);
	// 	if( !fs.isOpened() ) {
	// 		log("Can't open calibration data", filename, Logger::LOGLEVEL_DEBUG);
	// 		return;
	// 	}

	// 	fs["camera_matrix"] >> cam_matrix;
	// 	fs["distortion_coefficients"] >> dist_coeffs;
	// }

	void V_CAMCTRLApp::print(std::ostream &os) const {
		AppLayer<mavlink_message_t>::print(os);
	}

	void V_CAMCTRLApp::run() {
		Logger::log(name(), ": running", Logger::LOGLEVEL_DEBUG);

		static mavlink_message_t msg;
		// static mavlink_debug_t dbg;
		// static mavlink_huch_visual_flow_t flow;
		// double pitch, roll;
		// double imu_pitch, imu_roll, imu_pitchm1, imu_rollm1;
		// float imu_pitch_speed, imu_roll_speed;
		// int imu_pitch_ma, imu_roll_ma;
		// double imu_pitch_derot, imu_roll_derot;

		uint64_t dt = 0;
		int wait_time;

		if(Core::video_server) {
			int rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), sink_name.c_str());
			Logger::log(name(), ": bound to", sink_name, rc, Logger::LOGLEVEL_DEBUG, _loglevel);
		} else {
			log(name(), ": video server not running", Logger::LOGLEVEL_WARN);
			return;
		}

		// Logger::log(name(), ": request data stream extra1 from",
		// 						target_system, ":", target_component,
		// 						Logger::LOGLEVEL_DEBUG, _loglevel);
		// request_data_stream(target_system,
		// 										target_component,
		// 										MAV_DATA_STREAM_EXTRA1,
		// 										imu_rate);

		// double leak_f;
		// leak_f = 0.995;
		// flow.u = flow.v = flow.u_i = flow.v_i = 0.0f;
		// pitch = roll = 0.0;
		// imu_pitch = imu_roll = imu_pitchm1 = imu_rollm1 = 0.0;
		// imu_pitch_speed = imu_roll_speed = 0.0;
		// imu_pitch_ma = imu_roll_ma = 0;
		// imu_pitch_derot = imu_roll_derot = 0.0;

		mavlink_msg_heartbeat_pack(system_id(),
															 component_id,
															 &msg,
															 MAV_TYPE_QUADROTOR,
															 MAV_AUTOPILOT_GENERIC,
															 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
															 0,	//custom mode
															 MAV_STATE_ACTIVE);	//system status
		AppLayer<mavlink_message_t>::send(msg);

		// doesnt work for some unknown reason
		// log(name(), ": creating cv window", Logger::LOGLEVEL_DEBUG);
		// cv::namedWindow("oflow", CV_WINDOW_NORMAL | CV_GUI_NORMAL);
		uint64_t start_time = get_time_ms();
		uint64_t stop_time = get_time_ms();


		while( !interrupted() ) {
			//log(name(), "enter main loop", Logger::LOGLEVEL_DEBUG);

			wait_time = exec_tmr->calcSleeptime();
			// Logger::log(name(), "wait_time", wait_time, Logger::LOGLEVEL_DEBUG);
		
			/* wait */
			usleep(wait_time);
			// usleep(10000);

			// get effective dt
			dt = exec_tmr->updateExecStats();

			if(param_request_list) {
				Logger::log("V_CAMCTRLApp::run: param request", Logger::LOGLEVEL_DEBUG);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const char*) p->first.data(), p->second, MAVLINK_TYPE_FLOAT, 1, 0);
					AppLayer<mavlink_message_t>::send(msg);
				}
			}

			if(params["reset_i"] > 0.0) {
				params["reset_i"] = 0.0;
				// of_u_i = 0.0;
				// of_v_i = 0.0;
			}

			{ Lock sync_lock(sync_mutex);
				if(new_video_data) {
					//NULL;
					// // actual frame rate
					// stop_time = get_time_ms();
					// printf("del: %d\n", stop_time - start_time);
					// start_time = stop_time;

					calcCamCtrl();

					// differentiate from angle: bad
					// imu_pitch = attitude.pitch;
					// imu_roll = attitude.roll;
					// imu_pitch_speed = iirFilter(imu_pitch_speed, imu_pitch - imu_pitchm1);
					// imu_roll_speed = iirFilter(imu_roll_speed, imu_roll - imu_rollm1);
					// imu_pitchm1 = imu_pitch;
					// imu_rollm1 = imu_roll;

					// imu_pitchm1 = imu_pitch;
					// imu_rollm1  = imu_roll;
					// imu_pitch = static_cast<double>(raw_adc_imu.ygyro);
					// imu_roll  = static_cast<double>(raw_adc_imu.xgyro);
					// imu_roll_speed =  imu_rollm1  - imu_roll;
					// imu_pitch_speed = imu_pitchm1 - imu_pitch;

					// use raw_adc_imu
					// imu_roll_speed = static_cast<double>(raw_adc_imu.xgyro - 497);
					// imu_pitch_speed = static_cast<double>(raw_adc_imu.ygyro - 508);
					// imu_pitch_speed = static_cast<double>(raw_adc_imu.ygyro) - params["derot_pit_b"];
					// imu_roll_speed  = static_cast<double>(raw_adc_imu.xgyro) - params["derot_rol_b"];


					// imu_pitch_ma = ma_pitch->calc(raw_adc_imu.ygyro);
					// imu_roll_ma  = ma_roll->calc(raw_adc_imu.xgyro);

					// imu_pitch_speed = static_cast<float>(raw_adc_imu.ygyro);
					// imu_roll_speed  = static_cast<float>(raw_adc_imu.xgyro);

					// imu_pitch_derot = params["derot_pit_g"] * (imu_pitch_speed - imu_pitchm1);
					// imu_roll_derot = params["derot_rol_g"] * (imu_roll_speed - imu_rollm1);
					// imu_pitch_derot = params["derot_pit_g"] * (imu_pitch_speed - ((float)imu_pitch_ma/1200.));
					// imu_roll_derot = params["derot_rol_g"] * (imu_roll_speed - ((float)imu_roll_ma/1200.));

					// imu_pitchm1 = imu_pitch_speed;
					// imu_rollm1 = imu_roll_speed;

					// write back into attitude struct
					// attitude.pitchspeed = imu_pitch_speed;
					// attitude.rollspeed = imu_roll_speed;

					// if(params["dbg_en"] > 0.0) {
						// send_debug(&msg, &dbg, 0, of_u - imu_pitch_derot);
						// send_debug(&msg, &dbg, 1, of_v - imu_roll_derot);
						// send_debug(&msg, &dbg, 2, imu_pitch_derot);
						// send_debug(&msg, &dbg, 3, imu_roll_derot);
						// send_debug(&msg, &dbg, 0, lc_active);
						// send_debug(&msg, &dbg, 2, imu_pitch_speed);
						// send_debug(&msg, &dbg, 3, imu_roll_speed);
					// }

					// // derotate flow
					// of_u = of_u + (params["derot_pit_g"] * imu_pitch_speed);
					// of_v = of_v + (params["derot_rol_g"] * imu_roll_speed);

					// leaky integral / position estimate
					// of_u_i = (of_u_i * params["leak_f"]) + ((of_u - imu_pitch_derot) * 0.033); /// one over framerate
					// of_v_i = (of_v_i * params["leak_f"]) + ((of_v - imu_roll_derot)  * 0.033);

					// // derotate integral with IMU integral
					// of_u_i_derot = params["derot_pit_g"] * attitude.pitch;
					// of_v_i_derot = params["derot_rol_g"] * attitude.roll;

					// of_u_i_derot = params["derot_pit_g"] * attitude.pitch;
					// of_v_i_derot = params["derot_rol_g"] * attitude.roll;

					// if(params["dbg_en"] > 0.0) {
					// 	send_debug(&msg, &dbg, 4, of_u_i);
					// 	send_debug(&msg, &dbg, 5, of_v_i);
					// 	send_debug(&msg, &dbg, 6, of_u_i_derot);
					// 	send_debug(&msg, &dbg, 7, of_v_i_derot);
					// }

					// // calculate control signals
					// // pitch
					// if(params["ctl_mode"] == 0.0)
					// 	pitch = params["pitch_bias"] + (of_u_i * params["pitch_gain"]);
					// else if(params["ctl_mode"] == 1.0)
					// 	pitch = params["pitch_bias"] + of_u * params["pitch_gain"];
					// // limit pitch
					// if(pitch > params["pitch_limit"])
					// 	pitch = params["pitch_limit"];
					// if(pitch < -params["pitch_limit"])
					// 	pitch = -params["pitch_limit"];
					// // roll
					// if(params["ctl_mode"] == 0.0)
					// 	roll = params["roll_bias"] + (of_v_i * params["roll_gain"]);
					// else if(params["ctl_mode"] == 1.0)
					// 	roll = params["roll_bias"] + of_v * params["roll_gain"];
					// // limit roll
					// if(roll > params["roll_limit"])
					// 	roll = params["roll_limit"];
					// if(roll < -params["roll_limit"])
					// 	roll = -params["roll_limit"];

					// Logger::log(name(), ": of_u, of_v", of_u, of_v, Logger::LOGLEVEL_DEBUG);
					// Logger::log(name(), ": pitch, roll", huch_mk_imu.xgyro, huch_mk_imu.ygyro, Logger::LOGLEVEL_DEBUG);
					// Logger::log(name(), ": pitch, roll", imu_pitch_speed, imu_roll_speed, Logger::LOGLEVEL_DEBUG);
					// DataCenter::set_extctrl_pitch(pitch);
					// DataCenter::set_extctrl_roll(roll);

					// flow.u = of_u;
					// flow.v = of_v;
					// flow.u_i = of_u_i;
					// flow.v_i = of_v_i;
					// mavlink_msg_huch_visual_flow_encode(system_id(),
					// 																		component_id,
					// 																		&msg,
					// 																		&flow);

					
					// if (static_cast<int>(params["cam_type"]) == CAM_TYPE_PLANAR) {
					// 	mavlink_msg_huch_visual_flow_pack(system_id(),
					// 																		component_id,
					// 																		&msg,
					// 																		0, // get_time_us(),
					// 																		of_u,
					// 																		of_v,
					// 																		of_u_i,
					// 																		of_v_i);
					// }
					// else if (static_cast<int>(params["cam_type"]) == CAM_TYPE_OMNI) {
					// 	mavlink_msg_huch_visual_flow_pack(system_id(),
					// 																		component_id,
					// 																		&msg,
					// 																		0, // get_time_us(),
					// 																		of_x,
					// 																		of_y,
					// 																		of_yaw,
					// 																		of_alt);
					// }

					// AppLayer<mavlink_message_t>::send(msg);
				}
				new_video_data = false;
			}
			//FIXME: remove usleep
			// usleep(1000);
		}

		//unbind from video server
		Core::video_server->release( dynamic_cast<VideoClient*>(this) );
		log(name(), ": stop running", Logger::LOGLEVEL_DEBUG);
	}

	// send debug
	void V_CAMCTRLApp::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
		dbg->ind = index;
		dbg->value = value;
		mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
		AppLayer<mavlink_message_t>::send(*msg);
	}

	// read config
	void V_CAMCTRLApp::read_conf(const map<string, string> args) {
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

		iter = args.find("pitch_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_bias"];
		}
		else {
			params["pitch_bias"] = 0.0;
		}

		iter = args.find("pitch_limit");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_limit"];
		}
		else {
			params["pitch_limit"] = 200.0;
		}

		iter = args.find("roll_gain");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_gain"];
		}
		else {
			params["roll_gain"] = 10.0;
		}

		iter = args.find("roll_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_bias"];
		}
		else {
			params["roll_bias"] = 0.0;
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

		iter = args.find("ctl_mode");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_en"];
		}
		else {
			params["ctl_en"] = 0.0;
		}

		// run method update rate
		iter = args.find("ctl_update_rate");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> ctl_update_rate;
		}
		else
			ctl_update_rate = 100;

		// camera type
		iter = args.find("cam_type");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["cam_type"];
		}
		else {
			params["cam_type"] = 0.0;
		}

		// optical flow algorithm
		iter = args.find("of_algo");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["of_algo"];
		}
		else {
			params["of_algo"] = 0.0;
		}

		// smoothSize
		iter = args.find("smoothSize");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["smoothSize"];
		}
		else {
			params["smoothSize"] = 9.0;
		}

		////////////////////////////////////////////////////////////
		/// omni params
		// center_x
		iter = args.find("center_x");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["center_x"];
		}
		else {
			params["center_x"] = 0.0;
		}

		// center_y
		iter = args.find("center_y");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["center_y"];
		}
		else {
			params["center_y"] = 0.0;
		}

		// radius_inner
		iter = args.find("radius_inner");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["radius_inner"];
		}
		else {
			params["radius_inner"] = 0.0;
		}

		// radius_outer
		iter = args.find("radius_outer");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["radius_outer"];
		}
		else {
			params["radius_outer"] = 0.0;
		}

		// interpolation
		iter = args.find("interpolation");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["interpolation"];
		}
		else {
			params["interpolation"] = 0.0;
		}

		// scale_x
		iter = args.find("scale_x");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["scale_x"];
		}
		else {
			params["scale_x"] = 0.0;
		}

		// scale_y
		iter = args.find("scale_y");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["scale_y"];
		}
		else {
			params["scale_y"] = 0.0;
		}

		// unwrap_w
		iter = args.find("unwrap_w");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["unwrap_w"];
		}
		else {
			params["unwrap_w"] = 0.0;
		}

		// unwrap_h
		iter = args.find("unwrap_h");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["unwrap_h"];
		}
		else {
			params["unwrap_h"] = 0.0;
		}

		iter = args.find("derot_pit_b");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["derot_pit_b"];
		}
		else {
			params["derot_pit_b"] = 0.0;
		}

		iter = args.find("derot_rol_b");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["derot_rol_b"];
		}

		else {
			params["derot_rol_b"] = 0.0;
		}

		typedef map<string, double>::const_iterator ci;
		for(ci p = params.begin(); p!=params.end(); ++p) {
			// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
			// if(!strcmp(p->first.data(), (const char *)param_id)) {
			// params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
			Logger::log("v_camctrl_app::read_conf", p->first, params[p->first], Logger::LOGLEVEL_INFO);
			// Logger::log(name(), "handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_DEBUG);
		}

	}
} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
