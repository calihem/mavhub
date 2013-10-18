#include "v_oflow_car_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#ifdef HAVE_LIBFANN

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

#define SUBFLOWS

using namespace std;
using namespace cpp_pthread;
using namespace cv;
//using namespace hub::slam;

namespace mavhub {

  V_OFLOWCarApp::V_OFLOWCarApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
    AppInterface("v_oflow_car_app", loglevel),
    AppLayer<mavlink_message_t>("v_oflow_car_app", loglevel),
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
    of_u_i_derot(0.0),
    of_v_i_derot(0.0),
    of_yaw(0.0),
    of_alt(0.0),
    of_x(0.0),
    of_y(0.0),
    needToInit(true),
    is_width(320),
    is_height(240),
    param_request_list(0),
    lc_active(0)
    //		cam_matrix(3, 3, CV_32FC1),
    // dist_coeffs( cv::Mat::zeros(4, 1, CV_32FC1) ),
    // feature_detector(60, 3), //threshold, octaves
    // rotation_vector( cv::Mat::zeros(3, 1, CV_64FC1) ),
    // translation_vector( cv::Mat::zeros(3, 1, CV_64FC1))
  {
    // cout << loglevel << endl;
    Logger::log(name(), "Ctor", Logger::LOGLEVEL_DEBUG);
		
    pthread_mutex_init(&sync_mutex, NULL);
    // invalidate attitude
    attitude.time_boot_ms = 0;
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
    assign_variable_from_args(ctl_update_rate);
    assign_variable_from_args(target_system);
    assign_variable_from_args(target_component);
    assign_variable_from_args(imu_rate);
    assign_variable_from_args(en_heartbeat);
		
    // read config
    read_conf(args);

    // neural network config via OpenCV / yaml
    FileStorage fs2("M1_cv.yml", FileStorage::READ);
    fs2["M1_cv"] >> M1;
    // Logger::log(name(), "M1", M1, Logger::LOGLEVEL_DEBUG);
    // Mat u(8, 1, CV_32F);
    u = Mat::zeros(9, 1, CV_64FC1);

    // reservoir config via OpenCV / yaml
    fs2.open("res_M_cv.yml", FileStorage::READ);
    fs2["res_M_cv"] >> res_M;
    Logger::log(name(), "res_M", res_M, Logger::LOGLEVEL_DEBUG);

    // // get calibration data of camera
    // //FIXME: use image dimensions for default camera matrix
    // cam_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, 160.0, 0.0, 1.0, 120.0, 0.0, 0.0, 1.0);
    // string calib_filename;
    // get_value_from_args("calibration_data", calib_filename);
    // if(!calib_filename.empty())
    // 	load_calibration_data(calib_filename);

    Logger::log(name(), "ctl_update_rate", ctl_update_rate, Logger::LOGLEVEL_DEBUG);
    Logger::log(name(), "with_out_stream", with_out_stream, Logger::LOGLEVEL_DEBUG);

    // init execution timer
    exec_tmr = new Exec_Timing(ctl_update_rate);

    // initialize oflow models
    initModels();

    // moving average filter for gyro substraction
    ma_pitch = new MA(1200, 0.);
    ma_roll = new MA(1200, 0.);

    // init sensor_array structure
    sensor_array_x.subid = 0;
    sensor_array_y.subid = 1;
    sensor_array_x.usec = 0;
    sensor_array_y.usec = 0;
    for(int i = 0; i < 16; i++) {
      sensor_array_x.data[i] = 0.;
      sensor_array_y.data[i] = 0.;
    }
  }

  V_OFLOWCarApp::~V_OFLOWCarApp() {}

  int V_OFLOWCarApp::initModels() {
    int width;
    int height;
    // this is for omni case
    width = static_cast<int>(params["unwrap_w"]);
    height = static_cast<int>(params["unwrap_h"]);
    // create different flow algorithms
    // FO, HS, LK, LKPyr, FB, SF
    // experimental: SFA and other dimreduction methods
    // cvCalcOpticalFlowLK, BM, HS
#ifdef SUBFLOWS
    int i;
    for(i = 0; i < 4; i++) {
      ofModels[i]           = new LucasKanade(40, 40);
    }
#else
    ofModels[FIRST_ORDER]  = new FirstOrder(height, width);
    ofModels[HORN_SCHUNCK] = new HornSchunck(height, width);
    ofModels[LK]           = new LucasKanade(height, width);
    ofModels[HORN_SCHUNCK_CV] = new HornSchunckCV(height, width);
    ofModels[BLOCK_MATCHING_CV] = new BlockMatchingCV(height, width);
    ofModels[FARNEBACK] = new Farneback(height, width);
#endif
    return 0;
  }

  void V_OFLOWCarApp::calcESN() {
    printf("dx: %f\n", sensor_array_x.data[0]);
    printf("dy: %f\n", sensor_array_y.data[0]);
  }

  void V_OFLOWCarApp::handle_input(const mavlink_message_t &msg) {
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

            of_algo = (of_algorithm)(params["of_algo"]);
          }
        }
      }
      break;

    case MAVLINK_MSG_ID_HUCH_ACTION:
      Logger::log(name(), "handle_input: action request", (int)mavlink_msg_huch_action_get_target(&msg), system_id(), Logger::LOGLEVEL_DEBUG);
      if( (mavlink_msg_huch_action_get_target(&msg) == system_id()) ) {
        // 			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
        uint8_t action_id = mavlink_msg_huch_action_get_action(&msg);
        // if(action_id == MAV_ACTION_GET_IMAGE) {
        // 	Lock sync_lock(sync_mutex);
        // 	// new image with ACK
        // 	take_new_image = 3;
        // }
        switch(action_id) {
        case ACTION_TOGGLE_LC:
          lc_active = !lc_active;
          params["reset_i"] = (float)lc_active;
          Logger::log(name(), "action done: lc_active, reset_i", (int)lc_active, params["reset_i"], Logger::LOGLEVEL_DEBUG);
        default:
          break;
        }
      }
      break;

    case MAVLINK_MSG_ID_ATTITUDE:
      if( (msg.sysid == system_id()) ) {
        // Lock sync_lock(sync_mutex);
        mavlink_msg_attitude_decode(&msg, &attitude);
        // take system time for attitude
        attitude.time_boot_ms = get_time_ms();
      }
      break;

    case MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC:
      if( (msg.sysid == system_id()) ) {
        mavlink_msg_huch_imu_raw_adc_decode(&msg, &raw_adc_imu);
      }
      break;

    case MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE:
      if( (msg.sysid == system_id()) ) {
        mavlink_msg_huch_ctrl_hover_state_decode(&msg, &hover_state);
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

  void V_OFLOWCarApp::visualize(cv::Mat img) {
    static MHDenseOpticalFlow *oFlow;
    int sectors_x = 2;
    int sectors_y = 2;
    int sector_width  = is_width  / sectors_x;
    int sector_height = is_height / sectors_y;
    CvPoint p,q;
    const CvScalar color = CV_RGB(255,0,0);
    const int linethickness = 2.;
#ifdef SUBFLOWS
    Mat sml_img;
    Size sml_s(160, 120);
    int ii, jj, mi;
    for(ii = 0; ii < sectors_y; ii++) {
      for(jj = 0; jj < sectors_x; jj++) {
        Rect roirect(sector_width*jj+60, sector_height*ii+40, 40, 40);
        // cout << roirect << endl;
        // imageROI = new_image(roirect);
        mi = jj*2+ii;
        rectangle(img, roirect, 
                  Scalar( 0, 255, 255 ), linethickness, 8, 0);
        p.x = sector_width*jj+80; // +(sectorWidth/2);
        p.y = sector_height*ii+60; // (sectorHeight/2);
        q.x = p.x - 10.0 * sensor_array_x.data[(jj*sectors_y*2)+(2*ii)]; // dx;
        q.y = p.y - 10.0 * sensor_array_x.data[(jj*sectors_y*2)+(2*ii+1)]; //dy;
        line(img, p, q, color, linethickness, CV_AA, 0 );
      }
    }
    // optional but nice
    flip(img, img, 0);
    // scale down
    // resize(img, sml_img, sml_s);
    // img = sml_img.clone();
#else
    Logger::log(name(), "visualize of_algo", of_algo, Logger::LOGLEVEL_DEBUG);
    oFlow = (MHDenseOpticalFlow*)&(ofModels[of_algo]->getOpticalFlow());
    // oFlow->visualizeMeanXYf(8, 1, img);
    oFlow->visualizeMeanXYf(2, 2, img);
    Logger::log(name(), "visualize velX", oFlow->getMeanVelX(0, is_width, 0, is_height), Logger::LOGLEVEL_DEBUG);
    Logger::log(name(), "visualize velXf", oFlow->getMeanVelXf(0, is_width, 0, is_height), Logger::LOGLEVEL_DEBUG);
#endif
  }

  void V_OFLOWCarApp::calcFlow() {
    static MHDenseOpticalFlow *oFlow; // FIXME: heighten the scope of the flow pointer
    int sectors_x = 2;
    int sectors_y = 2;
    int sector_width  = is_width  / sectors_x;
    int sector_height = is_height / sectors_y;
    int i, j;
    static mavlink_message_t msg;
    static mavlink_debug_t dbg;

    //FIXME: remove benchmark

    uint64_t start, end;
    start = 0; end = 0;

    cv::Point center(new_image.cols/2, new_image.rows/2);
    Mat imageROI;

    if(new_image.cols == 0)
      return;

    // start = get_time_us(); // bench

#ifdef SUBFLOWS
    int ii, jj, mi;
    for(ii = 0; ii < sectors_y; ii++) {
      for(jj = 0; jj < sectors_x; jj++) {
        Rect roirect(sector_width*jj+60, sector_height*ii+40, 40, 40);
        // cout << roirect << endl;
        imageROI = new_image(roirect);
        mi = jj*2+ii;
        ofModels[mi]->calcOpticalFlow(imageROI);
        oFlow = (MHDenseOpticalFlow*)&(ofModels[mi]->getOpticalFlow());
        sensor_array_x.data[(jj*sectors_y*2)+(2*ii)] = oFlow->getMeanVelXf(1, 39, 1, 39);
        sensor_array_x.data[(jj*sectors_y*2)+(2*ii+1)] = oFlow->getMeanVelYf(1, 39, 1, 39);
      }
    }

    // we have a set of subflow / EMDs in sensor_array, no we want to execute
    // some type of dimensionality reduction / PCA
    // 1) standard PCA
    // 2) reservoir PCA
    // 3) autoencoder network
    // 3) whitening, ICA, SFA, KPCA

    // FIXME: exec pre-trained neural network?
    // put flow into u
    Mat_<double> yM;
    double y;
    Mat_<float>& u1 = (Mat_<float>&)u;
    // u1(0,0)  =sensor_array_x.data[0];
    // u1(1,0) = sensor_array_x.data[1];
    // u1(2,0) = sensor_array_x.data[2];
    // u1(3,0) = sensor_array_x.data[3];
    // u1(4,0) = sensor_array_x.data[4];
    // u1(5,0) = sensor_array_x.data[5];
    // u1(6,0) = sensor_array_x.data[6];
    // u1(7,0) = sensor_array_x.data[7];
    for (ii = 0; ii < 8; ii++) {
      u.at<double>(0,ii) = sensor_array_x.data[ii];
      // Logger::log(name(), "u(0,0), sa[0]", u.at<double>(0,ii), sensor_array_x.data[ii], Logger::LOGLEVEL_DEBUG);
    }
    // cout << "u1 = " << u1 << endl;
    yM = M1 * u;
    // y = yM(0,0);
    // cout << "M1 u = " << yM << endl;
    send_debug(&msg, &dbg, 30, yM.at<double>(0,0));
    send_debug(&msg, &dbg, 31, tanh(yM.at<double>(0,0)));

#else
    if (static_cast<int>(params["cam_type"]) == CAM_TYPE_PLANAR) {
      start = get_time_us();
      switch(of_algo) {
      case FIRST_ORDER:
        // use custom Horn-Schunck implementation
        preprocessImage(new_image);
        ofModels[FIRST_ORDER]->calcOpticalFlow2(new_image);
        // getOF_FirstOrder();
        break;
      case HORN_SCHUNCK:
        preprocessImage(new_image);
        ofModels[HORN_SCHUNCK]->calcOpticalFlow2(new_image);
        // getOF_HS();
        break;
      case LK:
        // use openCV Lucas-Kanade plain
        // log(name(), "pre getOF_LK", Logger::LOGLEVEL_DEBUG);
        // int subflowsx = 2;
        // int subflowsy = 2;
        ofModels[LK]->calcOpticalFlow(new_image);
        // getOF_LK();
        // log(name(), "post getOF_LK", Logger::LOGLEVEL_DEBUG);
        break;
      case LK_PYR:
        // use openCV pyramid Lucas-Kanade
        getOF_LK_Pyr();
        break;
      case HORN_SCHUNCK_CV:
        preprocessImage(new_image);
        ofModels[HORN_SCHUNCK_CV]->calcOpticalFlow2(new_image);
        // getOF_HORN_SCHUNCK_CV();
        break;
      case BLOCK_MATCHING_CV:
        ofModels[BLOCK_MATCHING_CV]->calcOpticalFlow2(new_image);
        // getOF_BLOCK_MATCHING_CV();
        break;
      case SIMPLEFLOW:
        getOF_SF();
        break;
      case FARNEBACK:
        ofModels[FARNEBACK]->calcOpticalFlow(new_image);
        break;
      default:
        break;
      }
    }
    end = get_time_us();
    Logger::log(name(), "calcFlow bench:", of_algo, end - start, Logger::LOGLEVEL_DEBUG);

    oFlow = (MHDenseOpticalFlow*)&(ofModels[of_algo]->getOpticalFlow());
    // for the car, quick test of autoencoder: transmit four sector oflow
    for(i = 0; i < sectors_x; i++) {
      for (j = 0; j < sectors_y; j++)
        {
          sensor_array_x.data[(i*sectors_y*2)+(2*j)] = oFlow->getMeanVelXf(max(sector_width*i, 1),
                                                                           sector_width*(i+1)-1,
                                                                           max(sector_height*j, 1),
                                                                           sector_height*(j+1)-1);
          sensor_array_x.data[(i*sectors_y*2)+(2*j+1)] = oFlow->getMeanVelYf(max(sector_width*i, 1),
                                                                             sector_width*(i+1)-1,
                                                                             max(sector_height*j, 1),
                                                                             sector_height*(j+1)-1);
        }
    }
    
    switch(of_algo) {
    case BLOCK_MATCHING_CV:
      of_u = oFlow->getMeanVelXf(0, (is_width/10), 0, (is_height/10));
      of_v = oFlow->getMeanVelYf(0, is_width/10, 0, is_height/10);
      break;
    default:
      of_u = oFlow->getMeanVelXf(1, is_width-1, 1, is_height-1);
      of_v = oFlow->getMeanVelYf(1, is_width-1, 1, is_height-1);
      break;
    }
#endif

    // end = get_time_us();
    // Logger::log(name(), "bench: ", end - start, Logger::LOGLEVEL_DEBUG);

    if(with_out_stream && Core::video_server) {
      img_display = new_image.clone();
      visualize(img_display);
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
        resize(img_display, img_display, Size(), 0.5, 0.5);
        // Logger::log(name(), "x,y", img_display.cols, img_display.rows, Logger::LOGLEVEL_DEBUG);
        Core::video_server->push(appsrc, img_display.data, img_display.cols, img_display.rows, 8);
        //Core::video_server->push(appsrc, old_image.data, old_image.cols, old_image.rows, 24);
      }
      else
        log(name(), ": no appsrc found", Logger::LOGLEVEL_DEBUG);
    }

    // cv::imshow("oflow",new_image);
    //uint64_t stop_time = get_time_ms();
    //Logger::log(name(), ": needed calcFlow", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG, _loglevel);
  }

  void V_OFLOWCarApp::getOF_FirstOrder() {
    // static MHDenseOpticalFlow* oFlow;
    preprocessImage(new_image);
    ofModels[FIRST_ORDER]->calcOpticalFlow2(new_image);
    // oFlow = (MHDenseOpticalFlow*)&ofModels[FIRST_ORDER]->getOpticalFlow();
    // of_u = oFlow->getMeanVelXf(1, is_width-1, 1, is_height-1);
    // of_v = oFlow->getMeanVelYf(1, is_width-1, 1, is_height-1);
  }

  void V_OFLOWCarApp::getOF_HS() {
    static MHDenseOpticalFlow *oFlow;
    preprocessImage(new_image);
    ofModels[HORN_SCHUNCK]->calcOpticalFlow2(new_image);
    oFlow = (MHDenseOpticalFlow*)&ofModels[HORN_SCHUNCK]->getOpticalFlow();
    of_u = oFlow->getMeanVelXf(1, is_width-1, 1, is_height-1);
    of_v = oFlow->getMeanVelYf(1, is_width-1, 1, is_height-1);
  }

  void V_OFLOWCarApp::getOF_LK() {
    uint64_t start, end;
    start = 0; end = 0;


    start = get_time_us();
    ofModels[LK]->calcOpticalFlow(new_image);
    end = get_time_us();
    Logger::log(name(), "getOF_LK:calcOpticalFlow bench", end - start, Logger::LOGLEVEL_DEBUG);
    // get oFlow object ref
  }

  void V_OFLOWCarApp::getOF_HORN_SCHUNCK_CV() {
    static MHDenseOpticalFlow *oFlow;
    int sectors_x = 2;
    int sectors_y = 2;
    int sector_width  = is_width  / sectors_x;
    int sector_height = is_height / sectors_y;
    int i, j;
    uint64_t start, end;
    start = 0; end = 0;

    // don't need this either for HORN_SCHUNCK_CV
    // preprocessImage(new_image);

    // calculate X and Y flow matrices
    // ofModel->calcOpticalFlow(new_image);
    // Logger::log(name(), "pre calcOpticalFlow", new_image.cols, new_image.rows, Logger::LOGLEVEL_DEBUG);
    start = get_time_us();
    ofModels[HORN_SCHUNCK_CV]->calcOpticalFlow2(new_image);
    end = get_time_us();
    Logger::log(name(), "getOF_HORN_SCHUNCK_CV:calcOpticalFlow bench", end - start, Logger::LOGLEVEL_DEBUG);
    // log(name(), "post calcOpticalFlow", Logger::LOGLEVEL_DEBUG);
    // get oFlow object ref
    //oFlow = (MHDenseOpticalFlow*)&(ofModel->getOpticalFlow());
    oFlow = (MHDenseOpticalFlow*)&(ofModels[HORN_SCHUNCK_CV]->getOpticalFlow());

    // visualize sectorized mean flow
    // this isn't necessary anymore
    // oFlow->visualizeMeanXYf(1, 1, new_image);

    // Logger::log(name(), "HORN_SCHUNCK_CV", Logger::LOGLEVEL_DEBUG);
    // of_u = iirFilter(of_u, oFlow->getMeanVelXf(1, 99, 1, 99));
    // of_v = iirFilter(of_v, oFlow->getMeanVelYf(1, 99, 1, 99));

    // for the car, quick test of autoencoder: transmit four sector oflow
    for(i = 0; i < sectors_x; i++) {
      for (j = 0; j < sectors_y; j++)
        {
          sensor_array_x.data[(i*sectors_y*2)+(2*j)] = oFlow->getMeanVelXf(max(sector_width*i, 1),
                                                                           sector_width*(i+1)-1,
                                                                           max(sector_height*j, 1),
                                                                           sector_height*(j+1)-1);
          sensor_array_x.data[(i*sectors_y*2)+(2*j+1)] = oFlow->getMeanVelYf(max(sector_width*i, 1),
                                                                             sector_width*(i+1)-1,
                                                                             max(sector_height*j, 1),
                                                                             sector_height*(j+1)-1);
        }
    }
    

    of_u = oFlow->getMeanVelXf(1, is_width-1, 1, is_height-1);
    of_v = oFlow->getMeanVelYf(1, is_width-1, 1, is_height-1);

    // of_u = 0.;
    // of_v = 0.;
  }

  void V_OFLOWCarApp::getOF_BLOCK_MATCHING_CV() {
    static MHDenseOpticalFlow *oFlow;
    int sectors_x = 2;
    int sectors_y = 2;
    int sector_width  = is_width  / sectors_x / 10;
    int sector_height = is_height / sectors_y / 10;
    int i, j;
    uint64_t start, end;
    start = 0; end = 0;

    // don't need this either for BLOCK_MATCHING_CV
    // preprocessImage(new_image);

    // calculate X and Y flow matrices
    // ofModel->calcOpticalFlow(new_image);
    // Logger::log(name(), "pre calcOpticalFlow", new_image.cols, new_image.rows, Logger::LOGLEVEL_DEBUG);
    start = get_time_us();
    ofModels[BLOCK_MATCHING_CV]->calcOpticalFlow2(new_image);
    end = get_time_us();
    Logger::log(name(), "getOF_BLOCK_MATCHING_CV:calcOpticalFlow bench", end - start, Logger::LOGLEVEL_DEBUG);
    // log(name(), "post calcOpticalFlow", Logger::LOGLEVEL_DEBUG);
    // get oFlow object ref
    //oFlow = (MHDenseOpticalFlow*)&(ofModel->getOpticalFlow());
    oFlow = (MHDenseOpticalFlow*)&(ofModels[BLOCK_MATCHING_CV]->getOpticalFlow());

    // visualize secotrized mean flow
    // this isn't necessary anymore
    // oFlow->visualizeMeanXYf(1, 1, new_image);

    // Logger::log(name(), "BLOCK_MATCHING_CV", Logger::LOGLEVEL_DEBUG);
    // of_u = iirFilter(of_u, oFlow->getMeanVelXf(1, 99, 1, 99));
    // of_v = iirFilter(of_v, oFlow->getMeanVelYf(1, 99, 1, 99));

    // for the car, quick test of autoencoder: transmit four sector oflow
    for(i = 0; i < sectors_x; i++) {
      for (j = 0; j < sectors_y; j++)
        {
          sensor_array_x.data[(i*sectors_y*2)+(2*j)] = oFlow->getMeanVelXf(max(sector_width*i, 1),
                                                                           sector_width*(i+1)-1,
                                                                           max(sector_height*j, 1),
                                                                           sector_height*(j+1)-1);
          sensor_array_x.data[(i*sectors_y*2)+(2*j+1)] = oFlow->getMeanVelYf(max(sector_width*i, 1),
                                                                             sector_width*(i+1)-1,
                                                                             max(sector_height*j, 1),
                                                                             sector_height*(j+1)-1);
        }
    }
    

    of_u = oFlow->getMeanVelXf(0, is_width/10, 0, is_height/10);
    of_v = oFlow->getMeanVelYf(0, is_width/10, 0, is_height/10);

    // of_u = 0.;
    // of_v = 0.;
  }

  void V_OFLOWCarApp::getOF_SF() {
    // Mat flowl(Size(10,10), CV_32F);
    Mat_<Point2f> flowl;
    calcOpticalFlowSF(old_image, new_image, flowl, 1, 2, 0.1);
    // cout << flow << endl;
    cout << flowl.cols << endl;
    cout << flowl.rows << endl;
    cout << flowl(10,10) << endl;
    // cout << size(flow) << endl;

    // Mat_<Point2f> flowk;
    // Ptr<DenseOpticalFlow> tvl1 = createOptFlow_DualTVL1();
    // const double start = (double)getTickCount();
    // tvl1->calc(old_image, new_image, flowk);
    // const double timeSec = (getTickCount() - start) / getTickFrequency();
    // cout << "calcOpticalFlowDual_TVL1 : " << timeSec << " sec" << endl;
    // // Mat out;
    // // drawOpticalFlow(flow, out);

    // of_u = 
  }

  // getOF_FARNEBACK
  // getOF_LKPyr

  float V_OFLOWCarApp::getMeanVelXf(CvMat &velXf, int x0, int x1, int y0, int y1) const {
    float sum_x = 0;	//sum of vectors
    int16_t num_x = 0;	//number of vectors which are not null

    // signed char vel;
    float vel;

    for(int y=y0;y<y1;y++) {//for every row in range
      for(int x=x0;x<x1;x++) {//for every col in range
        vel = CV_MAT_ELEM(velXf, float, y, x);
        if(vel) {
          num_x++;
          sum_x += vel;
        }
      }
    }

    if(num_x) {
      return sum_x / num_x;
    }

    return 0;
  }

  float V_OFLOWCarApp::getMeanVelYf(CvMat &velYf, int x0, int x1, int y0, int y1) const {
    float sum_y = 0;	//sum of vectors
    int16_t num_y = 0;	//number of vectors which are not null

    float vel;

    for(int y=y0;y<y1;y++) {//for every row in range
      for(int x=x0;x<x1;x++) {//for every col in range
        vel = CV_MAT_ELEM(velYf, float, y, x);
        if(vel) {
          num_y++;
          sum_y += vel;
        }
      }
    }

    if(num_y) {
      return sum_y / num_y;
    }

    return 0;
  }

  void V_OFLOWCarApp::getOF_LK_Pyr() {
    // static MHDenseOpticalFlow *oFlow;
    static vector<Point2f> points[2];
    static const int MAX_COUNT = 500;
    static Point2f pt;
    static bool addRemovePt = false;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    // uint64_t start, end;
    // start = end = 0;

    // preprocessImage(new_image);

    of_u = 0.;
    of_v = 0.;

    if( needToInit )
      {
        // automatic initialization
        goodFeaturesToTrack(new_image, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
        if(points[1].size() < 4)
          return;

#if (CV_MINOR_VERSION > 2)
        cornerSubPix(new_image, points[1], subPixWinSize, Size(-1,-1), termcrit);
#endif
        // addRemovePt = false;
        cout << "goodFeatures Init done: " << points[1].size() << endl;
        needToInit = false;
      }
    else if( !points[0].empty() )
      {
        vector<uchar> status;
        vector<float> err;

        // if(prevGray.empty())
        // 	gray.copyTo(prevGray);

        // start = get_time_us();

#if (CV_MINOR_VERSION >= 2)
        calcOpticalFlowPyrLK(old_image, new_image,
                             points[0], points[1],
                             status, err, winSize,
                             3, termcrit,
                             0, 0);
#else
        calcOpticalFlowPyrLK(old_image, new_image,
                             points[0], points[1],
                             status, err, winSize,
                             3, termcrit,
                             0, 0, 0.001);
#endif

        // end = get_time_us();
        // Logger::log(name(), "calcOFLKPyr bench", end - start, Logger::LOGLEVEL_DEBUG);
				
        // cout << "opoints: " << points[0].size() << ", " << points[0] << endl;
        // cout << "npoints: " << points[1].size() << ", " << points[1] << endl;

        size_t j;
        Point2f dp;
        for(j = 0; j < points[0].size(); j++) {
          dp = points[0].at(j) - points[1].at(j);
          of_u += dp.x;
          of_v += dp.y;
          // cout << "p[0]-" << j << ": " << norm() << endl;
        }

        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
          {
            if( addRemovePt )
              {
                if( norm(pt - points[1][i]) <= 5 )
                  {
                    addRemovePt = false;
                    continue;
                  }
              }

            if( !status[i] )
              continue;

            points[1][k++] = points[1][i];
            // circle(new_image, points[1][i], 3, Scalar(0,255,0), -1, 8);
          }
        points[1].resize(k);
      }

    if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
      {
        vector<Point2f> tmp;
        tmp.push_back(pt);
#if (CV_MINOR_VERSION > 2)
        cornerSubPix(new_image, tmp, winSize, cvSize(-1,-1), termcrit);
#endif
        points[1].push_back(tmp[0]);
        addRemovePt = false;
      }


    // // Logger::log(name(), "LK", Logger::LOGLEVEL_DEBUG);
    // // of_u = iirFilter(of_u, oFlow->getMeanVelXf(1, 99, 1, 99));
    // // of_v = iirFilter(of_v, oFlow->getMeanVelYf(1, 99, 1, 99));

    if(points[1].size() < 4) {
      // automatic initialization
      goodFeaturesToTrack(new_image, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
      if(points[1].size() < 4)
        return;

#if (CV_MINOR_VERSION > 2)
      cornerSubPix(new_image, points[1], subPixWinSize, Size(-1,-1), termcrit);
#endif
      // addRemovePt = false;
      cout << "goodFeatures ReInit done: " << points[1].size() << endl;
    }

    std::swap(points[1], points[0]);
  }

  UnwrapSettings& V_OFLOWCarApp::defaultSettings() {
    return *(new UnwrapSettings(
                                static_cast<int>(params["center_x"]),
                                static_cast<int>(params["center_y"]),
                                static_cast<int>(params["radius_inner"]),
                                static_cast<int>(params["radius_outer"]),
                                static_cast<int>(params["interpolation"]),
                                static_cast<int>(params["scale_x"]),
                                static_cast<int>(params["scale_y"]),
                                static_cast<int>(params["unwrap_w"]),
                                static_cast<int>(params["unwrap_h"])
                                )
             );
  }

  //
  // create a panoramic view 
  //
  void V_OFLOWCarApp::unwrapImage(cv::Mat* inputImg,
                               cv::Mat* outputImg,
                               UnwrapSettings& opt) {
		
    int uwWidth = opt.fw != 0 ? opt.fw : int(ceil((opt.ro * 2.0 * PI) * opt.sx));
		 
    int uwHeight = opt.fh != 0 ? opt.fh : int(ceil((opt.ro - opt.ri) * opt.sy));
	
    // IplImage unwrappedImg = cvCreateImage(cvSize(uwWidth, uwHeight), IPL_DEPTH_8U , inputImg->nChannels);
    // cv::Mat unwrappedImg = cv::Mat::zeros(cv::Size(uwWidth, uwHeight), CV_8U , inputImg->nChannels);
    cv::Mat unwrappedImg = cv::Mat::zeros(cv::Size(uwWidth, uwHeight), CV_8UC1);
	
    // cout << "ImageProcessing::unwrapImage: " << unwrappedImg.type() << endl;
    // cout << opt.cx << ", " << opt.cy << ", " << uwWidth << ", " << uwHeight << endl;

    for (int uwX = 0; uwX < uwWidth; uwX++) {
      for (int uwY = 0; uwY < uwHeight; uwY++) {
        // determine polar coordinates (r,a):
        double r = double(opt.ri) + double(uwY) * double(opt.ro - opt.ri) / double(uwHeight);
        //double r = double(opt.ri) + double(uwHeight - uwY) * double(opt.ro - opt.ri) / double(uwHeight);
			
        //double a = -double(uwX) * 2.0 * PI / double(uwWidth) + PI/2;
        double a = -double(uwX) * 2.0 * PI / double(uwWidth);
        // determine cartesian coordinates (iX,iY): 
        double iX = r * cos(a) + opt.cx;
        double iY = r * sin(a) + opt.cy;

        if ((iX < 1) || (iX > inputImg->cols - 2) || (iY < 1) || (iY > inputImg->rows - 2)) {
          unwrappedImg.at<uint8_t>(uwY,uwX) = 0; //cv::Scalar(); // set all channels to zero
          // cvSet2D(unwrappedImg,uwY,uwX,CvScalar()); // set all channels to zero
          // memset(cvPtr2D(unwrappedImg,uwY,uwX), 0 , inputImg->nChannels);
        } else {
          // if (opt.im == NO_INTERPOLATION) {
          int tmpX = int(iX + 0.5);
          int tmpY = int(iY + 0.5);
          unwrappedImg.at<uint8_t>(uwY,uwX) = inputImg->at<uint8_t>(tmpY,tmpX);
          // } else {
          // 	unwrappedImg.at<uint8_t>(uwY,uwX) = getInterpolation(inputImg, opt.im, iX, iY);
          // }

          // cout << (int)unwrappedImg.at<uint8_t>(uwY, uwX) << endl;

        }
      }
    }

    // inputImg = &unwrappedImg;
    unwrappedImg.copyTo(*outputImg);
    // return unwrappedImg;
  }


  void V_OFLOWCarApp::preprocessImage(cv::Mat img) {
    int smoothSize = static_cast<int>(params["smoothSize"]);
    // fix use of smoothSize argument
    if(smoothSize > 0)
      // cvSmooth(&new_image, &new_image, CV_GAUSSIAN, smoothSize, 0);
      cv::GaussianBlur(img, img,
                       cv::Size(smoothSize,smoothSize),
                       0., 0.);
    // return 0;
  }

  void V_OFLOWCarApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
    if(!data) return;

    uint64_t start_time = get_time_us();

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

    if (counter == 10 || counter == 11) {
      counter++;
      video_data.copyTo(old_image);
      video_data.copyTo(new_image);
      return;
    }

    if (static_cast<int>(params["cam_type"]) == CAM_TYPE_PLANAR) {
      new_image.copyTo(old_image);
      video_data.copyTo(new_image);
    }
    // else if (static_cast<int>(params["cam_type"]) == CAM_TYPE_OMNI) {
    //   new_image.copyTo(old_image);
    //   unwrapImage(&video_data, &new_image, defaultSettings());
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
    uint64_t stop_time = get_time_us();
    // Logger::log(name(), ": needed handle_video_data", stop_time-start_time, "us", Logger::LOGLEVEL_DEBUG, _loglevel);
  }

  // void V_OFLOWCarApp::load_calibration_data(const std::string &filename) {
  // 	cv::FileStorage fs(filename, cv::FileStorage::READ);
  // 	if( !fs.isOpened() ) {
  // 		log("Can't open calibration data", filename, Logger::LOGLEVEL_DEBUG);
  // 		return;
  // 	}

  // 	fs["camera_matrix"] >> cam_matrix;
  // 	fs["distortion_coefficients"] >> dist_coeffs;
  // }

  void V_OFLOWCarApp::print(std::ostream &os) const {
    AppLayer<mavlink_message_t>::print(os);
  }

  void V_OFLOWCarApp::run() {
    Logger::log(name(), ": running", Logger::LOGLEVEL_DEBUG);

    static mavlink_message_t msg;
    static mavlink_debug_t dbg;
    static mavlink_huch_visual_flow_t flow;
    double pitch, roll;
    // double imu_pitch, imu_roll, imu_pitchm1, imu_rollm1;
    float imu_pitch_speed, imu_roll_speed;
    int imu_pitch_ma, imu_roll_ma;
    double imu_pitch_derot, imu_roll_derot;

    // uint64_t dt = 1;
    int wait_time;

    // sleep(1);

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
    flow.u = flow.v = flow.u_i = flow.v_i = 0.0f;
    pitch = roll = 0.0;
    // imu_pitch = imu_roll = imu_pitchm1 = imu_rollm1 = 0.0;
    imu_pitch_speed = imu_roll_speed = 0.0;
    imu_pitch_ma = imu_roll_ma = 0;
    imu_pitch_derot = imu_roll_derot = 0.0;

    // mavlink_msg_heartbeat_pack(system_id(),
    // 													 component_id,
    // 													 &msg,
    // 													 MAV_TYPE_QUADROTOR,
    // 													 MAV_AUTOPILOT_GENERIC,
    // 													 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
    // 													 0,	//custom mode
    // 													 MAV_STATE_ACTIVE);	//system status
    // AppLayer<mavlink_message_t>::send(msg);

    // doesnt work for some unknown reason
    // log(name(), ": creating cv window", Logger::LOGLEVEL_DEBUG);
    // cv::namedWindow("oflow", CV_WINDOW_NORMAL | CV_GUI_NORMAL);

    // usleep(10000);

    log(name(), "enter main loop", Logger::LOGLEVEL_DEBUG);
    uint64_t start, end;
    start = 0; end = 0;
    while( !interrupted() ) {
      start = get_time_us();
      wait_time = exec_tmr->calcSleeptime();
      //Logger::log(name(), "wait_time", wait_time, Logger::LOGLEVEL_DEBUG);
		
      /* wait */
      usleep(wait_time);
      // usleep(10000);

      // get effective dt
      // dt = 
      exec_tmr->updateExecStats();

      // if(param_request_list) {
      //   Logger::log("V_OFLOWCarApp::run: param request", Logger::LOGLEVEL_DEBUG);
      //   param_request_list = 0;

      //   typedef map<string, double>::const_iterator ci;
      //   for(ci p = params.begin(); p!=params.end(); ++p) {
      //     // Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
      //     mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const char*) p->first.data(), p->second, MAVLINK_TYPE_FLOAT, 1, 0);
      //     AppLayer<mavlink_message_t>::send(msg);
      //   }
      // }

      // if(params["reset_i"] > 0.0) {
      //   params["reset_i"] = 0.0;
      //   of_u_i = 0.0;
      //   of_v_i = 0.0;
      // }

      {
        if(new_video_data) {
          // NULL;
          // log(name(), "run, pre flow", Logger::LOGLEVEL_DEBUG);
          {
            uint64_t start, end;
            Lock sync_lock(sync_mutex);
            // log(name(), "pre calcFlow", Logger::LOGLEVEL_DEBUG);
            start = get_time_us();
            calcFlow();
            end = get_time_us();
            // Logger::log(name(), "calcFlow bench", end - start, Logger::LOGLEVEL_DEBUG);
            // log(name(), "post calcFlow", Logger::LOGLEVEL_DEBUG);
            // calcESN();
          }
          // log(name(), "run, post flow", Logger::LOGLEVEL_DEBUG);

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


          // FIXME: make that filter recursive
          // FIXME: adaptation
          imu_pitch_ma = ma_pitch->calc(raw_adc_imu.ygyro);
          imu_roll_ma  = ma_roll->calc(raw_adc_imu.xgyro);

          imu_pitch_speed = static_cast<float>(raw_adc_imu.ygyro);
          imu_roll_speed  = static_cast<float>(raw_adc_imu.xgyro);

          // imu_pitch_derot = params["derot_pit_g"] * (imu_pitch_speed - imu_pitchm1);
          // imu_roll_derot = params["derot_rol_g"] * (imu_roll_speed - imu_rollm1);
          float imu_pitch_ma_scaled = ((float)imu_pitch_ma/1200.);
          float imu_roll_ma_scaled   = ((float)imu_roll_ma/1200.);
          imu_pitch_derot = params["derot_pit_g"] * (imu_pitch_speed - imu_pitch_ma_scaled);
          imu_roll_derot = params["derot_rol_g"] * (imu_roll_speed - imu_roll_ma_scaled);

          // imu_pitchm1 = imu_pitch_speed;
          // imu_rollm1 = imu_roll_speed;

          // write back into attitude struct
          // attitude.pitchspeed = imu_pitch_speed;
          // attitude.rollspeed = imu_roll_speed;

          if(params["dbg_en"] > 0.0) {
            // send_debug(&msg, &dbg, 0, of_u - imu_pitch_derot);
            // send_debug(&msg, &dbg, 1, of_v - imu_roll_derot);
            // send_debug(&msg, &dbg, 2, imu_pitch_derot);
            // send_debug(&msg, &dbg, 3, imu_roll_derot);
            send_debug(&msg, &dbg, 0, lc_active);
            send_debug(&msg, &dbg, 2, imu_pitch_ma_scaled);
            send_debug(&msg, &dbg, 3, imu_roll_ma_scaled);
          }

          // // derotate flow
          // of_u = of_u + (params["derot_pit_g"] * imu_pitch_speed);
          // of_v = of_v + (params["derot_rol_g"] * imu_roll_speed);

          // leaky integral / position estimate
          of_u_i = (of_u_i * params["leak_f"]) + ((of_u - imu_pitch_derot) * 0.033); /// one over framerate
          of_v_i = (of_v_i * params["leak_f"]) + ((of_v - imu_roll_derot)  * 0.033);

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

          // calculate control signals
          // pitch
          if(params["ctl_mode"] == 0.0)
            pitch = params["pitch_bias"] \
              + (of_u_i * params["pitch_gain"]);
          else if(params["ctl_mode"] == 1.0)
            pitch = params["pitch_bias"] + of_u * params["pitch_gain"];
          // limit pitch
          if(pitch > params["pitch_limit"])
            pitch = params["pitch_limit"];
          if(pitch < -params["pitch_limit"])
            pitch = -params["pitch_limit"];
          // roll
          if(params["ctl_mode"] == 0.0)
            roll = params["roll_bias"] \
              + (of_v_i * params["roll_gain"]);
          else if(params["ctl_mode"] == 1.0)
            roll = params["roll_bias"] + of_v * params["roll_gain"];
          // limit roll
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

					
          if (static_cast<int>(params["cam_type"]) == CAM_TYPE_PLANAR) {
            mavlink_msg_huch_visual_flow_pack(system_id(),
                                              component_id,
                                              &msg,
                                              0, // get_time_us(),
                                              of_u,
                                              of_v,
                                              of_u_i,
                                              of_v_i);
          }
          else if (static_cast<int>(params["cam_type"]) == CAM_TYPE_OMNI) {
            mavlink_msg_huch_visual_flow_pack(system_id(),
                                              component_id,
                                              &msg,
                                              0, // get_time_us(),
                                              of_x,
                                              of_y,
                                              of_yaw,
                                              of_alt);
          }

          AppLayer<mavlink_message_t>::send(msg);

          mavlink_msg_huch_sensor_array_encode(system_id(),
                                               component_id,
                                               &msg,
                                               &sensor_array_x);
          AppLayer<mavlink_message_t>::send(msg);
          mavlink_data16_t dd;
          dd.data[0] = 33;
          mavlink_msg_data16_encode(system_id(),
                                    component_id,
                                    &msg,
                                    &dd);
          AppLayer<mavlink_message_t>::send(msg);
          
          // mavlink_msg_huch_sensor_array_encode(system_id(),
          //                                      component_id,
          //                                      &msg,
          //                                      &sensor_array_y);
          // AppLayer<mavlink_message_t>::send(msg);
        }
        new_video_data = false;
      }

      //FIXME: remove usleep
      // usleep(1000);
      end = get_time_us();
      // Logger::log(name(), "bench run(): ", end - start, Logger::LOGLEVEL_DEBUG);
    }

    //unbind from video server
    Core::video_server->release( dynamic_cast<VideoClient*>(this) );
    log(name(), ": stop running", Logger::LOGLEVEL_DEBUG);
  }

  // send debug
  void V_OFLOWCarApp::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
    dbg->ind = index;
    dbg->value = value;
    mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
    AppLayer<mavlink_message_t>::send(*msg);
  }

  // read config
  void V_OFLOWCarApp::read_conf(const map<string, string> args) {
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
    of_algo = (of_algorithm)(params["of_algo"]);

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

    // typedef map<string, double>::const_iterator ci;
    // for(ci p = params.begin(); p!=params.end(); ++p) {
    // 	// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
    // 	// if(!strcmp(p->first.data(), (const char *)param_id)) {
    // 	// params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
    // 	Logger::log("v_oflow_car_app::read_conf", p->first, params[p->first], Logger::LOGLEVEL_INFO);
    // 	// Logger::log(name(), "handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_DEBUG);
    // }

  }
} // namespace mavhub
#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1
#endif // HAVE_LIBFANN
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
