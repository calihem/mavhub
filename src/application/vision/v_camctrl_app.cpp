#include "v_camctrl_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)
#if (defined(HAVE_LIBV4L2))

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
#ifdef HAVE_LIBOSCPACK
  V_CAMCTRLOscPacketListener::V_CAMCTRLOscPacketListener(const V_CAMCTRLApp &app) :
    osc::OscPacketListener()
  {
  }

  void V_CAMCTRLOscPacketListener::ProcessMessage(const osc::ReceivedMessage& m,
                                                  const IpEndpointName& remoteEndpoint)
  {
    Logger::log("processMessage", Logger::LOGLEVEL_DEBUG);

    try{
      // example of parsing single messages. osc::OsckPacketListener
      // handles the bundle traversal.
            
      if( strcmp( m.AddressPattern(), "/mavhub/camctrl" ) == 0 ){
        // example #1 -- argument stream interface
        osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
        // bool a1;
        // osc::int32 a2;
        // float a3;
        // const char *a4;
        osc::int32 e, c, g;
        args >> e >> c >> g >> osc::EndMessage;
                
        Logger::log("received '/mavhub/camctrl' message with arguments: ", e, c, g, Logger::LOGLEVEL_DEBUG);
        // std::cout << "received '/mavhub/camctrl' message with arguments: "
        // 					<< e << " " << c << " " << g << "\n";
                
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

#endif // HAVE_LIBOSCPACK

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
    histSize(8),
    uniform(true),
    accumulate(false),
    hist_w(512),
    hist_h(400),
    bin_w(0),
    mean(0.0),
                                                 var(0.0),
                                                 std(0.0),
                                                 skew(0.0),
                                                 skew_scale(1.0),
                                                 N(1),
                                                 Ntenth(1),
                                                 exposure(20),
                                                 contrast(32)
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
    // assign_variable_from_args(ctl_update_rate)
    assign_variable_from_args(device)
    Logger::log(name(), "device", device, Logger::LOGLEVEL_DEBUG);

		
    // read config
    read_conf(args);

    Logger::log(name(), "ctl_update_rate", ctl_update_rate, Logger::LOGLEVEL_DEBUG);
    Logger::log(name(), "ctl_mode", ctl_mode, Logger::LOGLEVEL_DEBUG);

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
    if(cc_v4l2_open(device))
      Logger::log(name(), "Couldn't open video device", Logger::LOGLEVEL_DEBUG);
    cc_v4l2_query();
    cc_list_controls();
    // initialize all controls
    // printf("unknown control: '%d'\n", cam_ctrls["blub"]);
    // printf("Auto Exposure: %d\n", cc_v4l2_get(cam_ctrls["Auto Exposure"]));
    cc_v4l2_set(cam_ctrls["Auto Exposure"], 1); // manual mode = 1
    // cc_v4l2_set(cam_ctrls["Auto Gain"], 0);
    cc_v4l2_set(cam_ctrls["Gain, Automatic"], 0); // 3.7.0 since v4l driver?
    // cc_v4l2_set(cam_ctrls["Auto White Balance"], 0); // ipse
    cc_v4l2_set(cam_ctrls["White Balance, Automatic"], 0);
    // printf("Auto Exposure: %d\n", cc_v4l2_get(cam_ctrls["Auto Exposure"]));
    exposure = cc_v4l2_get(cam_ctrls["Exposure"]);
    contrast = cc_v4l2_get(cam_ctrls["Contrast"]);
    // gain = cc_v4l2_get(cam_ctrls["Main Gain"]);
    gain = cc_v4l2_get(cam_ctrls["Gain"]);
    // printf("Exposure: %d\n", exposure);

    pid_cam = new PID(0, params["cam_Kc"], params["cam_Ti"],
                      params["cam_Td"]);

#ifdef HAVE_LIBOSCPACK
    osc_en = 0;
    get_value_from_args("osc_en", osc_en);
    if(osc_en > 0) {
      get_value_from_args("osc_port", osc_port);
      /* initializations */
      osc_lp = new V_CAMCTRLOscPacketListener(*this);
      // osc_lp = new V_CAMCTRLOscPacketListener();
      osc_sp = new UdpListeningReceiveSocket(
                                             IpEndpointName(IpEndpointName::ANY_ADDRESS, osc_port),
                                             osc_lp);
    }
#endif // HAVE_LIBOSCPACK
  }

  V_CAMCTRLApp::~V_CAMCTRLApp() {}

  void V_CAMCTRLApp::setExposure(int value) {
    exposure = value;
  }

  void V_CAMCTRLApp::setContrast(int value) {
    contrast = value;
  }

  void V_CAMCTRLApp::setGain(int value) {
    gain = value;
  }

  int V_CAMCTRLApp::cc_v4l2_open(std::string device) {
    // const char devicefile[] = "/dev/video0";
    // printf("device1: %s\n", devicefile);
    // printf("device2: %s\n", device.c_str());
    // fd = v4l2_open(devicefile, O_RDWR, 0);
    fd = v4l2_open(device.c_str(), O_RDWR, 0);
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
        // printf("add_control: ctrl.id: %d, ctrl.name = %s\n", ctrl.id, ctrl.name);
        cam_ctrls[(char *)ctrl.name] = ctrl.id;
        // printf("add_control: ctrl.id: %d, ctrl.name = %s\n", cam_ctrls[(char *)ctrl.name], (char *)ctrl.name);
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
            // printf("add_control x: ctrl.id: %d, ctrl.name = %s\n", ctrl.id, ctrl.name);
            cam_ctrls[(char *)ctrl.name] = ctrl.id;
            // printf("add_control: ctrl.id: %d, ctrl.name = %s\n", cam_ctrls[(char *)ctrl.name], (char *)ctrl.name);
          }
        }

        /* Check any custom controls */
        for(int i=V4L2_CID_PRIVATE_BASE; ; i++) {
          ctrl.id = i;
          if(v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0) {
            // mw->add_control(ctrl, fd, grid, gridLayout);
            // printf("add_control priv: ctrl.id: %d, ctrl.name = %s\n", ctrl.id, ctrl.name);
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
    // printf("updateHardware: c.id = %d, c.value = %d\n", c.id, c.value);
    if(v4l2_ioctl(fd, VIDIOC_S_CTRL, &c) == -1) {
      // QString msg;
      // msg.sprintf("Unable to set %s\n%s", name, strerror(errno));
      // QMessageBox::warning(this, "Unable to set control", msg, "OK");
      // printf("blub\n");
      Logger::log(name(), "Couldn't set v4l2 control", id, Logger::LOGLEVEL_DEBUG);
    }
    // updateStatus();
  }   

  int V_CAMCTRLApp::cc_v4l2_get(int id)
  {   
    struct v4l2_control c;
    if(id == 0) return -1;
    c.id = id;
    // printf("updateHardware: c.id = %d\n", c.id);
    if(v4l2_ioctl(fd, VIDIOC_G_CTRL, &c) == -1) {
      // QString msg;
      // msg.sprintf("Unable to get %s\n%s", name,
      // 						strerror(errno));
      // QMessageBox::warning(this, "Unable to get control", msg, "OK");
      Logger::log(name(), "Couldn't get v4l2 control", id, Logger::LOGLEVEL_DEBUG);
    }
    return c.value;
  }

  void V_CAMCTRLApp::calcCamCtrlMean() {
    // float e_mean = 0;
    // static float e_mean_i = 0;
    // e_mean = params["cam_sp"] - mean;
    // exposure = exposure + (e_mean * 1.0);
    exposure = exposure + pid_cam->calc(0.1, mean);
    Logger::log(name(), "exposure: ", exposure, Logger::LOGLEVEL_DEBUG);
    Logger::log(name(), "mean, sp: ", mean, pid_cam->getSp(), Logger::LOGLEVEL_DEBUG);
    contrast = 32;
  }

  void V_CAMCTRLApp::calcCamCtrlHeuristic() {
    int i;
    int cnt_lower = 0, cnt_upper = 0;
    // set camera
    // check lower
    for(i = 0; i < 2; i++) {
      cnt_lower += (int)cap_hist.at<float>(i);
    }
    // check upper
    for(i = histSize-1; i > histSize-3; i--) {
      cnt_upper += (int)cap_hist.at<float>(i);
    }

    // Logger::log(name(), "lower:", cnt_lower, "upper:", cnt_upper, Logger::LOGLEVEL_DEBUG);

    // heuristics
    if((cnt_lower > Ntenth) && (cnt_upper > Ntenth)) {
      contrast--;
      // Logger::log(name(), "Wanne", Logger::LOGLEVEL_DEBUG);
    }
    else if(((int)cap_hist.at<float>(0) < Ntenth) &&
            ((int)cap_hist.at<float>(histSize-1) < Ntenth)) {
      contrast++;
      // Logger::log(name(), "Beule", Logger::LOGLEVEL_DEBUG);
    }
    else if(cnt_lower > Ntenth) {
      exposure++;
      // Logger::log(name(), "Links", Logger::LOGLEVEL_DEBUG);
    }
    else if(cnt_upper > Ntenth) {
      exposure--;
      // Logger::log(name(), "Rechts", Logger::LOGLEVEL_DEBUG);
    }
  }

  void V_CAMCTRLApp::calcCamCtrlHomeoRand() {
    int stateCritical;
    // int i;
    int cnt_lower = 0, cnt_upper = 0;

    // check lower
    cnt_lower = (int)cap_hist.at<float>(0);
    // check upper
    cnt_upper = (int)cap_hist.at<float>(histSize-1);

    // Logger::log(name(), "lower:", cnt_lower, "upper:", cnt_upper, Logger::LOGLEVEL_DEBUG);
		
    if(cnt_lower > (Ntenth*40) || cnt_upper > (Ntenth*40))
      stateCritical = 1;
    else
      stateCritical = 0;

    if(stateCritical) {
      // 	randomlyCreateNewConfig();
      // absolute value
      exposure = 255 * ((float)rand()/RAND_MAX);
      contrast = 255 * ((float)rand()/RAND_MAX);
      gain = 63 * ((float)rand()/RAND_MAX);

      // differential
      // exposure += (10 * ((float)rand()/RAND_MAX)) - 5;
      // contrast += (10 * ((float)rand()/RAND_MAX)) - 5;
      // gain += (6 * ((float)rand()/RAND_MAX)) - 3;

      // exposure = 10;
      //			printf("rand: %f\n", );
    }
    else {
    }
  }

  void V_CAMCTRLApp::calcCamCtrlWeightedHisto() {
    int i;
    // int cnt_lower = 0, cnt_upper = 0;
    int exp_pos = 0, exp_neg = 0;
    // int contr_pos = 0, contr_neg = 0;

    // set camera
    // check lower
    for(i = 0; i < (histSize/2); i++) {
      exp_pos += (int)cap_hist.at<float>(i);
    }
    // check upper
    for(i = histSize-1; i >= (histSize/2); i--) {
      exp_neg += (int)cap_hist.at<float>(i);
    }
    exposure += (exp_pos / 1000.);
    exposure -= (exp_neg / 1000.);

    // Logger::log(name(), "exposure:", exposure, Logger::LOGLEVEL_DEBUG);
    // Logger::log(name(), "exp_pos:", exp_pos, "exp_neg:", exp_neg, Logger::LOGLEVEL_DEBUG);
		
  }

  void V_CAMCTRLApp::calcCamCtrlExtern() {
    exposure = DataCenter::get_exposure();
    contrast = DataCenter::get_contrast();
    gain = DataCenter::get_gain();
  }

  void V_CAMCTRLApp::calcCamCtrl() {
    static mavlink_message_t msg;
    static mavlink_debug_t dbg;
    static mavlink_huch_cam_state_t cam_state;
    static mavlink_data64_t d64;

#ifdef HAVE_LIBOSCPACK
    char buffer[OSC_OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OSC_OUTPUT_BUFFER_SIZE);
#endif // HAVE_LIBOSCPACK

    // FIXME: make dt a class variable
    // FIXME: hard/soft limit function from library (mavhub, ...)
    // FIXME: add different methods

    //FIXME: remove benchmark
    //uint64_t start_time = get_time_ms();

    // uint64_t start, end;
    // start = 0; end = 0;

    /// Set the ranges ( for B,G,R) )
    float lrange[] = { 0, 256 } ;
    const float* lhistRange = { lrange };

    int i; // , N, Ntenth;
    int histRatio;

    calcHist(&new_image, 1, 0, Mat(), cap_hist, 1, &histSize, &lhistRange, uniform, accumulate);

    N = is_width * is_height;
    Ntenth = (int)(N * 0.01);
    histRatio = 256/histSize; // binWidth
    mean = 0.;
    var = 0.;
    std = 0.;
    skew = 0.;
    for (i = 0; i < histSize; i++) {
      mean += i * cap_hist.at<float>(i) * histRatio;
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

    // switch mode
    switch(ctl_mode) {
    case 0:
      calcCamCtrlMean();
      break;
    case 1:
      calcCamCtrlHeuristic();
      break;
    case 2:
      calcCamCtrlHomeoRand();
      break;
    case 3:
      calcCamCtrlWeightedHisto();
      break;
    case 4:
      calcCamCtrlExtern();
      break;
    default:
      calcCamCtrlMean();
      break;
    }
    // set and read camera settings
    // FIXME: use output clamping / surpression


    // clamp output?
    if(params["output_enable"] > 0) {
      cc_v4l2_set(cam_ctrls["Exposure"], exposure);
      cc_v4l2_set(cam_ctrls["Contrast"], contrast);
      // cc_v4l2_set(cam_ctrls["Main Gain"], gain);
      cc_v4l2_set(cam_ctrls["Gain"], gain);
    }

    exposure = cc_v4l2_get(cam_ctrls["Exposure"]);
    contrast = cc_v4l2_get(cam_ctrls["Contrast"]);
    // gain = cc_v4l2_get(cam_ctrls["Main Gain"]);
    gain = cc_v4l2_get(cam_ctrls["Gain"]);

    // send debug: statistics, cam ctrl params
    send_debug(&msg, &dbg, 0, mean);
    send_debug(&msg, &dbg, 1, std);
    send_debug(&msg, &dbg, 2, skew);
    send_debug(&msg, &dbg, 3, exposure);
    send_debug(&msg, &dbg, 4, contrast);
    send_debug(&msg, &dbg, 5, gain);

    // send cam_state
    cam_state.cam_index = 0;
    cam_state.exposure = exposure;
    cam_state.contrast = contrast;
    cam_state.gain = gain;
    cam_state.hist1 = cap_hist.at<float>(0);
    cam_state.hist2 = cap_hist.at<float>(1);
    cam_state.hist3 = cap_hist.at<float>(2);
    cam_state.hist4 = cap_hist.at<float>(3);
    cam_state.hist5 = cap_hist.at<float>(4);
    cam_state.hist6 = cap_hist.at<float>(5);
    cam_state.hist7 = cap_hist.at<float>(6);
    cam_state.hist8 = cap_hist.at<float>(7);

    mavlink_msg_huch_cam_state_encode(
                                      system_id(),
                                      component_id,
                                      &msg,
                                      &cam_state);
    Logger::log(name(), "sending cam_state", cam_state.cam_index, Logger::LOGLEVEL_DEBUG);
    AppLayer<mavlink_message_t>::send(msg);

    // send binary data over mavlink
    int num_camctrl_parms = 3;
    d64.type = 0;
    d64.len = 44;
    bfconvert.val[0] = exposure;
    bfconvert.val[1] = contrast;
    bfconvert.val[2] = gain;
    for (i = 0; i < histSize; i++) {
      // add single values to osc message
      // p << cap_hist.at<float>(i);
      // add values to buf
      bfconvert.val[i+num_camctrl_parms] = cap_hist.at<float>(i);// / (float)N - 1;
    }
    for (i = 0; i < 44; i++) {
      d64.data[i] = bfconvert.bytes[i];
    }
    mavlink_msg_data64_encode(
                              system_id(),
                              component_id,
                              &msg,
                              &d64);
    Logger::log(name(), "sending data64", (int)d64.type, Logger::LOGLEVEL_DEBUG);
    AppLayer<mavlink_message_t>::send(msg);
		
#ifdef HAVE_LIBOSCPACK
    if(osc_en > 0)
      {
        Logger::log(name(), "osc_en", osc_en, Logger::LOGLEVEL_DEBUG);
        // prepare local hist buffer
        float hist[histSize+num_camctrl_parms];
        // add histogram vector
        hist[0] = exposure; ///128. - 1;
        hist[1] = contrast; ///128. - 1;
        hist[2] = gain; ///32. -1;
        for (i = 0; i < histSize; i++) {
          // add single values to osc message
          // p << cap_hist.at<float>(i);
          // add values to buf
          hist[i+num_camctrl_parms] = cap_hist.at<float>(i);// / (float)N - 1;
        }

        p << osc::BeginBundleImmediate
          << osc::BeginMessage( "/mavhub/camctrl") ;
        // << exposure
        // << contrast
        // << gain;
        // add histogram as blob
        p << osc::Blob(hist, (histSize+num_camctrl_parms)*sizeof(float));

        p << osc::EndMessage
          << osc::EndBundle;

        // IpEndpointName( "192.168.1.10", 17779),
        osc_sp->SendTo(
                       IpEndpointName( "127.0.0.1", 17779),
                       p.Data(),
                       p.Size()
                       );
      }
#endif // HAVE_LIBOSCPACK

    // Logger::log(name(), "exp:", exposure, Logger::LOGLEVEL_DEBUG);
    // Logger::log(name(), "ctr:", contrast, Logger::LOGLEVEL_DEBUG);

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

            pid_cam->setKc(params["cam_Kc"]);
            pid_cam->setTi(params["cam_Ti"]);
            pid_cam->setTd(params["cam_Td"]);
            pid_cam->setBias(params["cam_bias"]);
            pid_cam->setSp(params["cam_sp"]);
            ctl_mode = int(params["ctl_mode"]);
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

    case MAVLINK_MSG_ID_HUCH_CAM_CMD:
      if( (msg.sysid == system_id()) ) {
        // mavlink_msg_huch_ctrl_hover_state_decode(&msg, &hover_state);
        // transfer desired values via datacenter
        DataCenter::set_camctrl(
                                mavlink_msg_huch_cam_cmd_get_exposure(&msg),
                                mavlink_msg_huch_cam_cmd_get_contrast(&msg),
                                mavlink_msg_huch_cam_cmd_get_gain(&msg));
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
    int i;
    /// Normalize the result to [ 0, histImage.rows ]
    histImage = 0.;
    // normalize(cap_hist, cap_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
    for(i = 1; i < histSize; i++ )
      {
        line(histImage,
             Point( bin_w*(i-1), hist_h - cvRound(cap_hist.at<float>(i-1) * 0.1)),
             Point( bin_w*(i), hist_h - cvRound(cap_hist.at<float>(i) * 0.1)),
             Scalar( 255, 0, 0), 1, 8, 0);
      }	

    line(histImage,
         Point(10, hist_h),
         Point(10, hist_h-exposure),
         Scalar( 100, 100, 0), 1, 8, 0
         );
    line(histImage,
         Point(20, hist_h),
         Point(20, hist_h-contrast),
         Scalar( 100, 100, 0), 1, 8, 0
         );
    line(histImage,
         Point(30, hist_h),
         Point(30, hist_h-(gain*4)),
         Scalar( 100, 100, 0), 1, 8, 0
         );
    // Logger::log(name(), "mean", mean, Logger::LOGLEVEL_DEBUG, _loglevel);
    line(histImage,
         Point(2*mean, hist_h),
         Point(2*mean, 0),
         Scalar( 100, 100, 0), 2, 8, 0
         );
		
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
    // uint64_t start_time;
    // uint64_t stop_time;

    // sleep(2);

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

    // uint64_t start_time = get_time_ms();
    // uint64_t stop_time = get_time_ms();

    // FIXME: unfortunately blocks, need to use bridge_osc_app
    // osc_sp->Run();

    pid_cam->setSp(params["cam_sp"]);

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

    iter = args.find("reset_i");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["reset_i"];
    }
    else {
      params["reset_i"] = 0.0;
    }

    iter = args.find("output_enable");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["output_enable"];
    }
    else {
      params["output_enable"] = 1.0;
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
      s >> params["ctl_mode"];
    }
    else {
      params["ctl_mode"] = 0.0;
    }

    // mean pixel value setpoint
    iter = args.find("cam_sp");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["cam_sp"];
    }
    else {
      params["cam_sp"] = 80.;
    }

    // run method update rate
    iter = args.find("ctl_update_rate");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> ctl_update_rate;
    }
    else
      ctl_update_rate = 100;

    // run method update rate
    iter = args.find("ctl_mode");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["ctl_mode"];
    }
    else
      params["ctl_mode"] = ctl_mode = 100;
    //	assign variable
    ctl_mode = int(params["ctl_mode"]);

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

    // PID params
    iter = args.find("cam_Kc");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["cam_Kc"];
    }
    else {
      params["cam_Kc"] = 0.1;
    }
    iter = args.find("cam_Ti");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["cam_Ti"];
    }
    else {
      params["cam_Ti"] = 0.0;
    }
    iter = args.find("cam_Td");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["cam_Td"];
    }
    else {
      params["cam_Td"] = 0.0;
    }


    // typedef map<string, double>::const_iterator ci;
    // for(ci p = params.begin(); p!=params.end(); ++p) {
    // 	// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
    // 	// if(!strcmp(p->first.data(), (const char *)param_id)) {
    // 	// params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
    // 	Logger::log("v_camctrl_app::read_conf", p->first, params[p->first], Logger::LOGLEVEL_INFO);
    // 	// Logger::log(name(), "handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_DEBUG);
    // }

  }
} // namespace mavhub

#endif // defined(HAVE_LIBV4L2)

#endif // defined(HAVE_OPENCV2) && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
