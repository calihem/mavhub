#ifndef _V_OFLOW_CAR_APP_H_
#define _V_OFLOW_CAR_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#ifdef HAVE_LIBFANN
#include "floatfann.h"

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>

#if CV_MINOR_VERSION >= 2
#include <brisk/brisk.h>
#include "lib/slam/features.h"
#include "protocol/protocollayer.h"

#include "ofmodel.h"
#include "OpticalFlow.h"
#include "ofdifferential.h"

#include "../../module/filter_ma.h"
#include "../../module/exec_timing.h"

#include <inttypes.h> //uint8_t

#include "toolbox.h"
/* // #define ABS(x) ((x)<0?-(x):(x)) */
/* #define SGN(x) ((x)==0?0:((x)>0?1:-1)) */

/* /\\** */
/*    fast 32 bit integer square root */
/* *\/ */
/* #define ITERATE(N)                              \ */
/*   tries = root + (1 << (N));                    \ */
/*   if (n >= tries << (N))                        \ */
/*     {   n -= tries << (N);                      \ */
/*       root |= 2 << (N);                         \ */
/*     } */
/* inline uint32_t isqrt (uint32_t n) { */
/*   uint32_t root = 0, tries; */
/*   ITERATE(15); ITERATE(14); ITERATE(13); ITERATE(12); */
/*   ITERATE(11); ITERATE(10); ITERATE( 9); ITERATE( 8); */
/*   ITERATE( 7); ITERATE( 6); ITERATE( 5); ITERATE( 4); */
/*   ITERATE( 3); ITERATE( 2); ITERATE( 1); ITERATE( 0); */

/*   return root >> 1; */
/* } */

namespace mavhub {

  class V_OFLOWCarApp : public AppLayer<mavlink_message_t>,
    public hub::gstreamer::VideoClient {

  public:
      V_OFLOWCarApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
      virtual ~V_OFLOWCarApp();

      virtual void handle_input(const mavlink_message_t &msg);
      virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

  protected:
      virtual void print(std::ostream &os) const;
      virtual void run();

  private:
      /// component id
      uint16_t component_id;
      /// enable video output stream
      bool with_out_stream;
      /// capture new image request
      uint8_t take_new_image;
      /// new video data flag
      bool new_video_data;
      /// Mutex to sync between application thread and input calls
      pthread_mutex_t sync_mutex;
      // pthread_mutex_t extctrl_mutex;
      unsigned int target_system;
      unsigned int target_component;
      unsigned int imu_rate;
      unsigned int en_heartbeat;
      /// Current attitude of system
      mavlink_attitude_t attitude;
      /// raw imu data
      // mavlink_huch_mk_imu_t huch_mk_imu;

      /// update rate
      int ctl_update_rate;

      // oflow / planar-persprective
      float of_u;
      float of_v;
      float of_u_i; /// integral
      float of_v_i; /// integral
      float of_u_i_derot; /// derotation
      float of_v_i_derot; /// derotation

      // oflow omni
      float of_yaw;
      float of_alt;
      float of_x;
      float of_y;

      // LK pyr init
      bool needToInit;

      of_algorithm of_algo;
      OFModel *ofModel;
      OFModel* ofModels[NUM_OF_ALGORITHM];
      OpticalFlow *oFlow;

      /// input stream parameters
      int is_width;
      int is_height;

      /// execution timing
      Exec_Timing* exec_tmr;

      /// parameter request
      int param_request_list;
      /// parameters
      std::map<std::string, double>	params;

      // neural network
      struct fann *ann;   /// the network
      fann_type ann_x[8]; /// ann input
      fann_type *ann_y;   /// ann output

      mavlink_huch_imu_raw_adc_t raw_adc_imu; /// for Gyro values
      mavlink_huch_ctrl_hover_state_t hover_state; /// for altitude estimate
      mavlink_huch_sensor_array_t sensor_array_x; // raw sensor array
      mavlink_huch_sensor_array_t sensor_array_y; // raw sensor array

      /// gyro moving average filter
      MA *ma_pitch;
      MA *ma_roll;

      /// lateral control active
      uint8_t lc_active;

      //FIXME: replace old_* by database of these informations
      std::string sink_name;
      /* cv::Mat cam_matrix; */
      /* cv::Mat dist_coeffs; */
      cv::Mat old_image;
      cv::Mat new_image;
      cv::Mat new_image_raw;
      cv::Mat img_display;
      cv::Mat flow;
      // neural network
      cv::Mat M1;
      cv::Mat u;
      /* mavlink_attitude_t old_attitude; */
      /* mavlink_attitude_t new_attitude; */
      /* 		std::vector<cv::KeyPoint> old_features; */
      /* 		std::vector<cv::KeyPoint> new_features; */
      /* 		std::vector<cv::Point3f> old_object_points; */
      /* 		cv::BriskFeatureDetector feature_detector; */
      /* 		cv::Mat old_descriptors; */
      /* 		cv::Mat new_descriptors; */
      /* 		cv::BriskDescriptorExtractor descriptor_extractor; */
      /* #ifdef HAVE_SSSE3 */
      /* 		cv::BruteForceMatcher<cv::HammingSse> matcher; */
      /* #else */
      /* 		cv::BruteForceMatcher<cv::Hamming> matcher; */
      /* #endif */
      /* 		cv::Mat rotation_vector; */
      /* 		cv::Mat translation_vector; */

      int initModels();
      void calcFlow();
      void calcESN();
      // void load_calibration_data(const std::string &filename);
      void getOF_FirstOrder();
      void getOF_HS();
      void getOF_LK();
      void getOF_LK2();
      void getOF_HORN_SCHUNCK_CV();
      void getOF_BLOCK_MATCHING_CV();
      void getOF_SF();
      virtual float getMeanVelXf(CvMat &vel, int x0, int x1, int y0, int y1) const;
      virtual float getMeanVelYf(CvMat &vel, int x0, int x1, int y0, int y1) const;
      void getOF_LK_Pyr();
      void preprocessImage(cv::Mat img);
      UnwrapSettings& defaultSettings();
      void unwrapImage(cv::Mat* inputImg, cv::Mat* outputImg, UnwrapSettings& opt);
      void visualize(cv::Mat img);
      // uint8_t getInterpolation(cv:Mat* inputImg, int im, double x, double y);

      /// send debug data
      void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);
      /// read data from config
      virtual void read_conf(const std::map<std::string, std::string> args);

      /* int iirFilter(int old_of, int new_of, int alpha); */
      /* int iirFilter(int old_of, int new_of); */
      inline float iirFilter(float old_of, float new_of, float alpha) {
        return (old_of * alpha - old_of + new_of) / alpha;
      }

      inline float iirFilter(float old_of, float new_of) {
        // return (7 * old_of + new_of) / 8;  // ((alpha-1) * old_of + new_of) / alpha
        return (3. * old_of + new_of) / 4.;  // ((alpha-1) * old_of + new_of) / alpha
      }

      inline float iirFilterHP(float old_of, float new_of, float alpha) {
        return (old_of * alpha - old_of + new_of) / alpha;
      }

    };

} // namespace mavhub

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif // HAVE_LIBFANN
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
#endif // _OPENGL_APP_H_
