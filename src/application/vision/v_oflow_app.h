#ifndef _V_OFLOW_APP_H_
#define _V_OFLOW_APP_H_

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
/* #include <opencv2/core/core.hpp> */
/* #include <opencv2/highgui/highgui.hpp> */
#include <opencv/cv.h>
//#include <opencv/highgui.h>

#if CV_MINOR_VERSION >= 2
#include <brisk/brisk.h>
#include "lib/slam/features.h"
#include "protocol/protocollayer.h"

#include "ofmodel.h"
#include "OpticalFlow.h"
#include "ofdifferential.h"

#include <inttypes.h> //uint8_t

// #define ABS(x) ((x)<0?-(x):(x))
#define SGN(x) ((x)==0?0:((x)>0?1:-1))

struct UnwrapSettings {
	int cx;		// x-coordinate of center point
	int cy;		// y-coordinate of center point
	int ri;		// inner radius
	int ro;		// outer radius
	int im;		// interpolation mode
	double sx;	// scaling in x-direction
	double sy;	// scaling in y-direction
	int fw;		// fixed width 
	int fh;		// fixed height 
	UnwrapSettings(int cx, int cy, int ri, int ro, int im, double sx, double sy, int fw, int fh);
};

namespace mavhub {

	/// Enumeration of camera types
	enum cam_type_t {
		CAM_TYPE_PLANAR, /// planar camera image
		CAM_TYPE_OMNI	/// Omnidirectional camera image
	};

	/// Enumeration of algorithm types
	enum of_algorithm_t {
		UNKNOWN, /// Unknown algorithm
		FIRST_ORDER, /// First order
		HORN_SCHUNCK, /// Horn-Schunck
		CENSUS_TRANSFORM, /// Census Transform
		LINE_SUM, /// Line sum
		LINE_CENSUS /// Line census
	};

	class V_OFLOWApp : public MavlinkAppLayer,
		public hub::gstreamer::VideoClient {

	public:
		V_OFLOWApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~V_OFLOWApp();

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
		unsigned int target_system;
		unsigned int target_component;
		unsigned int imu_rate;
		unsigned int en_heartbeat;
		/// Current attitude of system
		mavlink_attitude_t attitude;
		/// raw imu data
		// mavlink_huch_mk_imu_t huch_mk_imu;

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

		of_algorithm algo;
		OFModel *ofModel;
		OpticalFlow *oFlow;

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

		//FIXME: replace old_* by database of these informations
		std::string sink_name;
		/* cv::Mat cam_matrix; */
		/* cv::Mat dist_coeffs; */
		cv::Mat old_image;
		cv::Mat new_image;
		cv::Mat new_image_raw;
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

		int initModel(of_algorithm algo);
		void calcFlow();
		// void load_calibration_data(const std::string &filename);
		void getOF_FirstOrder();
		void getOF_FirstOrder2();
		void getOF_FirstOrder_Omni();
		void preprocessImage(cv::Mat img);
		UnwrapSettings& defaultSettings();
		void unwrapImage(cv::Mat* inputImg, cv::Mat* outputImg, UnwrapSettings& opt);
		void visualize(cv::Mat img);
		// uint8_t getInterpolation(cv:Mat* inputImg, int im, double x, double y);

		void ctrlLateral();

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
