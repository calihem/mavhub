#ifndef _V_CAMCTRL_APP_H_
#define _V_CAMCTRL_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
/* #include <opencv2/core/core.hpp> */
/* #include <opencv2/highgui/highgui.hpp> */
/* #include <opencv2/imgproc/imgproc.hpp> */
/* #include <opencv/cv.h> */
/* #include <opencv/highgui.h> */

#if CV_MINOR_VERSION >= 2
/* #include <brisk/brisk.h> */
/* #include "lib/slam/features.h" */
#include "protocol/protocollayer.h"
#include "../../module/PID.h"

/* #include "ofmodel.h" */
/* #include "OpticalFlow.h" */
/* #include "ofdifferential.h" */

/* #include "../../module/filter_ma.h" */
#include "../../module/exec_timing.h"

#include <linux/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include <inttypes.h> //uint8_t
#if (defined HAVE_LIBV4L2)
#include <libv4l2.h>

// HAVE_LIBOSCPACK
#ifdef HAVE_LIBOSCPACK
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#define OSC_OUTPUT_BUFFER_SIZE 1024
#endif // HAVE_LIBOSCPACK

// #define ABS(x) ((x)<0?-(x):(x))
#define SGN(x) ((x)==0?0:((x)>0?1:-1))

namespace mavhub {
#ifdef HAVE_LIBOSCPACK
	class V_CAMCTRLApp;
	// OSC Packet Listener Class
	class V_CAMCTRLOscPacketListener : public osc::OscPacketListener {
	public:
		V_CAMCTRLOscPacketListener(const V_CAMCTRLApp& app);
    virtual void ProcessMessage(const osc::ReceivedMessage& m, 
																const IpEndpointName& remoteEndpoint);
	};
#endif // HAVE_LIBOSCPACK

	// main cmctrl class
	class V_CAMCTRLApp : public AppLayer<mavlink_message_t>,
		public hub::gstreamer::VideoClient {

	public:
		V_CAMCTRLApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~V_CAMCTRLApp();

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

		/// update rate
		int ctl_update_rate;
		int ctl_mode;

		/// input stream parameters
		int is_width;
		int is_height;

		/// execution timing
		Exec_Timing* exec_tmr;

		/// parameter request
		int param_request_list;
		/// parameters
		std::map<std::string, double>	params;

		//FIXME: replace old_* by database of these informations
		std::string sink_name;
		/* cv::Mat cam_matrix; */
		/* cv::Mat dist_coeffs; */
		cv::Mat old_image;
		cv::Mat new_image;
		cv::Mat new_image_raw;
		cv::Mat img_display;

		// histogram
		int histSize;
		float range[]; //  = { 0, 256 } ;
		const float* histRange; // = { range };
		bool uniform; // = true;
		bool accumulate; // = false;
		int hist_w; // = 512;
		int hist_h; // = 400;
		int bin_w; //  = cvRound( (double) hist_w/histSize );
		/// image for displaying the histogram
		cv::Mat histImage;
		/// 1D image for histogram values
		cv::Mat cap_hist;

		// image properties
		float mean;
		float var;
		float std;
		float skew;
		float skew_scale;
		int N;
		int Ntenth;

		// camera fd
		int fd;
		// cam controls
		std::map<std::string, int> cam_ctrls;
		/// internal exposure
		int exposure;
		/// internal contrast
		int contrast;
		/// internal gain
		int gain;

		// mean based PID
		PID* pid_cam;


#ifdef HAVE_LIBOSCPACK
		int osc_en;
		uint16_t osc_port;
		V_CAMCTRLOscPacketListener* osc_lp;
		UdpListeningReceiveSocket* osc_sp;
#endif // HAVE_LIBOSCPACK

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
		void setExposure(int value);
		void setContrast(int value);
		void setGain(int value);

		void calcCamCtrl(); // main calculation
		/// cam ctrl heuristics based on mean pixel intensity
		void calcCamCtrlMean();
		/// cam ctrl heuristics
		void calcCamCtrlHeuristic();
		/// cam ctrl homeostatic with random reconfiguration
		void calcCamCtrlHomeoRand();
		/// cam ctrl weighted histogram function
		void calcCamCtrlWeightedHisto();
		void calcCamCtrlExtern();
		void preprocessImage(cv::Mat img);
		void visualize(cv::Mat img_src, cv::Mat img);
		// uint8_t getInterpolation(cv:Mat* inputImg, int im, double x, double y);

		/// send debug data
		void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);

		/// v4l2 open device
		int cc_v4l2_open();
		/// v4l2 query device
		int cc_v4l2_query();
		/// query collected device controls
		void cc_list_controls();
		/// v4l2 get control
		int cc_v4l2_get(int id);
		/// v4l2 set control
		void cc_v4l2_set(int id, int value);
	};

} // namespace mavhub

#endif // defined(HAVE_LIBV4L2)
#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
#endif // _OPENGL_APP_H_
