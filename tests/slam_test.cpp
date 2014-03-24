#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <inttypes.h>
#include <fstream>
#include <cmath>	//sqrt

#include "lib/slam/tracker.h"
#include "lib/hub/utility.h"

using namespace hub;
using namespace hub::slam;

BOOST_AUTO_TEST_SUITE(SLAMTestSuite)

BOOST_AUTO_TEST_CASE(Test_slam)
{
	cv::VideoCapture video_capture("../tests/data/video.yuv");
	BOOST_CHECK( video_capture.isOpened() );
	const float altitude = 66; // average height used in video
	// matlab results
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 281.29246,   0.0, 157.23632,
	                                                    0.0, 277.64192, 111.98711,
	                                                    0.0,   0.0,   1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.06161, -0.17161, 0.0013, -0.00171, 0.0);
	// OpenCV results
// 	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.56391,   0.0, 156.02819,
// 	                                                    0.0, 284.468277, 111.535801,
// 	                                                    0.0,   0.0,   1.0);
// 	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912407462, -0.2874599, 0.00188917, 0.0017930665, 0.09828093);

	Tracker tracker(320, 240, camera_matrix, distortion_coefficients);
	std::vector<float> parameter_vector(6, 0);

// cv::namedWindow("image", CV_WINDOW_AUTOSIZE);

	cv::Mat image;
	unsigned int counter = 0;
// 	for(unsigned int i=0; i<10 && video_capture.grab(); i++ ) {
	while( video_capture.grab() ) {
		counter++;
// 		if( (counter % 20) )
// 			continue;

		cv::Mat frame;
		video_capture.retrieve(frame);
		cv::cvtColor(frame, image, CV_BGR2GRAY);

		tracker.track_camera(image, parameter_vector, altitude);
		std::stringstream parameter_stream;
		parameter_stream << parameter_vector;
		BOOST_TEST_MESSAGE( "parameter_vector: " << parameter_stream.str());

// cv::imshow("image", image);
// cv::waitKey(1);
	}
	
	tracker.save_map("video");
}

BOOST_AUTO_TEST_SUITE_END()
