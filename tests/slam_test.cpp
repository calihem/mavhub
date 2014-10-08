#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <inttypes.h>
#include <fstream>
#include <cmath>	//sqrt

#include "lib/slam/tracker.h"
#include "lib/hub/utility.h"

using namespace std;
using namespace hub;
using namespace hub::slam;

BOOST_AUTO_TEST_SUITE(hub_slam_tests)

static const string dir_prefix("../tests/data/hovering/");
static const string whitespaces(" \t\f\v\n\r");
static const unsigned int image_width = 752;
static const unsigned int image_height = 480;

template<typename T>
int parse_data(const string &line, string::size_type &end, T &data);

template<>
int parse_data<std::string>(const string &line, string::size_type &end, std::string &data) {
	string::size_type start = line.find_first_not_of(whitespaces, end);
	end = line.find_first_of(whitespaces, start);
	data = line.substr(start, end-start).insert(0, dir_prefix);

	return 0;
}

// partial specialization of function templates aren't allowd :(
template<>
int parse_data< std::vector<float> >(const string &line, string::size_type &end, std::vector<float> &data) {
	string::size_type start;
	istringstream value_stream;
	for(unsigned int i=0; i<data.size(); i++) {
		start = line.find_first_not_of(whitespaces, end);
		end = line.find_first_of(whitespaces, start);
		value_stream.str(line.substr(start, end-start));
		value_stream.clear(); //remove error state flag
		value_stream >> data[i];
	}
	
	return 0;
}

template<typename T>
int parse_data_line(ifstream &file, double &time, T &data) {
	string line;
start:
	if( !getline(file, line) )
		return -1;
  
	// skip empty lines
	if( line.empty() ) goto start;
	// ignore whitespace at beginning
	string::size_type start = line.find_first_not_of(whitespaces);
	// skip white lines
	if(start == string::npos) goto start;
	// skip comments
	if(line[start] == '%') goto start;

	// read timestamp
	string::size_type end = line.find_first_of(whitespaces, start);
	istringstream time_stream(line.substr(start, end-start));
	time_stream >> std::dec >> time;

	return parse_data(line, end, data);
}

BOOST_AUTO_TEST_CASE(tracker_translation_test) {
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
	std::bitset<8> debug_mask;
	debug_mask.set(Tracker::POSE);
// 	for(unsigned int i=0; i<10 && video_capture.grab(); i++ ) {
	while( video_capture.grab() ) {
		counter++;
 		if( (counter % 20) )
 			continue;

		cv::Mat frame;
		video_capture.retrieve(frame);
		cv::cvtColor(frame, image, CV_BGR2GRAY);

		tracker.track_camera(image, parameter_vector, altitude, debug_mask);
		std::stringstream parameter_stream;
		parameter_stream << parameter_vector;
		BOOST_TEST_MESSAGE( "parameter_vector: " << parameter_stream.str());

// cv::imshow("image", image);
// cv::waitKey(100);
	}
	
	tracker.save_map("video");
}

BOOST_AUTO_TEST_CASE(tracker_rotation_test) {
	//TODO
}

BOOST_AUTO_TEST_CASE(tracker_dead_reckoning_test) {
	//TODO
}

BOOST_AUTO_TEST_CASE(tracker_test) {
	// open file with image names
	static const string image_list_name(dir_prefix + "hovering_down_images.txt");
	ifstream  image_list_file(image_list_name.c_str(), ifstream::in);
	BOOST_CHECK( image_list_file.is_open() );

	// open file with imu data
	static const string imu_data_name(dir_prefix + "hovering_down_imu.txt");
	ifstream imu_data_file(imu_data_name.c_str(), ifstream::in);
	BOOST_CHECK( imu_data_file.is_open() );

	// open file with vicon data
	static const string vicon_data_name(dir_prefix + "hovering_down_vicon.txt");
	ifstream vicon_data_file(vicon_data_name.c_str(), ifstream::in);
	BOOST_CHECK( vicon_data_file.is_open() );

	// create tracker
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 260.0, 0.0, image_width/2,
		0.0, 260.0, image_height/2,
		0.0, 0.0, 1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
	Tracker tracker(image_width, image_height, camera_matrix, distortion_coefficients);

	double image_time, imu_time, vicon_time;
	string image_name;
	cv::Mat image;
	vector<float> parameter_vector(6, 0);
	vector<float> imu_vector(7, 0);
	vector<float> vicon_vector(6, 0);
	while( parse_data_line(image_list_file, image_time, image_name) == 0 ) {
		// read image
		image = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
		BOOST_REQUIRE_MESSAGE(image.data, "Can't read image " << image_name << ", abort execution.");

		// read imu data
		parse_data_line(imu_data_file, imu_time, imu_vector);
		BOOST_CHECK(image_time == imu_time);

		// set parameter vector
		parameter_vector[0] = imu_vector[0];
		parameter_vector[1] = imu_vector[1];
		parameter_vector[2] = imu_vector[2];

		// start output stream
		std::stringstream message_stream;
		message_stream << parameter_vector;

		tracker.track_camera(image, parameter_vector, imu_vector[6]);

		message_stream << " -> " << parameter_vector ;
		BOOST_TEST_MESSAGE( "parameter_vector: " << message_stream.str());

		parse_data_line(vicon_data_file, vicon_time, vicon_vector);
		message_stream.str("");
		message_stream << vicon_vector;
		BOOST_TEST_MESSAGE( "vicon_vector: " << message_stream.str());

		//TODO check against ground truth
	}
}

BOOST_AUTO_TEST_SUITE_END()
