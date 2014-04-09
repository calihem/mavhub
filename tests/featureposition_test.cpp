#include "lib/slam/features.h"
#include "lib/slam/pose.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/detail/unit_test_parameters.hpp>

#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>

#include <cstdlib>
#include <limits>	//epsilon

#include "lib/hub/utility.h"
#include "lib/hub/time.h"

#define VISUAL_OUTPUT 0

using namespace boost::unit_test;
using namespace hub;
using namespace hub::slam;

BOOST_AUTO_TEST_SUITE(FeaturePositionTestSuite)

BOOST_AUTO_TEST_CASE(test_featureposition)
{
	//FIXME: avoid static filenames
	cv::Mat first_image = cv::imread("../tests/data/snap66_0.jpg", 0);
	cv::Mat second_image = cv::imread("../tests/data/snap66_10.jpg", 0);
	BOOST_REQUIRE(first_image.data && second_image.data);

	// the descriptor constructor is expensive and shouldn't be part of the benchmark
	cv::BriskDescriptorExtractor descriptor_extractor;

	uint64_t start_time = get_time_us();
	const uint64_t begin_time = start_time;

	// get features from image
	cv::BriskFeatureDetector feature_detector(60, 3); //threshold, octaves
	std::vector<cv::KeyPoint> first_keypoints, second_keypoints;
	feature_detector.detect(first_image, first_keypoints);
	feature_detector.detect(second_image, second_keypoints);
	BOOST_CHECK( !first_keypoints.empty() && !second_keypoints.empty() );
	uint64_t stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[  keypoints ]"
		<< " first image: " << first_keypoints.size()
		<< " | second image: " << second_keypoints.size()
		<< " | " << stop_time-start_time << "us");

	//OpenCV
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.56391339422925,   0.0, 156.0281939747785,
	                                                    0.0, 284.46827726278644, 111.53580190229269,
	                                                    0.0,   0.0,   1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.091240746270705428, -0.28745994339695397, 0.0018891719675274753, -0.0017930665030902364, 0.098280930311843484);
	// matlab
// 	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 281.29246,   0.0, 157.23632,
// 	                                                    0.0, 277.64192, 111.98711,
// 	                                                    0.0,   0.0,   1.0);
// 	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.06161, -0.17161, 0.0013, -0.00171, 0.0);

	start_time = get_time_us();

	cv::Mat first_descriptors, second_descriptors;
	// calculate descriptors which can remove or add some of the keypoints
	descriptor_extractor.compute(first_image, first_keypoints, first_descriptors);
	descriptor_extractor.compute(second_image, second_keypoints, second_descriptors);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[descriptors ] " << stop_time-start_time << "us");
	BOOST_CHECK( !first_keypoints.empty() && !second_keypoints.empty() );
	start_time = get_time_us();

	// undistort points
	std::vector<cv::Point2f> first_idealpoints;
	undistort_n2i(first_keypoints,
		camera_matrix,
		distortion_coefficients,
		first_idealpoints);
	std::vector<cv::Point2f> second_idealpoints;
	undistort_n2i(second_keypoints,
		camera_matrix,
		distortion_coefficients,
		second_idealpoints);
// 	BOOST_TEST_MESSAGE("[undistortion] " << stop_time-start_time << "us");
// 	BOOST_CHECK( !first_idealpoints.empty() && !second_idealpoints.empty() );
// 	start_time = get_time_us();

	// match descriptors
#if CV_MINOR_VERSION <= 3
	cv::BruteForceMatcher<cv::Hamming> matcher;
#else
	cv::BFMatcher matcher(cv::NORM_HAMMING);
#endif
	std::vector<cv::DMatch> matches;
	matcher.match(first_descriptors, second_descriptors, matches);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[  matching  ] " << stop_time-start_time) << "us";
	BOOST_CHECK( !matches.empty() );
	start_time = get_time_us();

	// filter matches
	std::vector<char> matches_mask(matches.size(), 1);
	filter_matches_by_robust_distribution(first_idealpoints, second_idealpoints, matches, matches_mask);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[  filtering ] " << stop_time-start_time) << "us";
	start_time = get_time_us();

	std::vector<float> parameter_vector(6, 0);

	// calculate corresponding 3D object points
	std::vector<cv::Point3f> objectpoints( first_idealpoints.size() );
	inverse_ideal_pinhole_model<float, rotation_matrix_rad>(
		reinterpret_cast<float*>( &(first_idealpoints[0].x) ),
		&parameter_vector[0],
		66.0, //distance from ground
		reinterpret_cast<float*>( &(objectpoints[0].x) ),
		first_idealpoints.size() );
// 	idealpoints_to_objectpoints<rotation_matrix_rad>(first_idealpoints,
// 		66, //distance from ground
// 		parameter_vector,
// 		objectpoints);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[objectpoints] " << stop_time-start_time) << "us";
	start_time = get_time_us();

	guess_translation<float>(objectpoints,
		second_idealpoints,
		66,
		matches,
		&parameter_vector[3],
		matches_mask);

	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[transl guess] " << stop_time-start_time) << "us";
	BOOST_TEST_MESSAGE("translation estimation: (" << parameter_vector[3] << ", " << parameter_vector[4] << ", " << parameter_vector[5] << ")");
	start_time = get_time_us();

// 	parameter_vector[4] = 10.0;

	float info[LM_INFO_SZ];
	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_approx_ideal_pinhole_euler_jac<float> >(
// 	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_ideal_pinhole_euler_jac<float> >(
		objectpoints,
		second_idealpoints,
		matches,
		parameter_vector,
		matches_mask,
		100,	//max iterations
		info);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[pose estim. ] " << stop_time-start_time) << "us";
	BOOST_TEST_MESSAGE("estimate_pose returned " << rc
		<< " in " << info[5]
		<< " iterations, reason " << info[6]
		<< ", error " << info[1] / objectpoints.size()
		<< " [initial " << info[0] / objectpoints.size()
		<< "], " << (int)info[7] << "/" << (int)info[8]
		<< " func/fjac evals, " << (int)info[9] << " lin. systems.");
	BOOST_CHECK(rc > 0);

	BOOST_TEST_MESSAGE("estimated pose (" << rc << "): "
		<< parameter_vector[0] <<", " << parameter_vector[1] <<", " << parameter_vector[2] << ", " 
		<< parameter_vector[3] <<", " << parameter_vector[4] <<", " << parameter_vector[5] << ")"
	);
	BOOST_TEST_MESSAGE("whole pipeline took: " << (stop_time-begin_time)/1000 << "ms");


#if VISUAL_OUTPUT
	cv::Mat match_img;
	cv::drawMatches(first_image, first_keypoints,
		second_image, second_keypoints,
		matches,
		match_img,
		cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
		matches_mask,
		cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);

	cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
	cv::imshow("image", match_img);
	cv::waitKey();
#endif //VISUAL_OUTPUT
}

BOOST_AUTO_TEST_SUITE_END()
