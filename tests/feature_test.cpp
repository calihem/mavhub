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

#define PRECISION float
// #define PRECISION double

static const PRECISION DISTANCE = 50;

using namespace boost::unit_test;
using namespace hub;
using namespace hub::slam;

template<typename T>
inline T ssd(const T &a, const T &b) {
	return (a-b)*(a-b);
}

template<typename T>
inline T ssd(const cv::Point_<T> &point_a, const cv::Point_<T> &point_b) {
	T sq_diff = (point_a.x-point_b.x)*(point_a.x-point_b.x);
	sq_diff += (point_a.y-point_b.y)*(point_a.y-point_b.y);

	return sq_diff;
}

template<typename T>
inline T ssd(const cv::Point3_<T> &point_a, const cv::Point3_<T> &point_b) {
	T sq_diff = (point_a.x-point_b.x)*(point_a.x-point_b.x);
	sq_diff += (point_a.y-point_b.y)*(point_a.y-point_b.y);
	sq_diff += (point_a.z-point_b.z)*(point_a.z-point_b.z);

	return sq_diff;
}

template<typename PointT, typename T>
T error(const std::vector< PointT >& points_a,
	const std::vector< PointT >& points_b) {
	
	if( points_a.size() != points_b.size() )
		return -1;
	
	T rc = 0;
	for(size_t i=0; i<points_a.size(); i++) {
		rc += sqrt( ssd<T>(points_a[i], points_b[i]) );
	}

	return rc;
}

double match_ratio(const std::vector<cv::KeyPoint> &src_keypoints,
	const std::vector<cv::KeyPoint> &dst_keypoints,
	std::vector<cv::DMatch> &matches,
	const std::vector<char> &mask) {

	if(matches.empty()) return 0.0;
	BOOST_CHECK(matches.size() == mask.size());

	unsigned int valid_counter = 0;
	unsigned int invalid_counter = 0;
	for(unsigned int i=0; i<matches.size(); i++) {
		if(mask[i] == 0) continue;

		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;

		const int x_diff = src_keypoints[src_index].pt.x - dst_keypoints[dst_index].pt.x;
		const int y_diff = src_keypoints[src_index].pt.y - dst_keypoints[dst_index].pt.y;

		//allgaeu_left and allgaeu_right are shifted by approx. 5px in horizontal direction
		if( abs(y_diff) == 0
		&& abs(x_diff) >= 4
		&& abs(x_diff) <= 6)
			valid_counter++;
		else
			invalid_counter++;
	}
	
	return double(valid_counter)/(valid_counter+invalid_counter);
}

BOOST_AUTO_TEST_SUITE(hub_feature_tests)

BOOST_AUTO_TEST_CASE(Test_features)
{

	std::vector< cv::Point3_<PRECISION> > objectpoints;
	objectpoints.push_back( cv::Point3_<PRECISION>(0, 0, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(-20, -15, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(-25, 35, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(22, 17, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(10, -20, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(-15, 6, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(5, -28, DISTANCE) );

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);
// 	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912, -0.287, 0.001889, 0.001793, 0.09828);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

// 	std::vector<PRECISION> pose_vector(6, 0);
	std::vector<PRECISION> pose_vector(6);
	pose_vector[0] = 0.1;	//phi
	pose_vector[1] = 0.2;	//theta
	pose_vector[2] = 0.3;	//psi
	pose_vector[3] = 1.0;	//t1
	pose_vector[4] = -1.0;	//t2
	pose_vector[5] = 0.5;	//t3

	PRECISION e;

	//
	// project points
	//
	std::vector< cv::Point_<PRECISION> > imagepoints( objectpoints.size() );
	pinhole_model_euler(
		reinterpret_cast<PRECISION*>( &(objectpoints[0].x) ),
		&pose_vector[0],
		camera_matrix,
		reinterpret_cast<PRECISION*>( &(imagepoints[0].x) ),
		objectpoints.size() );
// 	objectpoints_to_imagepoints(objectpoints,
// 	objectpoints_to_idealpoints(objectpoints,
// 		pose_vector,
// 		camera_matrix,
// 		imagepoints);

	//
	// test undistort_n2i of cv::KeyPoint
	//
	// fill keypoints with coordinates of imagepoints
	std::vector<cv::KeyPoint> keypoints( imagepoints.size() );
	for(unsigned int i=0; i<imagepoints.size(); i++) {
		keypoints[i].pt = imagepoints[i];
	}
	std::vector< cv::Point_<PRECISION> > undistorted_idealpoints;
	undistort_n2i(keypoints,
		camera_matrix,
		distortion_coefficients,
		undistorted_idealpoints);
	std::vector< cv::Point_<PRECISION> > idealpoints( imagepoints.size() );
	imagepoints_to_idealpoints(&imagepoints[0].x, camera_matrix, &idealpoints[0].x, imagepoints.size());
	e = error<cv::Point_<PRECISION>, PRECISION>(undistorted_idealpoints, idealpoints);
	BOOST_TEST_MESSAGE("undistortion error: " << e);
	BOOST_CHECK(e <= 100*std::numeric_limits<PRECISION>::min());
}

BOOST_AUTO_TEST_CASE(test_featurematching) {
	//FIXME: avoid static filenames
	cv::Mat left_image = cv::imread("../tests/data/allgaeu_left.jpg", 0);
	cv::Mat right_image = cv::imread("../tests/data/allgaeu_right.jpg", 0);
	BOOST_REQUIRE(left_image.data && right_image.data);

	// get features from image
	cv::BriskFeatureDetector feature_detector(60, 3); //threshold, octaves
	std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
	feature_detector.detect(left_image, left_keypoints);
	feature_detector.detect(right_image, right_keypoints);
	BOOST_CHECK( !left_keypoints.empty() && !right_keypoints.empty() );

	// calculate descriptors which can remove or add some of the keypoints
	cv::BriskDescriptorExtractor descriptor_extractor;
	cv::Mat left_descriptors, right_descriptors;
	descriptor_extractor.compute(left_image, left_keypoints, left_descriptors);
	descriptor_extractor.compute(right_image, right_keypoints, right_descriptors);
	BOOST_CHECK( !left_keypoints.empty() && !right_keypoints.empty() );

	// match descriptors
#if CV_MINOR_VERSION <= 3
	cv::BruteForceMatcher<cv::Hamming> matcher;
#else
	cv::BFMatcher matcher(cv::NORM_HAMMING);
#endif
	std::vector<cv::DMatch> matches;
	uint64_t start_time = get_time_us();
	matcher.match(left_descriptors, right_descriptors, matches);
	uint64_t stop_time = get_time_us();
	BOOST_CHECK( !matches.empty() );

	std::vector<char> matches_mask(matches.size(), 1);
	BOOST_TEST_MESSAGE("[   brute    ] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

	start_time = get_time_us();
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[robust distr] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	start_time = get_time_us();
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[2x rob distr] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	start_time = get_time_us();
	std::vector<cv::DMatch> backward_matches;
	matcher.match(right_descriptors, left_descriptors, backward_matches);
	filter_matches_by_backward_matches(matches, backward_matches, matches_mask);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[  backward  ] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	BOOST_TEST_MESSAGE("[   bw+rd    ] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	// create mask for backward matching
	start_time = get_time_us();
	cv::Mat bw_mask = matchesmask(left_keypoints.size(),
		right_keypoints.size(),
		matches);
	backward_matches.clear();
	matcher.match(right_descriptors, left_descriptors, backward_matches, bw_mask.t());
	filter_matches_by_backward_matches(matches, backward_matches, matches_mask);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[ subset bw  ] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

	stop_time = get_time_us();
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	BOOST_TEST_MESSAGE("[   sbw+rd   ] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	start_time = get_time_us();
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	bw_mask = matchesmask(left_keypoints.size(),
		right_keypoints.size(),
		matches);
	backward_matches.clear();
	matcher.match(right_descriptors, left_descriptors, backward_matches, bw_mask.t());
	filter_matches_by_backward_matches(matches, backward_matches, matches_mask);
	stop_time = get_time_us();
	BOOST_TEST_MESSAGE("[   rd+sbw   ] ratio " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask)
		<< " | time " << stop_time-start_time) << "us";

#if VISUAL_OUTPUT
	cv::Mat match_img;
	cv::drawMatches(left_image, left_keypoints,
		right_image, right_keypoints,
		matches,
		match_img,
		cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
		matches_mask,
// 		std::vector<char>(),
		cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);

	cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
	cv::imshow("image", match_img);
	cv::waitKey();
#endif //VISUAL_OUTPUT
}


//FIXME move to pose test suite
BOOST_AUTO_TEST_CASE(test_featureposition) {
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

	estimate_translation_by_objects<float>(objectpoints,
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
	int rc = lm_pose_optimization< float, levmar_ideal_pinhole_euler<float>, levmar_approx_ideal_pinhole_euler_jac<float> >(
// 	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_ideal_pinhole_euler_jac<float> >(
		objectpoints,
		second_idealpoints,
		matches,
		parameter_vector,
		cv::Mat(),
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
