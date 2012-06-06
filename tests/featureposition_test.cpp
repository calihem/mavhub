#include "lib/slam/features.h"
#include "lib/slam/pose.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/detail/unit_test_parameters.hpp>

#include <opencv2/highgui/highgui.hpp>	//imread
#include <brisk/brisk.h>

#include <cstdlib>
#include <limits>	//epsilon

#include "utility.h"

#define VISUAL_OUTPUT 1

using namespace boost::unit_test;
using namespace hub::slam;
using namespace mavhub;

BOOST_AUTO_TEST_SUITE(FeaturePositionTestSuite)

BOOST_AUTO_TEST_CASE(test_featureposition)
{
	//FIXME: avoid static filenames
	cv::Mat first_image = cv::imread("../tests/images/snap66_0.jpg", 0);
	cv::Mat second_image = cv::imread("../tests/images/snap66_10.jpg", 0);
	BOOST_REQUIRE(first_image.data && second_image.data);

	uint64_t start_time = get_time_us();

	// get features from image
	cv::BriskFeatureDetector feature_detector(60, 3); //threshold, octaves
	std::vector<cv::KeyPoint> first_keypoints, second_keypoints;
	feature_detector.detect(first_image, first_keypoints);
	feature_detector.detect(second_image, second_keypoints);
	BOOST_CHECK( !first_keypoints.empty() && !second_keypoints.empty() );

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912, -0.287, 0.001889, 0.001793, 0.09828);

	// calculate corresponding 3D object points
	std::vector<cv::Point3f> objectpoints;
	keypoints_to_objectpoints(first_keypoints,
		66, //distance from ground
		objectpoints,
		camera_matrix,
		distortion_coefficients);

	// calculate descriptors which can remove or add some of the keypoints
	cv::BriskDescriptorExtractor descriptor_extractor;
	cv::Mat first_descriptors, second_descriptors;
	descriptor_extractor.compute(first_image, first_keypoints, first_descriptors);
	descriptor_extractor.compute(second_image, second_keypoints, second_descriptors);
	BOOST_CHECK( !first_keypoints.empty() && !second_keypoints.empty() );

	// match descriptors
	cv::BruteForceMatcher<cv::HammingSse> matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(first_descriptors, second_descriptors, matches);
	BOOST_CHECK( !matches.empty() );

	std::vector<char> matches_mask(matches.size(), 1);

	filter_matches_by_robust_distribution(first_keypoints, second_keypoints, matches, matches_mask);

	std::vector<float> parameter_vector(6, 0);
	int rc = estimate_pose(objectpoints,
		second_keypoints,
		matches,
		camera_matrix,
		distortion_coefficients,
		parameter_vector,
		matches_mask);
	BOOST_CHECK(rc > 0);

	uint64_t stop_time = get_time_us();

	BOOST_TEST_MESSAGE("estimated pose: "
		<< parameter_vector[0] <<", " << parameter_vector[1] <<", " << parameter_vector[2] << ", " 
		<< parameter_vector[3] <<", " << parameter_vector[4] <<", " << parameter_vector[5]
	);
	BOOST_TEST_MESSAGE("position estimation took: " << (stop_time-start_time)/1000 << "ms");

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
