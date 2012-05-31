#include "lib/slam/features.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/detail/unit_test_parameters.hpp>

#include <opencv2/highgui/highgui.hpp>	//imread
#include <brisk/brisk.h>

#include <cstdlib>
#include <limits>	//epsilon

#define VISUAL_OUTPUT 0

using namespace boost::unit_test;
using namespace hub::slam;

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

BOOST_AUTO_TEST_SUITE(FeatureMatchingTestSuite)

BOOST_AUTO_TEST_CASE(Test_featurematching)
{
	unit_test_log.set_threshold_level( log_messages );

	//FIXME: avoid static filenames
	cv::Mat left_image = cv::imread("../tests/images/allgaeu_left.jpg", 0);
	cv::Mat right_image = cv::imread("../tests/images/allgaeu_right.jpg", 0);
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
	cv::BruteForceMatcher<cv::HammingSse> matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(left_descriptors, right_descriptors, matches);
	BOOST_CHECK( !matches.empty() );

//TODO: time analysis
	std::vector<char> matches_mask(matches.size(), 1);
	BOOST_TEST_MESSAGE("match ratio (raw): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (robust distribution): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (2x robust distribution): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	std::vector<cv::DMatch> backward_matches;
	matcher.match(right_descriptors, left_descriptors, backward_matches);
	filter_matches_by_backward_matches(matches, backward_matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (backward matching): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (bw+rd): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	// create mask for backward matching
	cv::Mat bw_mask = matchesmask(left_keypoints.size(),
		right_keypoints.size(),
		matches);
	backward_matches.clear();
	matcher.match(right_descriptors, left_descriptors, backward_matches, bw_mask.t());
	filter_matches_by_backward_matches(matches, backward_matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (subset backward matching): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (sbw+rd): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

	//reset mask
	std::fill(matches_mask.begin(), matches_mask.end(), 1);
	filter_matches_by_robust_distribution(left_keypoints, right_keypoints, matches, matches_mask);
	bw_mask = matchesmask(left_keypoints.size(),
		right_keypoints.size(),
		matches);
	backward_matches.clear();
	matcher.match(right_descriptors, left_descriptors, backward_matches, bw_mask.t());
	filter_matches_by_backward_matches(matches, backward_matches, matches_mask);
	BOOST_TEST_MESSAGE("match ratio (rd+sbw): " << match_ratio(left_keypoints, right_keypoints, matches, matches_mask) );

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

BOOST_AUTO_TEST_SUITE_END()
