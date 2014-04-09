#include "features.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 3

#include <iostream>     // cout
#include <iomanip>	//setprecision
#include <cmath>	//sin, cos
#include <limits>	//epsilon

#include "lib/hub/math.h"

#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout
#else
#define dout if(0) std::cout
#endif

namespace hub {
namespace slam {

brisk_landmark_t::brisk_landmark_t(const cv::KeyPoint &kp,
	const cv::Point3f &op,
	const uint8_t descr[16],
	const unsigned int counter,
	const int fc) : 
		keypoint(kp),
		object_point(op),
		counter(counter),
		first_occurence(fc) {
	memcpy(&descriptor, &descr, sizeof(descriptor));
}

void landmarks_t::clear() {
	keypoints.clear();
	objectpoints.clear();
	descriptors.resize(0);
	counters.clear();
	scene_ids.clear();
}

//FIXME: improve performance
void filter_ambigous_matches(std::vector<std::vector<cv::DMatch> > &matches) {
	std::vector<std::vector<cv::DMatch> > filtered_matches;

// 	std::list<int> query_indicies, train_indicies;
	int query_index, train_index;
	for(size_t i = 0; i < matches.size(); i++) {
		std::vector<cv::DMatch> row_matches;
		for(size_t j = 0; j < matches[i].size(); j++) {
			query_index = matches[i][j].queryIdx;
			train_index = matches[i][j].trainIdx;
			bool is_ambigous = false;
			for(size_t ii = 0; ii < matches.size(); ii++) {
				for(size_t jj = 0; jj < matches[ii].size(); jj++) {
					if (train_index == matches[ii][jj].trainIdx
					&& query_index != matches[ii][jj].queryIdx)
						is_ambigous = true;
					if (query_index == matches[ii][jj].queryIdx
					&& train_index != matches[ii][jj].trainIdx)
						is_ambigous = true;
				}
			}
// 			if(is_ambigous) {
			if(!is_ambigous) {
				row_matches.push_back(matches[i][j]);
			}
// 			query_indicies.push_back(matches[i][j].queryIdx);
// 			train_indicies.push_back(matches[i][j].trainIdx);
		}
		if(!row_matches.empty())
			filtered_matches.push_back(row_matches);
	}

	matches = filtered_matches;
// 	query_indicies.sort(); train_indicies.sort();
}

//FIXME: improve performance
//TODO
void filter_matches_by_lis(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask) {

	if(matches.size() != mask.size()) return;

	int valid_before = 0;
	for(size_t i=0; i<mask.size(); i++) {
		if(mask[i]) valid_before++;
	}

	std::vector<int> sequence(matches.size());
	//determine horizontal order of matches (with respect to src_keypoints)
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) {
			continue;
		}
		const float x_min = src_keypoints[matches[i].queryIdx].pt.x; // minimum of current match
		size_t x_min_counter = 0; // how many matches are smaller than current match
		for(size_t j = 0; j < matches.size(); j++) {
			size_t kp_index = matches[j].queryIdx;
			if(src_keypoints[kp_index].pt.x < x_min) {
				x_min_counter++;
			}
		}
		sequence[i] = x_min_counter;
	}

	float last_min = 0.0;
	std::vector<int> sorted_sequence;
	sorted_sequence.reserve(matches.size());
	std::map<int,int> seq2index_map; //maps sequence index to matches index
	//sort matches (with respect to dst_keypoints)
	for(size_t i = 0; i < matches.size(); i++) {
		float x_min = std::numeric_limits<float>::max();
		size_t x_min_index = 0;
		for(size_t j = 0; j < matches.size(); j++) {
			if(mask[j] == 0) continue;
			size_t kp_index = matches[j].trainIdx;
			if(dst_keypoints[kp_index].pt.x < x_min
			&& dst_keypoints[kp_index].pt.x > last_min) {
				x_min = dst_keypoints[kp_index].pt.x;
				x_min_index = j;
			}
		}
		sorted_sequence.push_back(sequence[x_min_index]);
		seq2index_map.insert( std::make_pair(sequence[x_min_index], x_min_index));
		last_min = x_min;
	}

	std::vector<int> lis;
	find_lis(sorted_sequence, lis);
	
	mask.assign(mask.size(), 0);
	for(size_t i = 0; i < lis.size(); i++) {
		mask[ seq2index_map[lis[i]] ] = 1;
	}

	int valid_after = 0;
	for(size_t i=0; i<mask.size(); i++) {
		if(mask[i]) valid_after++;
	}
	dout << "filtered " << valid_before - valid_after << " matches by lis" << std::endl;
}

void filter_matches_by_backward_matches(const std::vector<cv::DMatch> &matches,
		const std::vector<cv::DMatch> &backward_matches,
		std::vector<char> &mask) {

	if(matches.size() != mask.size()) return;

	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		const int src_index = matches[i].queryIdx;
		const int dst_index = matches[i].trainIdx;

		mask[i] = 0; //default is to filter out

		// begin inner loop
		for(size_t j = 0; j<backward_matches.size(); j++) {
			if(backward_matches[j].queryIdx == dst_index
			&& backward_matches[j].trainIdx == src_index) {
				mask[i] = 1;
				//stop inner loop
				break;
			}
		}
	}
}

void filter_matches_by_robust_distribution(const std::vector<cv::KeyPoint> &src_keypoints,
		const std::vector<cv::KeyPoint> &dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask) {

	if(matches.size() != mask.size()) return;

	// filter in horizontal direction first (reduces the computation cost of vertical filter)
	std::vector<float> x_distances;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;
		const unsigned int x_distance = abs(src_keypoints[src_index].pt.x - dst_keypoints[dst_index].pt.x);
		x_distances.push_back(x_distance);
	}
	if(x_distances.empty()) return;

	// calculate robust standard deviation
	float median;
	float sigma = robust_sigma(x_distances, median);
	if( sigma <= std::numeric_limits<float>::min() )
		return;

	size_t distance_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		// scale distance to fit standard normal distribution
		const float z = abs(x_distances[distance_index] - median) / sigma;
		//use 0,95 interval for z
		if(z > 1.96)
			mask[i] = 0;
		distance_index++;
	}

	// filter in vertical direction
	std::vector<float> y_distances;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;
		const unsigned int y_distance = abs(src_keypoints[src_index].pt.y - dst_keypoints[dst_index].pt.y);
		y_distances.push_back(y_distance);
	}
	if(y_distances.size() == 0) return;

	// calculate robust standard deviation
	sigma = robust_sigma(y_distances, median);
	if( sigma <= std::numeric_limits<float>::min() )
		return;

	distance_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		// scale distance to fit standard normal distribution
		const float z = abs(y_distances[distance_index] - median) / sigma;
		//use 0,95 interval for z
		if(z > 1.96)
			mask[i] = 0;

		distance_index++;
	}
}

void filter_matches_by_robust_distribution(const std::vector<cv::Point2f> &src_points,
		const std::vector<cv::Point2f> &dst_points,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask) {

	if(matches.size() != mask.size()) return;

	// filter in horizontal direction first (reduces the computation cost of vertical filter)
	std::vector<float> x_distances; x_distances.reserve( src_points.size() );
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;
		const unsigned int x_distance = abs(src_points[src_index].x - dst_points[dst_index].x);
		x_distances.push_back(x_distance);
	}
	if(x_distances.empty()) return;

	// calculate robust standard deviation
	float median;
	float sigma = robust_sigma(x_distances, median);
	if( sigma <= std::numeric_limits<float>::min() )
		return;

	size_t distance_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		// scale distance to fit standard normal distribution
		const float z = abs(x_distances[distance_index] - median) / sigma;
		//use 0,95 interval for z
		if(z > 1.96)
			mask[i] = 0;
		distance_index++;
	}

	// filter in vertical direction
	std::vector<float> y_distances; y_distances.reserve( src_points.size() );
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;
		const unsigned int y_distance = abs(src_points[src_index].y - dst_points[dst_index].y);
		y_distances.push_back(y_distance);
	}
	if(y_distances.size() == 0) return;

	// calculate robust standard deviation
	sigma = robust_sigma(y_distances, median);
	if( sigma <= std::numeric_limits<float>::min() )
		return;

	distance_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		// scale distance to fit standard normal distribution
		const float z = abs(y_distances[distance_index] - median) / sigma;
		//use 0,95 interval for z
		if(z > 1.96)
			mask[i] = 0;

		distance_index++;
	}
}

void find_lis(const std::vector<int> &sequence, std::vector<int> &lis) {

	lis.clear();
	lis.push_back(0);

	std::vector<int> tmp_vec(sequence.size());
	int lower, upper;
	for(size_t i = 0; i < sequence.size(); i++) {
		// if next element is greater than last element of longest subseq
		if( sequence[lis.back()] < sequence[i] ) {
			tmp_vec[i] = lis.back();
			lis.push_back(i);
			continue;
		}
		
		// binary search to find smallest element
		for(lower = 0, upper = lis.size()-1; lower < upper;) {
			int bin_index = (lower + upper) / 2;
			if(sequence[lis[bin_index]] < sequence[i])
				lower = bin_index + 1;
			else
				upper = bin_index;
		}
		
		// update lis if new value is smaller then previously
		if(sequence[i] < sequence[lis[lower]]) {
			if(lower > 0)
				tmp_vec[i] = lis[lower-1];
			lis[lower] = i;
		}
	}
	
	for(lower = lis.size(), upper = lis.back(); lower--; upper = tmp_vec[upper])
		lis[lower] = upper;
}

void fusion_matches(const std::vector<cv::DMatch> &forward_matches,
		    const std::vector<cv::DMatch> &backward_matches,
		    std::vector<cv::DMatch> &matches) {

	for(size_t i = 0; i < forward_matches.size(); i++) {
		int src_index = forward_matches[i].queryIdx;
		int dst_index = forward_matches[i].trainIdx;

		// begin inner loop
		for(size_t j = 0; j<backward_matches.size(); j++) {
			if(backward_matches[j].queryIdx == dst_index
			&& backward_matches[j].trainIdx == src_index) {
				matches.push_back(forward_matches[i]);
				//stop inner loop
				break;
			}
		}
	}
}

void idealpoints_to_imagepoints(const std::vector<cv::Point2f>& idealpoints,
	const cv::Mat& camera_matrix,
	std::vector<cv::Point2f>& imagepoints) {

	if( idealpoints.empty() ) return;

	const double cx = camera_matrix.at<double>(0, 2);
	const double cy = camera_matrix.at<double>(1, 2);
	const double fx = camera_matrix.at<double>(0, 0);
	const double fy = camera_matrix.at<double>(1, 1);

	imagepoints.resize( idealpoints.size() );
	for(unsigned int i=0; i<idealpoints.size(); i++) {
		imagepoints[i].x = fx*idealpoints[i].x + cx;
		imagepoints[i].y = fy*idealpoints[i].y + cy;
	}
}

void imagepoints_to_idealpoints(const std::vector<cv::Point2f>& imagepoints,
	const cv::Mat& camera_matrix,
	std::vector<cv::Point2f>& idealpoints) {

	if( imagepoints.empty() ) return;

	const double cx = camera_matrix.at<double>(0, 2);
	const double cy = camera_matrix.at<double>(1, 2);
	const double fx = camera_matrix.at<double>(0, 0);
	const double fy = camera_matrix.at<double>(1, 1);

	idealpoints.resize( imagepoints.size() );
	for(unsigned int i=0; i<imagepoints.size(); i++) {
		idealpoints[i].x = (imagepoints[i].x - cx)/fx;
		idealpoints[i].y = (imagepoints[i].y - cy)/fy;
	}
}

cv::Mat matchesmask(const int num_src_kps,
	const int num_dst_kps,
	const std::vector<cv::DMatch> &matches) {

	cv::Mat mask(num_src_kps, num_dst_kps, CV_8UC1, cv::Scalar(0));
	for(unsigned int i=0; i<matches.size(); i++) {
		const int dst_index = matches[i].queryIdx;
		const int src_index = matches[i].trainIdx;
		mask.at<uchar>(dst_index, src_index) = 1;
	}

	return mask;
}

cv::Point2f transform_affine(const cv::Point2f &point, const cv::Mat &transform_matrix) {
	if(transform_matrix.rows != 2 || transform_matrix.cols != 3)
		return point;

	cv::Point2f transformed;

	const double *matrix_data = reinterpret_cast<const double*>(transform_matrix.data);
	transformed.x = matrix_data[0]*point.x + matrix_data[1]*point.y + matrix_data[2];
	transformed.y = matrix_data[3]*point.x + matrix_data[4]*point.y + matrix_data[5];
	
	return transformed;
}

void undistort_n2i(const std::vector<cv::KeyPoint> &keypoints,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	std::vector<cv::Point2f> &undistorted_points) {

	if( keypoints.empty() ) return;
	if(camera_matrix.cols != 3 || camera_matrix.rows != 3 ) return;
	if(distortion_coefficients.total() < 5) return;

	const double cx = camera_matrix.at<double>(0, 2);
	const double cy = camera_matrix.at<double>(1, 2);
	const double fx = camera_matrix.at<double>(0, 0);
	const double fy = camera_matrix.at<double>(1, 1);
	const double *k = distortion_coefficients.ptr<double>();

	undistorted_points.resize( keypoints.size() );

	double ideal_x, ideal_y;
	for(unsigned int i=0; i<keypoints.size(); i++) {
		ideal_x = (keypoints[i].pt.x-cx)/fx;
		ideal_y = (keypoints[i].pt.y-cy)/fy;
		const double x0 = ideal_x, y0 = ideal_y;

		//undistort points using simplified iterative approach from OpenCV
		for(uint8_t j=0; j<5; j++) {
			const double xx = ideal_x*ideal_x;
			const double xy = ideal_x*ideal_y;
			const double yy = ideal_y*ideal_y;
			const double r2 = xx+yy;

			const double radial_factor = 1.0/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
			const double tang_x = 2*k[2]*xy + k[3]*(r2 + 2*xx);
			const double tang_y = k[2]*(r2 + 2*yy) + 2*k[3]*xy;

			ideal_x = (x0 - tang_x)*radial_factor;
			ideal_y = (y0 - tang_y)*radial_factor;
		}

		undistorted_points[i].x = ideal_x;
		undistorted_points[i].y = ideal_y;
	}
}

void update_landmarks(landmarks_t &landmarks,const std::vector<std::vector<cv::DMatch> > &matches) {
	// increment all counters by 1
/*	for(std::vector<int>::iterator iter = landmarks.counters.begin(); iter != landmarks.counters.end(); ++iter)
	{
		(*iter)++;
	}*/
	std::vector<uint8_t> mask(landmarks.counters.size(), 0);
	for(size_t i = 0; i < matches.size(); i++) {
		for(size_t j = 0; j < matches[i].size(); j++) {
			unsigned int index = matches[i][j].queryIdx;
			landmarks.counters[index] = (landmarks.counters[index] + 101) / 2;
			mask[index] = 1;
		}
	}

	for(unsigned int i = 0; i < landmarks.counters.size(); i++) {
		if(mask[i] == 0)
			landmarks.counters[i] /= 2;
	}
}


} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

