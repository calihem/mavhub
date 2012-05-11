#include "features.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <iostream>     // cout
#include <iomanip>	//setprecision
#include <cmath>	//sin, cos
#include <limits>	//epsilon

#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout
#else
#define dout if(0) std::cout
#endif

#define rad2deg(r) ((r) * 180) / M_PI
#define deg2rad(d) ((d) * M_PI) / 180

namespace hub {
namespace slam {

#define min_eigenval(dxx, dxy, dyy) 0.5 * (dxx + dyy - sqrt( (dxx + dyy) * (dxx + dyy) - 4 * (dxx * dyy - dxy * dxy) ))
brisk_landmark_t::brisk_landmark_t(const cv::KeyPoint &kp,
	const cv::Point3f &op,
	const uint8_t descr[16],
	const unsigned int counter,
	const int fc) {
	//TODO
}

void landmarks_t::clear() {
	keypoints.clear();
	objectpoints.clear();
	descriptors.resize(0);
	counters.clear();
	scene_ids.clear();
}

int egomotion(const std::vector<cv::Point3f> objectpoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	cv::Mat &rotation_vector,
	cv::Mat &translation_vector,
	const bool use_extrinsic_guess,
	std::vector<char> matches_mask) {

	if(matches_mask.empty()) {
		matches_mask.assign(matches.size(), 1);
	} else if(matches_mask.size() != matches.size()) {
		return -1;
	}

	std::vector<cv::Point3f> object_points; //matched object points
	object_points.reserve( objectpoints.size() );
	std::vector<cv::Point2f> image_points; //matched destination image points
	image_points.reserve( dst_keypoints.size() );
	for(size_t i = 0; i < matches.size(); i++) {
		if( matches_mask[i] == 0) continue;

		const int src_index = matches[i].queryIdx;
		object_points.push_back( objectpoints[src_index] );

		const int dst_index = matches[i].trainIdx;
		const cv::KeyPoint& dst_keypoint = dst_keypoints[dst_index];
		image_points.push_back(dst_keypoint.pt);
	}

	try {
// 		cv::solvePnPRansac(object_points,
		cv::solvePnP(object_points,
			image_points,
			camera_matrix,
			distortion_coefficients,
			rotation_vector,
			translation_vector,
			use_extrinsic_guess );
	}
	catch(cv::Exception &e) {
		return -2;
// 		rotation_vector = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
// 		rotation_vector.copyTo(translation_vector);
	}
	
	return 0;
}

void filter_landmarks(const landmarks_t &landmarks, cv::Mat &mask) {
	for(unsigned int i = 0; i < landmarks.counters.size(); i++) {
		if(landmarks.counters[i] < 70)
			//set complete row
// 			mask.at<uint8_t>(i, 0) = 0;
			mask.row(i) = cv::Scalar(0);
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

// int counter = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		int src_index = matches[i].queryIdx;
		int dst_index = matches[i].trainIdx;

		mask[i] = 0;
// counter++;
		// begin inner loop
		for(size_t j = 0; j<backward_matches.size(); j++) {
			if(backward_matches[j].queryIdx == dst_index
			&& backward_matches[j].trainIdx == src_index) {
				mask[i] = 1;
// counter--;
				//stop inner loop
				break;
			}
		}
	}
	
// dout << "filtered " << counter << " matches by backward matching" << std::endl;
}

void filter_matches_by_robust_distribution(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
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
	if(x_distances.size() == 0) return;

	// calculate median and MAD as robust estimations of mean and standard deviation
	const float x_median = const_median(x_distances);
	const float x_mad = mad(x_distances, x_median);

	size_t distance_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		if(abs(x_distances[distance_index] - x_median) > 3*x_mad) {
			mask[i] = 0;
		}
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

	// calculate median and MAD as robust estimations of mean and standard deviation
	const float y_median = const_median(y_distances);
	const float y_mad = mad(y_distances, y_median);

	distance_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		if( abs(y_distances[distance_index] - y_median) > 3*y_mad ) {
			mask[i] = 0;
		}
		distance_index++;
	}
}

//FIXME: improve performance
void filter_matches_by_distribution(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask) {

	if(matches.size() != mask.size()) return;

	std::vector<float> match_length;
	std::vector<float> match_angle;
	cv::L2<float> euclidean_distance;
	cv::L1<float> manhattan_distance;

	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		unsigned int src_index = matches[i].queryIdx;
		unsigned int dst_index = matches[i].trainIdx;
		float length = euclidean_distance(&(src_keypoints[src_index].pt.x), &(dst_keypoints[dst_index].pt.x), 2);
		match_length.push_back(length);
		float x_length = manhattan_distance(&(src_keypoints[src_index].pt.x), &(dst_keypoints[dst_index].pt.x), 1);
		float y_length = manhattan_distance(&(src_keypoints[src_index].pt.y), &(dst_keypoints[dst_index].pt.y), 1);
		float angle;
		if (x_length > 0.0)
			angle = std::atan(y_length/ x_length);
		else
			angle = 0.0;
		match_angle.push_back(angle);
	}

	float mean_length = mean(match_length);
	float mean_angle = mean(match_angle);
	float s_dev_length = std_dev(match_length);
	float s_dev_angle = std_dev(match_angle);

int counter = 0;
	size_t index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		if( abs(match_length[index] - mean_length) > s_dev_length
		|| abs(match_angle[index] - mean_angle) > s_dev_angle ) {
			mask[i] = 0;
counter++;
		}
		index++;
	}
dout << "filtered " << counter << " matches by distribution" << std::endl;
}
   
void filter_matches_by_landmarks(const landmarks_t &landmarks, std::vector<std::vector<cv::DMatch> > &matches) {
	std::vector<std::vector<cv::DMatch> > filtered_matches;

	for(size_t i = 0; i < matches.size(); i++) {
		std::vector<cv::DMatch> row_matches;
		for(size_t j = 0; j < matches[i].size(); j++) {
			unsigned int index = matches[i][j].queryIdx;
			if(landmarks.counters[index] < 80) { // good match
				row_matches.push_back(matches[i][j]);
			} //else
// 				std::cout << index << ": " << landmarks.counters[index] << std::endl;
		}
		if(!row_matches.empty())
			filtered_matches.push_back(row_matches);
	}

	matches = filtered_matches;
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

//FIXME: improve performance
void fusion_matches(const std::vector<std::vector<cv::DMatch> > &forward_matches,
		    const std::vector<std::vector<cv::DMatch> > &backward_matches,
		    std::vector<std::vector<cv::DMatch> > &matches) {

	for(size_t i = 0; i < forward_matches.size(); i++) {
		std::vector<cv::DMatch> row_matches;
		for(size_t j = 0; j < forward_matches[i].size(); j++) {
			int src_index = forward_matches[i][j].queryIdx;
			int dst_index = forward_matches[i][j].trainIdx;

			// begin inner loops
			for(size_t ii = 0; ii < backward_matches.size(); ii++) {
				for(size_t jj = 0; jj < backward_matches[ii].size(); jj++) {
					if(backward_matches[ii][jj].queryIdx == dst_index
					&& backward_matches[ii][jj].trainIdx == src_index) {
						row_matches.push_back(forward_matches[i][j]); 
						//stop inner loops
						jj = backward_matches[ii].size();
						ii = backward_matches.size() - 1;
					}
				}
			}
		}
		if(!row_matches.empty())
			matches.push_back(row_matches);
	}
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

cv::Mat find_homography(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<std::vector<cv::DMatch> >& matches,
	int method,
	double ransac_reproj_threshold) {

	std::vector<cv::Point2f> src_points, dst_points;

	int index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		src_points.resize(src_points.size() + matches[i].size());
		dst_points.resize(src_points.size());
		for(size_t j = 0; j < matches[i].size(); j++) {
			src_points[index] = src_keypoints[matches[i][j].queryIdx].pt; 
			dst_points[index] = dst_keypoints[matches[i][j].trainIdx].pt; 

			index++;
		}
	}

	if(src_points.size() <= 3 || src_points.size() != dst_points.size())
		return (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	return cv::findHomography(src_points, dst_points, method, ransac_reproj_threshold);
}

void imagepoints_to_objectpoints(const std::vector<cv::Point2f>& imagepoints,
	const std::vector<float>& distances,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients) {

	if( imagepoints.size() == 0) return;
	if( imagepoints.size() != distances.size() ) return;

	objectpoints.resize( imagepoints.size() );
	std::vector<cv::Point2f> undistorted_points( imagepoints.size() );

	// undistortPoints returns ideal point coordinates, i.e. x = undistorted_x * fx + cx and y = undistorted_y * fy + cy 
	undistortPoints(imagepoints, undistorted_points, camera_matrix, distortion_coefficients);
	for(unsigned int i = 0; i < undistorted_points.size(); i++) {
		//for ideal point coordinates it is enough to multiply with the distance
		const float x = undistorted_points[i].x * distances[i];
		const float y = undistorted_points[i].y * distances[i];
		objectpoints[i] = cv::Point3f(x, y, distances[i]);
	}
}

void keypoints_to_objectpoints(const std::vector<cv::KeyPoint>& keypoints,
	const float distance,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients) {

	std::vector<cv::Point2f> imagepoints( keypoints.size() );
	for(std::vector<cv::KeyPoint>::const_iterator kp_iter = keypoints.begin();
		kp_iter != keypoints.end();
		++kp_iter) {
		
		imagepoints.push_back(kp_iter->pt);
	}
	imagepoints_to_objectpoints(imagepoints,
		distance,
		objectpoints,
		camera_matrix,
		distortion_coefficients);
}

// simple (and not working) algo to determine rotation
float yaw(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	std::vector<char> mask) {

	if(matches.size() != mask.size()) return 0.0;

	const cv::Point2f center(camera_matrix.at<double>(0, 2), camera_matrix.at<double>(1, 2));
	cv::L2<float> euclidean_distance;

	std::vector<float> radian_meassures; // normed to radius 1
	std::vector<float> x_distances;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;
		const float b = euclidean_distance(&(src_keypoints[src_index].pt.x), &(dst_keypoints[dst_index].pt.x), 2);
		const float l1 = euclidean_distance(&(src_keypoints[src_index].pt.x), &(center.x), 2);
		const float l2 = euclidean_distance(&(dst_keypoints[dst_index].pt.x), &(center.x), 2);
		const float radius = (l1 + l2)/2;
// dout << "P1: (" << src_keypoints[src_index].pt.x << ", " << src_keypoints[src_index].pt.y << ") "
// 	<< "P2: (" << dst_keypoints[dst_index].pt.x << ", " << dst_keypoints[dst_index].pt.y << ") "
// 	<< "b " << b
// 	<< " radius " << radius
// 	<< std::endl;
		if(radius >= 5.0) //ignore movement near rotation center
			// use intercept theorem to normalize rotation movement to radius 1
			radian_meassures.push_back(b/radius);
// 		if(radius <= 1.0)
// 			radian_meassures.push_back(b);
// 		else
// 			radian_meassures.push_back(b/(radius-1));
	}
	if(radian_meassures.size() == 0) return 0.0;
// for(size_t i = 0; i < radian_meassures.size(); i++) {
// 	dout << radian_meassures[i] << " ";
// }
// dout << std::endl;
	const float r_median = median(radian_meassures);
// 	const float ret = rad2deg( acos(1-r_median/2) );
	const float ret = 2*rad2deg( asin(r_median/2) );

// dout << "r_median: " << r_median << ", ret: " << ret << std::endl;
	return ret;
}


} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

