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

int egomotion(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	cv::Mat &rotation_vector,
	cv::Mat &translation_vector,
	std::vector<char> matches_mask) {

	if(matches_mask.empty()) {
		matches_mask.assign(matches.size(), 1);
	} else if(matches_mask.size() != matches.size()) {
		return -1;
	}

	//TODO: check dimensions of camera_matrix and distortion_coefficients

	double fx = camera_matrix.at<double>(0, 0);
	double fy = camera_matrix.at<double>(1, 1);
	if( abs(fx) <= std::numeric_limits<double>::epsilon())fx = 1.0;
	if( abs(fy) <= std::numeric_limits<double>::epsilon()) fy = 1.0;

	std::vector<cv::Point3f> object_points;
	object_points.reserve(src_keypoints.size());
	std::vector<cv::Point2f> image_points;
	image_points.reserve(dst_keypoints.size());
	for(size_t i = 0; i < matches.size(); i++) {
		if( matches_mask[i] == 0) continue;
		//FIXME
		// determine (wrong) object points
		int src_index = matches[i].queryIdx;
		const cv::KeyPoint& src_keypoint = src_keypoints[src_index];
		object_points.push_back( cv::Point3f(src_keypoint.pt.x/fx, src_keypoint.pt.y/fy, 0.0) );

		// add destination image points of matched points
		int dst_index = matches[i].trainIdx;
		const cv::KeyPoint& dst_keypoint = dst_keypoints[dst_index];
		image_points.push_back(dst_keypoint.pt);
	}

	try {
		cv::solvePnPRansac(object_points,
// 		cv::solvePnP(object_points,
			image_points,
			camera_matrix,
			distortion_coefficients,
			rotation_vector,
			translation_vector,
// 			true ); //use extrinsic guess
			false ); //use extrinsic guess
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

//TODO
void filter_lis(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		std::vector<std::vector<cv::DMatch> > &matches) {

	unsigned int num_of_matches = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		num_of_matches += matches[i].size();
	}
// 	vector< pair<int, int> > hor_sorted_match_indicies(num_of_matches);
// 	for(size_t i = 0; i < matches.size(); i++) {
// 		num_of_matches += matches[i].size();
// 	}
	
// 	vector<cv::DMatch> hor_sorted_matches;
// 	for(size_t i = 0; i < matches.size(); i++) {
// 		hor_sorted_matches.push_back(matches[i]);
// 	}
// 	sort(hor_sorted_matches.begin(), hor_sorted_matches.end(), sort_matches_horizontal);


	unsigned int index_counter = 0;
	std::vector<int> row_indicies(matches.size(), 0);
	while (index_counter < num_of_matches) {
		int min_row_index = 0;
		float minimum = 0.0;
		for(size_t i = 0; i < matches.size(); i++) {
			int keypoint_index = matches[i][row_indicies[i]].queryIdx;
			if( src_keypoints[keypoint_index].pt.x < minimum ) {
				minimum = src_keypoints[keypoint_index].pt.x;
				min_row_index = i;
			}
		}
		matches[min_row_index][row_indicies[min_row_index]].imgIdx = index_counter++;
		row_indicies[min_row_index]++;
	}

	std::vector<cv::DMatch> hor_sorted_matches;
	row_indicies =  std::vector<int>(matches.size(), 0);
	while(hor_sorted_matches.size() < num_of_matches) {
		int min_row_index = 0;
		float minimum = 0.0;
		for(size_t i = 0; i < matches.size(); i++) {
			int keypoint_index = matches[i][row_indicies[i]].trainIdx;
			if( dst_keypoints[keypoint_index].pt.x < minimum ) {
				minimum = dst_keypoints[keypoint_index].pt.x;
				min_row_index = i;
			}
		}
 		hor_sorted_matches.push_back( matches[min_row_index][row_indicies[min_row_index]] );
		row_indicies[min_row_index]++;
	}
}

void filter_matches_by_backward_matches(const std::vector<cv::DMatch> &matches,
		const std::vector<cv::DMatch> &backward_matches,
		std::vector<char> mask) {

	if(matches.size() != mask.size()) return;

int counter = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		int src_index = matches[i].queryIdx;
		int dst_index = matches[i].trainIdx;

		mask[i] = 0;
counter++;
		// begin inner loop
		for(size_t j = 0; j<backward_matches.size(); j++) {
			if(backward_matches[j].queryIdx == dst_index
			&& backward_matches[j].trainIdx == src_index) {
				mask[i] = 1;
counter--;
				//stop inner loop
				break;
			}
		}
	}
	
dout << "filtered " << counter << " matches by backward matching" << std::endl;
}

//FIXME: improve performance
void filter_matches_by_distribution(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> mask) {

	if(matches.size() != mask.size());
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
	std::vector<cv::DMatch> filtered_matches;
	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;

		if( abs(match_length[i] - mean_length) >= s_dev_length
		|| abs(match_angle[i] - mean_angle) >= s_dev_angle ) {
			mask[i] = 0;
counter++;
		}
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

void keypoints_to_objectpoints(const std::vector<cv::KeyPoint>& keypoints,
	const cv::Mat& camera_matrix,
	const float distance,
	std::vector<cv::Point3f>& objectpoints) {

	//TODO: test camera_matrix for beeing 3x3 double
	//TODO: have a look at cvConvertPointsHomogenious

	double fx = camera_matrix.at<double>(0, 0);
	double fy = camera_matrix.at<double>(1, 1);
	//FIXME: check for |fx| <= 0+eps
	if(fx == 0.0) fx = 1.0;
	if(fy == 0.0) fy = 1.0;
	for(std::vector<cv::KeyPoint>::const_iterator kp_iter = keypoints.begin();
		kp_iter != keypoints.end();
		++kp_iter) {

		//TODO: use distance for z
		objectpoints.push_back( cv::Point3f( (kp_iter->pt).x/fx, (kp_iter->pt).y/fy, 0) );
	}

// 	cv::Mat point_matrix = cv::Mat(image_points).reshape(1).t();
// 	cv::Mat object_matrix = inv_cam_matrix*point_matrix;
}


} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

