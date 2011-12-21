#include "features.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <iostream>     // cout
#include <iomanip>	//setprecision

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

void determine_egomotion(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<std::vector<cv::DMatch> >& matches,
	cv::Mat &camera_matrix,
	cv::Mat &distortion_coefficients,
	cv::Mat &rotation_vector,
	cv::Mat &translation_vector) {

	double fx = camera_matrix.at<double>(0, 0);
	double fy = camera_matrix.at<double>(1, 1);
	//FIXME: check for |fx| <= 0+eps
	if(fx == 0.0) fx = 1.0;
	if(fy == 0.0) fy = 1.0;

	std::vector<cv::Point3f> object_points;
	object_points.reserve(src_keypoints.size());
	std::vector<cv::Point2f> image_points;
	image_points.reserve(dst_keypoints.size());
	for(size_t i = 0; i < matches.size(); i++) {
		for(size_t j = 0; j < matches[i].size(); j++) {
			//FIXME
			// determine (wrong) object points
			int src_index = matches[i][j].queryIdx;
			const cv::KeyPoint& src_keypoint = src_keypoints[src_index];
			object_points.push_back( cv::Point3f(src_keypoint.pt.x/fx, src_keypoint.pt.y/fy, 0.0) );

			// add destination image points of matched points
			int dst_index = matches[i][j].trainIdx;
			const cv::KeyPoint& dst_keypoint = dst_keypoints[dst_index];
			image_points.push_back(dst_keypoint.pt);
		}
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
// 		rotation_vector = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
// 		rotation_vector.copyTo(translation_vector);
	}
}

void filter_landmarks(const landmarks_t &landmarks, cv::Mat &mask) {
	for(unsigned int i = 0; i < landmarks.counters.size(); i++) {
std::cout << i << ": " << landmarks.counters[i] << std::endl;
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
					if(backward_matches[ii][jj].queryIdx == dst_index) {
						if(backward_matches[ii][jj].trainIdx == src_index)
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

