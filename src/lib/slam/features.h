#ifndef _HUB_FEATURES_H_
#define _HUB_FEATURES_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2

#include <opencv/cv.h>	//cv::Mat

#if (CV_MINOR_VERSION >= 2)

#include <opencv2/features2d/features2d.hpp>	//DescriptorMatcher

namespace hub {
namespace slam {

struct brisk_landmark_t {
	brisk_landmark_t(const cv::KeyPoint &kp,
		const cv::Point3f &op,
		const uint8_t descr[16],
		const unsigned int counter,
		const int fc);
	cv::KeyPoint keypoint;	// 2D coordinate, octave, ...
	cv::Point3f object_point;	// 3D coordinate
	uint8_t descriptor[16];	// 128 bit value
	unsigned int counter;	// how often occured this landmark in a scene
	int first_occurence;	// index of first scene in which the landmark occured
};

struct landmarks_t {
	void clear();
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> objectpoints;
	cv::Mat descriptors;
	std::vector<int> counters; //FIXME: needed?
	std::vector<int> scene_ids; //FIXME: needed?
};

/**
 * \brief Determine egomotion based on feature matches.
 * \param[in] objectpoints
 * \param[in] dst_keypoints
 * \param[in] matches
 * \param[in] camera_matrix
 * \param[in] distortion_coefficients
 * \param[in,out] rotation_vector
 * \param[in,out] translation_vector
 * \param[in] use_extrinsic_guess Use \a rotation_vector and \a translation_vector as initial guess.
 * \param[in] matches_mask matches.at(i) will only be considered if matches_mask.at(i) is non-zero
 */
int egomotion(const std::vector<cv::Point3f> objectpoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	cv::Mat &rotation_vector,
	cv::Mat &translation_vector,
	const bool use_extrinsic_guess = false,
	std::vector<char> matches_mask = std::vector<char>() );


/// Filter out landmarks which have a small counter value 
void filter_landmarks(const landmarks_t &landmarks, cv::Mat &mask);

/// Increase counter value of landmark if it was matched
void update_landmarks(landmarks_t &landmarks,const std::vector<std::vector<cv::DMatch> > &matches);

/// Filter out matches 
void filter_ambigous_matches(std::vector<std::vector<cv::DMatch> > &matches);

void find_lis(const std::vector<int> &sequence, std::vector<int> &lis);

void filter_matches_by_lis(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask);

/// Filter out matches which have no corresponding backward match.
void filter_matches_by_backward_matches(const std::vector<cv::DMatch> &matches,
		const std::vector<cv::DMatch> &backward_matches,
		std::vector<char> &mask);

void filter_matches_by_robust_distribution(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask);

void filter_matches_by_distribution(const std::vector<cv::KeyPoint> src_keypoints,
		const std::vector<cv::KeyPoint> dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask);

void filter_matches_by_landmarks(const landmarks_t &landmarks, std::vector<std::vector<cv::DMatch> > &matches);

template <typename Distance>
int filter_matches_by_imu(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<std::vector<cv::DMatch> >& matches,
	const cv::Point& center,
	const float rad_roll, const float rad_pitch, const float rad_yaw,
	const float delta_x, const float delta_y,
	std::vector<uint8_t>& filter,
	Distance distance_metric = Distance() );

cv::Mat find_homography(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<std::vector<cv::DMatch> >& matches,
	int method=0,
	double ransac_reproj_threshold=3);

void fusion_matches(const std::vector<std::vector<cv::DMatch> > &forward_matches,
		    const std::vector<std::vector<cv::DMatch> > &backward_matches,
		    std::vector<std::vector<cv::DMatch> > &matches);

void imagepoints_to_objectpoints(const std::vector<cv::Point2f>& imagepoints,
	const float distance,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

void imagepoints_to_objectpoints(const std::vector<cv::Point2f>& imagepoints,
	const std::vector<float>& distances,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

void keypoints_to_objectpoints(const std::vector<cv::KeyPoint>& keypoints,
	const float distance,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

/**
 * \brief Calculate the Shi-Tomasi score.
 *
 * Computes the minimal eigenvalue of the Hessian matrix. A
 * larger minimal eigenvalue means a better feature. For
 * further informations have a look at Harris corner
 * detector. 
 * \param image The image to analyse
 * \param x x-component of image point
 * \param y y-component of image point
 * \param box_radius The radius of the box building the subimage for
 * which the derivatives are calculated. A value of 1 should be fine.
 * \return Minimal eigenvalue. A good threshold for detecting good
 * features should be in the range from 40 to 150.
 */
template <typename T>
T shi_tomasi_score(const cv::Mat &image, const int x, const int y, const int box_radius);

cv::Point2f transform_affine(const cv::Point2f &point, const cv::Mat &transform_matrix);

float yaw(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	std::vector<char> mask);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
inline void imagepoints_to_objectpoints(const std::vector<cv::Point2f>& imagepoints,
	const float distance,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients) {
	
	std::vector<float> distances(imagepoints.size(), distance);
	imagepoints_to_objectpoints(imagepoints,
		distances,
		objectpoints,
		camera_matrix,
		distortion_coefficients);
}

template <typename T>
T mean(const std::vector<T> &values) {
	if(values.size() == 0) return 0;

	T sum = 0;
	for(typename std::vector<T>::const_iterator i=values.begin(); i != values.end(); ++i) {
		sum += *i;
	}
	return sum / values.size();
}

/**
 * \brief Calculate median using selection algorithm in O(n)
 */
template <typename T>
T const_median(const std::vector<T> &values) {
	typename std::vector<T> values_copy(values);
	return median(values_copy);
}

template <typename T>
T median(std::vector<T> &values) {
	typename std::vector<T>::iterator first = values.begin();
	typename std::vector<T>::iterator last = values.end();
	typename std::vector<T>::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last);
	return *middle;
}

/**
 * \brief Calculate the median absolute deviation.
 * \param[in] values A vector of values.
 * \param[in] median The median of the given values.
 */
template <typename T>
T mad(const std::vector<T> &values, const T median) {
	if(values.size() == 0)
		return 0;

	std::vector<T> absolute_deviations;
	absolute_deviations.reserve(values.size());
	for(size_t i=0; i<values.size(); i++) {
		absolute_deviations.push_back( abs(values[i] - median) );
	}
	return hub::slam::median(absolute_deviations);
}

template <typename T>
T std_dev(const std::vector<T> &values) {
	if(values.size() <= 1) return 0;

	T m = mean(values);
	T sum = 0;
	for(typename std::vector<T>::const_iterator i=values.begin(); i != values.end(); ++i) {
		sum += std::pow(*i-m, 2);
	}
	return sqrt( sum/(values.size()-1) );
}

template <typename Distance>
int filter_matches_by_imu(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<std::vector<cv::DMatch> >& matches,
	const cv::Point& center,
	const float rad_roll, const float rad_pitch, const float rad_yaw,
	const float delta_x, const float delta_y,
	std::vector<uint8_t>& filter,
	Distance distance_metric) {

	double alpha = cos(rad_yaw);
	double beta = sin(rad_yaw);

	/* Transformation matrix
	 * ( alpha beta  (1-alpha)*center.x-beta*center.y-delta_x )
	 * ( -beta alpha beta*center.x-(1-alpha)*center.y-delta_y )
	 */
	cv::Mat transform_matrix(2, 3, CV_64F);
	double *matrix_data = reinterpret_cast<double*>(transform_matrix.data);
	matrix_data[0] = alpha;
	matrix_data[1] = beta;
	matrix_data[2] = (1-alpha)*center.x - beta*center.y - delta_x;
	matrix_data[3] = -beta;
	matrix_data[4] = alpha;
	matrix_data[5] = beta*center.x - (1-alpha)*center.y-delta_y;

// 	filter.resize(src_keypoints.size() * dst_keypoints.size());
	int rc = 0;
	int index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		filter.resize(filter.size() + matches[i].size());
		for(size_t j = 0; j < matches[i].size(); j++) {
			int src_index = matches[i][j].queryIdx;
			int dst_index = matches[i][j].trainIdx;
			
			const cv::KeyPoint& src_keypoint = src_keypoints[src_index];
			const cv::KeyPoint& dst_keypoint = dst_keypoints[dst_index];

			cv::Point2f transformed = transform_affine(dst_keypoint.pt, transform_matrix);
			typename Distance::ResultType distance = distance_metric(&src_keypoint.pt.x, &transformed.x, 2);
// 			std::cout << "distance of (" << src_keypoint.pt.x << ", " << src_keypoint.pt.y << ") and ("
// 				<< transformed.x << ", " << transformed.y << ") is " << distance << std::endl;
			if(distance < 5) {
				filter.at(index) = 1;
				rc++;
			} else {
				filter.at(index) = 0;
			}
			index++;
		}
	}

	return rc;
}

template <typename T>
T shi_tomasi_score(const cv::Mat &image, const int x, const int y, const int box_radius) {
	//TODO: optional range check
	//check coordinate range
	if(x <= box_radius || x >= image.cols-box_radius-1
	|| y <= box_radius || y >= image.rows-box_radius-1)
		return 0.0;

	int32_t dx, dy;
	int32_t dxx = 0, dxy = 0, dyy = 0;
	//iterate through box (window)
	for(int i=y-box_radius; i <= y+box_radius; i++) {
		for(int j=x-box_radius; j <= x+box_radius; j++) {
			dx = image.at<uint8_t>(i, j+1) - image.at<uint8_t>(i, j-1);
			dy = image.at<uint8_t>(i+1, j) - image.at<uint8_t>(i-1, j);
			dxx += dx*dx; dxy += dx*dy; dyy += dy*dy;
		}
	}

	//calculate minimal eigenvalue
	T t_dxx = static_cast<T>(dxx);
	T t_dxy = static_cast<T>(dxy);
	T t_dyy = static_cast<T>(dyy);
	T min_lambda = min_eigenval(t_dxx, t_dxy, t_dyy);

	//thus we have used unscaled version to calculate derivatives, we have to do it now
	int num_pixels = (2*box_radius+1)*(2*box_radius+1);
	return min_lambda / (4.0*num_pixels);
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif // _HUB_FEATURES_H_
