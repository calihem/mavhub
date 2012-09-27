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

/**
 * Estimate 3D motion from inverse projected image points
 */
cv::Point3f feature_movement(const std::vector<cv::Point3f> &objectpoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	std::vector<char> mask);

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

void filter_matches_by_robust_distribution(const std::vector<cv::KeyPoint> &src_keypoints,
		const std::vector<cv::KeyPoint> &dst_keypoints,
		const std::vector<cv::DMatch> &matches,
		std::vector<char> &mask);

template <typename Distance>
int filter_matches_by_imu(const std::vector<cv::KeyPoint>& src_keypoints,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<std::vector<cv::DMatch> >& matches,
	const cv::Point& center,
	const float rad_roll, const float rad_pitch, const float rad_yaw,
	const float delta_x, const float delta_y,
	std::vector<uint8_t>& filter,
	Distance distance_metric = Distance() );

void fusion_matches(const std::vector<std::vector<cv::DMatch> > &forward_matches,
		    const std::vector<std::vector<cv::DMatch> > &backward_matches,
		    std::vector<std::vector<cv::DMatch> > &matches);

void imagepoints_to_objectpoints(const std::vector<cv::Point2f>& imagepoints,
	const float distance,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients,
	std::vector<cv::Point3f>& objectpoints);

void keypoints_to_objectpoints(const std::vector<cv::KeyPoint>& keypoints,
	const float distance,
	std::vector<cv::Point3f>& objectpoints,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

/**
 * \brief Get mask from matches
 *
 * \param[in] num_src_kps number of source (train) keypoints
 * \param[in] num_dst_kps number of destination (query) keypoints
 * \param[in] matches vector of matches
 * \return Mask matrix 
 */
cv::Mat matchesmask(const int num_src_kps,
	const int num_dst_kps,
	const std::vector<cv::DMatch> &matches);
/**
 * \brief Projects 3D point coordinates to their 2D ideal image coordinates.
 */
void objectpoints_to_idealpoints(const std::vector<cv::Point3f>& objectpoints,
	const std::vector<float>& camera_pose,
	std::vector<cv::Point2f>& idealpoints);

/**
 * \brief Projects 3D point coordinates to their 2D image coordinates.
 * 
 * This function is almost similar to cv::projectPoints except that it doesn't
 * support distorted image points.
 * \param[in] objectpoints Vector of 3D object points.
 * \param[in] rotation_vector 3D rotation vector containing euler angles (rad).
 * \param[in] translation_vector 3D translation vector.
 * \param[in] camera_matrix Matrix of intrinsic parameters
 * \param[out] imagepoints Outputvector containing the 2D projected object points.
 */
void objectpoints_to_imagepoints(const std::vector<cv::Point3f>& objectpoints,
	const std::vector<float>& rotation_vector,
	const std::vector<float>& translation_vector,
	const cv::Mat& camera_matrix,
	std::vector<cv::Point2f>& imagepoints);

template <typename T>
T min_eigenval(const T &dxx, const T &dxy, const T &dyy);

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

/**
 * Undistort 2D image point using camera matrix and distotion coefficients.
 */ 
cv::Point2f undistort(const cv::Point2f &point,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

cv::Point2f undistort_n2n(const cv::Point2f &point,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

cv::Point2f undistort_n2i(const cv::Point2f &point,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients);

cv::Point2f undistort_i2i(const cv::Point2f &point,
	const cv::Mat& distortion_coefficients);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
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
inline T min_eigenval(const T &dxx, const T &dxy, const T &dyy) {
	return 0.5 * (dxx + dyy - sqrt( (dxx + dyy) * (dxx + dyy) - 4 * (dxx * dyy - dxy * dxy) ));
}

template <typename T>
T shi_tomasi_score(const cv::Mat &image, const int x, const int y, const int box_radius) {
	//TODO: optional range check
	//check coordinate range
	if(x <= box_radius || x >= image.cols-box_radius-1
	|| y <= box_radius || y >= image.rows-box_radius-1)
		return 0.0;

	int16_t dx, dy;
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
	T min_lambda = min_eigenval<T>(dxx, dxy, dyy);

	//thus we have used unscaled version to calculate derivatives, we have to do it now
	int num_pixels = 4*( (2*box_radius+1)*(2*box_radius+1) );
	//num_pixels is non zero, so it is safe to divide here
	return min_lambda / static_cast<T>(num_pixels);
}

inline cv::Point2f undistort_i2i(const cv::Point2f &point,
	const cv::Mat& distortion_coefficients) {

	//check vector dimension
	assert(distortion_coefficients.total() == 5);
	const double *k = distortion_coefficients.ptr<double>();

	double x = point.x;
	double y = point.y;
	const double x0 = x, y0 = y;

	//undistort points using simplified iterative approach from OpenCV
	for(uint8_t j=0; j<5; j++) {
		const double xx = x*x;
		const double xy = x*y;
		const double yy = y*y;
		const double r2 = xx+yy;

		const double radial_factor = 1.0/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
		const double tang_x = 2*k[2]*xy + k[3]*(r2 + 2*xx);
		const double tang_y = k[2]*(r2 + 2*yy) + 2*k[3]*xy;

		x = (x0 - tang_x)*radial_factor;
		y = (y0 - tang_y)*radial_factor;
	}

	return cv::Point2f(x, y);
}

inline cv::Point2f undistort_n2i(const cv::Point2f &point,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients) {

	const double cx = camera_matrix.at<double>(0, 2);
	const double cy = camera_matrix.at<double>(1, 2);
	const double fx = camera_matrix.at<double>(0, 0);
	const double fy = camera_matrix.at<double>(1, 1);

	return undistort_i2i(cv::Point2f((point.x - cx) / fx, (point.y - cy) / fy), distortion_coefficients);
}

inline cv::Point2f undistort_n2n(const cv::Point2f &point,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients) {

	cv::Point2f undist_point = undistort_n2i(point, camera_matrix, distortion_coefficients);

	const double cx = camera_matrix.at<double>(0, 2);
	const double cy = camera_matrix.at<double>(1, 2);
	const double fx = camera_matrix.at<double>(0, 0);
	const double fy = camera_matrix.at<double>(1, 1);

	undist_point.x = undist_point.x * fx + cx;
	undist_point.y = undist_point.y * fy + cy;

	return undist_point;
}

inline cv::Point2f undistort(const cv::Point2f &point,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients) {

	return undistort_n2n(point, camera_matrix, distortion_coefficients);
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif // _HUB_FEATURES_H_
