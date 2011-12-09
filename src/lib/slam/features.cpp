#include "features.h"

#include <iostream>     // cout
#include <iomanip>	//setprecision

#define min_eigenval(dxx, dxy, dyy) 0.5 * (dxx + dyy - sqrt( (dxx + dyy) * (dxx + dyy) - 4 * (dxx * dyy - dxy * dxy) ))

#ifdef HAVE_OPENCV_CV_H

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
			image_points,
			camera_matrix,
			distortion_coefficients,
			rotation_vector,
			translation_vector);
	}
	catch(cv::Exception &e) {
		rotation_vector = cv::Scalar(0);
		translation_vector = cv::Scalar(0);
	}
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

#endif // HAVE_OPENCV_CV_H
