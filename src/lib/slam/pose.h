#ifndef _HUB_POSE_H_
#define _HUB_POSE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2

#include <vector>
#include <cmath>	//sin, cos
#include <iostream>     // cout

#include <opencv/cv.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <levmar/levmar.h>

#include "lib/hubmath.h"

namespace hub {
namespace slam {

template<typename Precision = TooN::DefaultPrecision>
struct pinhole_model_data_t {
	pinhole_model_data_t(std::vector<Precision> &object_points,
		std::vector< cv::Point_<Precision> > &image_points,
		const cv::Mat &camera_matrix) :
			object_points(object_points),
			image_points(image_points),
			camera_matrix(camera_matrix) {};

	std::vector<Precision> &object_points;
	std::vector< cv::Point_<Precision> > &image_points;
	const cv::Mat &camera_matrix;

// 	pinhole_model_data_t(TooN::Matrix<3,TooN::Dynamic,Precision> &object_points, TooN::Matrix<3,3,Precision> &camera_matrix) :
// 		object_points(object_points), camera_matrix(camera_matrix) {};
// 	TooN::Matrix<3,TooN::Dynamic,Precision> &object_points;
// 	TooN::Matrix<3,3,Precision> &camera_matrix;
};

template<typename T>
void pinhole_model(T *p, T *hx, int m, int n, void *data) {
	using namespace TooN;

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points_vector = pinhole_model_data->object_points;
	std::vector< cv::Point_<T> > &image_points = pinhole_model_data->image_points;
	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;

	const int num_points = n/2;
	Matrix<3,Dynamic,T,Reference::RowMajor> object_points(&(object_points_vector[0]), 3, num_points);
// 	Matrix<2,Dynamic,T,Reference::RowMajor> image_points(&(image_points_vector[0]), 2, num_points); 

	Vector<3,T,Reference> rotation_vector(p);
	Vector<3,T,Reference> translation_vector(p+3);
	SE3<T> transformation_matrix(rotation_vector, translation_vector);
	Matrix<3,Dynamic>  rotated_points = transformation_matrix.get_rotation() * object_points;
// std::cout << "rotation: " << rotation_vector << std::endl;
// std::cout << "translation: " << translation_vector << std::endl;

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

// std::cout << "_Projected pinhole model_" << std::endl;
	// calculate residuals
	for(int i=0; i<num_points; i++) {
		T z = rotated_points[2][i] + translation_vector[2];
		if(z <= 0.01) z = 1.0;
		// hx = measured image point - projected(transformed(object_point))
		hx[i] = image_points[i].x;
		hx[i] -= cx + fx * (rotated_points[0][i] + translation_vector[0]) / z;
// 		hx[i] -= (rotated_points[0][i] + translation_vector[0]) / z;
		hx[num_points+i] = image_points[i].y;
		hx[num_points+i] -= cy + fy * (rotated_points[1][i] + translation_vector[1]) / z;
// 		hx[num_points+i] -= (rotated_points[1][i] + translation_vector[1]) / z;
// std::cout << "(" << cx + fx * (rotated_points[0][i] + translation_vector[0]) / z
// 	<< ", " << cy + fy * (rotated_points[1][i] + translation_vector[1]) / z << ")" << std::endl;
	}

// std::cout << "_Residuals_" << std::endl;
// for(int i=0; i<2*num_points; i++) {
// 	std::cout << hx[i] << std::endl; 
// }
	// apply M-estimator
	std::vector<T> residuals(hx, hx+n);
	const T sigma = cl_robust_sigma<T>(residuals);
	for(int i=0; i<n; i++) {
		hx[i] *= tukey_weight<T>(hx[i], sigma);
	}
}

template<typename T>
void jac_pinhole_model(T *p, T *jac, int m, int n, void *data) {
	using namespace TooN;
	assert(p);
	assert(jac);
	assert(m == 6);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points_vector = pinhole_model_data->object_points;
	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;

	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);
	const T phi = p[0];
	const T theta = p[1];
	const T psi = p[2];
	const T t1 = p[3];
	const T t2 = p[4];
	const T t3 = p[5];
// std::cout << "p: " << phi << ", " << theta << ", " << psi << ", " << t1 << ", " << t2 << ", " << t3 << std::endl;
	const T cphi = cos(phi);
	const T cpsi = cos(psi);
	const T ctheta = cos(theta);
	const T sphi = sin(phi);
	const T spsi = sin(psi);
	const T stheta = sin(theta);

	const T cphi_cpsi = cphi*cpsi;
	const T cphi_ctheta = cphi*ctheta;
	const T cphi_stheta = cphi*stheta;
	const T cphi_spsi = cphi*spsi;
	const T cpsi_ctheta = cpsi*ctheta;
	const T cpsi_sphi = cpsi*sphi;
	const T cpsi_stheta = cpsi*stheta;
	const T ctheta_sphi = ctheta*sphi;
	const T ctheta_spsi = ctheta*spsi;
	const T sphi_spsi = sphi*spsi;
	const T sphi_stheta = sphi*stheta;
	const T spsi_stheta = spsi*stheta;

	const T cphi_cpsi_ctheta = cphi_cpsi*ctheta;
	const T cphi_cpsi_stheta = cphi_cpsi*stheta;
	const T cphi_ctheta_spsi = cphi_ctheta*spsi;
	const T cphi_spsi_stheta = cphi_spsi*stheta;
	const T cpsi_ctheta_sphi = cpsi_ctheta*sphi;
	const T cpsi_sphi_stheta = cpsi_sphi*stheta;
	const T ctheta_sphi_spsi = ctheta_sphi*spsi;
	const T sphi_spsi_stheta = sphi_spsi*stheta;

	const int num_points = n/2;
	Matrix<3,Dynamic,T,Reference::RowMajor> object_points(&(object_points_vector[0]), 3, num_points);
// std::cout << "_Object points (jacobian)_" << std::endl;
// std::cout << object_points << std::endl;
	int j = 0;
	const unsigned int v_offset = m*num_points;
	for(int i=0; i<num_points; i++) {
		const T x1 = object_points[0][i];
		const T x2 = object_points[1][i];
		const T x3 = object_points[2][i];

		const T y1 = t1 - x2*(cphi_spsi - cpsi_sphi_stheta) + x3*(sphi_spsi + cphi_cpsi_stheta) + x1*cpsi_ctheta;
		const T y2 = t2 + x2*(cphi_cpsi + sphi_spsi_stheta) - x3*(cpsi_sphi - cphi_spsi_stheta) + x1*ctheta_spsi;
		const T y3 = t3 - x1*stheta + x3*cphi_ctheta + x2*ctheta_sphi;
		const T y3_square = y3*y3;
		// partial u / partial phi
		jac[j] = -(fx*(x2*(sphi_spsi + cphi_cpsi_stheta) + x3*(cphi_spsi - cpsi_sphi_stheta)))/y3 + (fx*(x2*cphi_ctheta - x3*ctheta_sphi)*y1)/y3_square;
		// partial v / partial phi
		jac[v_offset+j++] = (fy*(x2*(cpsi_sphi - cphi_spsi_stheta) + x3*(cphi_cpsi + sphi_spsi_stheta)))/y3 + (fy*(x2*cphi_ctheta - x3*ctheta_sphi)*y2)/y3_square;
		// partial u / partial theta
		jac[j] = -(fx*(x3*cphi_cpsi_ctheta - x1*cpsi_stheta + x2*cpsi_ctheta_sphi))/y3 - (fx*(x1*ctheta + x3*cphi_stheta + x2*sphi_stheta)*y1)/y3_square;
		// partial v / partial theta
		jac[v_offset+j++] = -(fy*(x3*cphi_ctheta_spsi - x1*spsi_stheta + x2*ctheta_sphi_spsi))/y3 - (fy*(x1*ctheta + x3*cphi_stheta + x2*sphi_stheta)*y2)/y3_square;
		// partial u / partial psi
		jac[j] = (fx*(x2*(cphi_cpsi + sphi_spsi_stheta) - x3*(cpsi_sphi - cphi_spsi_stheta) + x1*ctheta_spsi))/y3;
		// partial v / partial psi
		jac[v_offset+j++] = -(fy*(x3*(sphi_spsi + cphi_cpsi_stheta) - x2*(cphi_spsi - cpsi_sphi_stheta) + x1*cpsi_ctheta))/y3;
		// partial u / partial x
		jac[j] = -fx/y3;
		// partial v / partial x
		jac[v_offset+j++] = 0;
		// partial u / partial y
		jac[j] = 0;
		// partial v / partial y
		jac[v_offset+j++] = -fy/y3;
		// partial u / partial z
		jac[j] = (fx*y1)/y3_square;
		// partial v / partial z
		jac[v_offset+j++] = (fy*y2)/y3_square;
	}
// std::cout << "_Jacobians_" << std::endl;
// for(int i=0; i<n; i++) {
// 	int index = i*m;
// 	std::cout << "[" << jac[index]
// 		<< ", " << jac[index+1]
// 		<< ", " << jac[index+2]
// 		<< ", " << jac[index+3]
// 		<< ", " << jac[index+4]
// 		<< ", " << jac[index+5]
// 		<< "]" << std::endl;
// }
}

template<typename T>
void simple_jac_pinhole_model(T *p, T *jac, int m, int n, void *data) {
	using namespace TooN;
	assert(m >= 6);
	const int num_points = n/2;

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points_vector = pinhole_model_data->object_points;
	Matrix<3,Dynamic,T,Reference::RowMajor> object_points(&(object_points_vector[0]), 3, num_points);
	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;

	Vector<3,T,Reference> rotation_vector(p);
	Vector<3,T,Reference> translation_vector(p+3);
	SE3<> transformation_matrix(rotation_vector, translation_vector);
	Matrix<3,Dynamic> rotated_points = transformation_matrix.get_rotation() * object_points;

	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const unsigned int v_offset = m*num_points;
	int j = 0;
	for(int i=0; i<num_points; i++) {
		const T y1 = rotated_points[0][i] + translation_vector[0];
		const T y2 = rotated_points[1][i] + translation_vector[1];
		const T y3 = rotated_points[2][i] + translation_vector[2];
		const T y3_square = y3*y3;

		jac[j] = (fx*y1*y2)/y3_square;
		jac[v_offset+j++] = fy*(1+(y2*y2)/y3_square);
		jac[j] = -fx*(1+(y1*y1)/y3_square);
		jac[v_offset+j++] = -(fy*y1*y2)/y3_square;
		jac[j] = (fx*y2)/y3;
		jac[v_offset+j++] = -(fy*y1)/y3;
		jac[j] = -fx/y3;
		jac[v_offset+j++] = 0;
		jac[j] = 0;
		jac[v_offset+j++] = -fy/y3;
		jac[j] = (fx*y1)/y3_square;
		jac[v_offset+j++] = (fy*y2)/y3_square;
	}
}


// int estimate_pose(const std::vector<cv::Point3f> &object_points,
// 	const std::vector<cv::KeyPoint>& dst_keypoints,
// 	const std::vector<cv::DMatch>& matches,
// 	const cv::Mat &camera_matrix,
// 	const cv::Mat &distortion_coefficients,
// 	const std::vector<char> &matches_mask,
// 	std::vector<float> &parameter_vector);

template<typename T>
inline int wrap_levmar_der(
	void (*func)(T *p, T *hx, int m, int n, void *adata),
	void (*jacf)(T *p, T *j, int m, int n, void *adata),
	T *p,
	T *x,
	int m,
	int n,
	int itmax,
	T opts[4],
	T info[LM_INFO_SZ],
	T *work,
	T *covar,
	void *adata);

template<>
inline int wrap_levmar_der<float>(
	void (*func)(float *p, float *hx, int m, int n, void *adata),
	void (*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p,
	float *x,
	int m,
	int n,
	int itmax,
	float opts[4],
	float info[LM_INFO_SZ],
	float *work,
	float *covar,
	void *adata) {

	return slevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata);
}

template<>
inline int wrap_levmar_der<double>(
	void (*func)(double *p, double *hx, int m, int n, void *adata),
	void (*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p,
	double *x,
	int m,
	int n,
	int itmax,
	double opts[4],
	double info[LM_INFO_SZ],
	double *work,
	double *covar,
	void *adata) {

	return dlevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata);
}

template<typename T>
int estimate_pose(const std::vector< cv::Point3_<T> > &object_points,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	const std::vector<char> &matches_mask,
	std::vector<T> &parameter_vector) {

	if(parameter_vector.size() != 6)
		return -1;

	unsigned int num_matches; //number of valid matches
	if(matches_mask.empty()) { //empty mask => all matches are valid
		num_matches = matches.size();
	} else if(matches_mask.size() != matches.size()) { //mask doesn't fit the matches
		return -2;
	} else { //count number of valid matches
		num_matches = 0;
		for(size_t i = 0; i < matches.size(); i++) {
			if( matches_mask[i] == 0) continue;
			num_matches++;
		}
	}

	std::vector<T> matched_object_points(3*num_matches); // allocate matrix 3 x num_matches
// 	vector<float> matched_image_points(2*num_matches); // allocate matrix 2 x num_matches
	std::vector< cv::Point_<T> > matched_image_points(num_matches);
	
	unsigned int next_point_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if( !matches_mask.empty() && matches_mask[i] == 0) continue;

		const int src_index = matches[i].queryIdx;
		matched_object_points[next_point_index] = object_points[src_index].x;
		matched_object_points[next_point_index+num_matches] = object_points[src_index].y;
		matched_object_points[next_point_index+2*num_matches] = object_points[src_index].z;

		const int dst_index = matches[i].trainIdx;
		const cv::KeyPoint& dst_keypoint = dst_keypoints[dst_index];
// 		matched_image_points[next_point_index] = dst_keypoint.pt.x;
// 		matched_image_points[next_point_index+num_matches] = dst_keypoint.pt.y;
		matched_image_points[i].x = dst_keypoint.pt.x;
		matched_image_points[i].y = dst_keypoint.pt.y;

		next_point_index++;
	}

	std::vector< cv::Point_<T> > undistorted_points(num_matches);
	//undistortPoints produces ideal point coordinates
	undistortPoints(matched_image_points, undistorted_points, camera_matrix, distortion_coefficients);
// std::cout << "_Undistorted points_ (ideal)" << std::endl;
// for(typename std::vector< cv::Point_<T> >::iterator iter = undistorted_points.begin(); iter != undistorted_points.end(); ++iter) {
// 	std::cout << "(" << iter->x << ", " << iter->y << ")" << std::endl;
// }

const double cx = camera_matrix.at<double>(0, 2);
const double cy = camera_matrix.at<double>(1, 2);
const double fx = camera_matrix.at<double>(0, 0);
const double fy = camera_matrix.at<double>(1, 1);
for(typename std::vector< cv::Point_<T> >::iterator iter = undistorted_points.begin(); iter != undistorted_points.end(); ++iter) {
	iter->x = fx*iter->x + cx;
	iter->y = fy*iter->y + cy;
}
// std::cout << "_Undistorted points_ (unideal)" << std::endl;
// for(typename std::vector< cv::Point_<T> >::iterator iter = undistorted_points.begin(); iter != undistorted_points.end(); ++iter) {
// 	std::cout << "(" << iter->x << ", " << iter->y << ")" << std::endl;
// }
	pinhole_model_data_t<T> pinhole_model_data(matched_object_points,
// 		matched_image_points,
		undistorted_points,
		camera_matrix);

// 	T info[LM_INFO_SZ];

	int rc = wrap_levmar_der<T>(
		pinhole_model<T>,
		jac_pinhole_model<T>,
		&(parameter_vector[0]),	//parameter vector
		NULL,	//measurement vector
		6,	//parameter vector dimension
		2*num_matches,	//measurement vector dimension
		100,	//max. number of iterations
		NULL,	//opts
		NULL,	//info
		NULL,	//work
		NULL,	//covar
		(void*)&pinhole_model_data);	//data

	return rc;
}

} // namespace slam
} // namespace hub

#endif // HAVE_OPENCV2
#endif // _HUB_POSE_H_
