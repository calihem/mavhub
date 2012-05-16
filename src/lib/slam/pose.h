#ifndef _HUB_POSE_H_
#define _HUB_POSE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2

#include <vector>
#include <opencv/cv.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>

#include "lib/hubmath.h"

namespace hub {
namespace slam {

template<typename Precision = TooN::DefaultPrecision>
struct pinhole_model_data_t {
	pinhole_model_data_t(std::vector<Precision> &object_points,
		std::vector<cv::Point2f> &image_points,
		const cv::Mat &camera_matrix) :
			object_points(object_points),
			image_points(image_points),
			camera_matrix(camera_matrix) {};

	std::vector<Precision> &object_points;
	std::vector<cv::Point2f> &image_points;
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
	std::vector<cv::Point2f> &image_points = pinhole_model_data->image_points;
	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;

	const int num_points = n/2;
	Matrix<3,Dynamic,T,Reference::RowMajor> object_points(&(object_points_vector[0]), 3, num_points);
// 	Matrix<2,Dynamic,T,Reference::RowMajor> image_points(&(image_points_vector[0]), 2, num_points); 

	Vector<3,T,Reference> rotation_vector(p);
	Vector<3,T,Reference> translation_vector(p+3);
	SE3<T> transformation_matrix(rotation_vector, translation_vector);
	Matrix<3,Dynamic>  rotated_points = transformation_matrix.get_rotation() * object_points;

	const double cx = camera_matrix.at<double>(0, 2);
	const double cy = camera_matrix.at<double>(1, 2);
	const double fx = camera_matrix.at<double>(0, 0);
	const double fy = camera_matrix.at<double>(1, 1);

	// calculate residuals
	for(int i=0; i<num_points; i++) {
		T z = rotated_points[2][i] + translation_vector[2];
		if(z <= 0.01) z = 1.0;
		// hx = measured image point - projected(transformed(object_point))
		hx[i] = image_points[i].x;
		hx[i] -= cx + fx * (rotated_points[0][i] + translation_vector[0]) / z;
		hx[num_points+i] = image_points[i].y;
		hx[num_points+i] -= cy + fy * (rotated_points[1][i] + translation_vector[1]) / z;
	}

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
	assert(m >= 6);

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

int estimate_pose(const std::vector<cv::Point3f> &object_points,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	const std::vector<char> &matches_mask,
	std::vector<float> &parameter_vector);

} // namespace slam
} // namespace hub

#endif // HAVE_OPENCV2
#endif // _HUB_POSE_H_
