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
//FIXME: remove TooN dependency
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <levmar/levmar.h>

#include "lib/hub/math.h"
#include "lib/slam/features.h"

namespace hub {
namespace slam {

/**
 * \brief Struct containing the 3D object, their corresponding 2D image points and the intrinsic parameters.
 */
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
};

/**
 * Approximate 3D camera translation from matched object- and keypoints.
 */ 
template<typename T>
cv::Point3_<T> camera_translation(const std::vector< cv::Point3_<T> >& objectpoints,
	const std::vector<cv::KeyPoint>& keypoints,
	const T distance,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients,
	const std::vector<char>& mask);

/**
 * \brief Pinhole model function to calculate the residuals.
 * 
 * This function calculates the residuals between the projected object points
 * and the measured image points. The residuals than get weighted by the Tukey
 * M-estimator to be more robust against outliers.
 * For the meaning of the arguments have a look at the levmar documentation.
 * 
 * \sa void ideal_pinhole_model(T *p, T *hx, int m, int n, void *data)
 */
template<typename T>
void pinhole_model(T *p, T *hx, int m, int n, void *data);

/**
 * \brief Pinhole model function to calculate the residuals using ideal point coordinates.
 * \sa void pinhole_model(T *p, T *hx, int m, int n, void *data)
 */
template<typename T>
void ideal_pinhole_model(T *p, T *hx, int m, int n, void *data);

/**
 * \brief Calculates the Jacobian matrix of the pinhole camera model.
 * 
 * This function calculates the Jacobians which are needed for the analytical 
 * Levenberg-Marquardt implementations.
 * 
 * \param p Parameters p=(phi,theta,psi,t1,t2,t3) to minimize, with (phi,theta,psi) euclidean rotation angles and t=(t1,t2,t3) 
 *          translation. 
 * \param jac Jacobian Matrix 
 * \param m Dimension of parameter vector \arg p, i.e. m = 6
 * \param n Dimension of measurement vector, i.e. twice the number of object points.
 * \param data Pointer to pinhole_model_data_t struct.
 * 
 * \sa struct pinhole_model_data_t
 * \sa void ideal_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);
 * \sa void approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);
 */
template<typename T>
void jac_pinhole_model(T *p, T *jac, int m, int n, void *data);

/**
 * \brief Calculates the Jacobian matrix of the pinhole camera model using ideal point coordinates.
 * \sa void jac_pinhole_model(T *p, T *jac, int m, int n, void *data)
 */
template<typename T>
void ideal_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);

/**
 * \brief Calculates an approximation of the Jacobian matrix of the pinhole camera model.
 * 
 * Determines Jacobian by pose parameters as described by Ethan Eade in
 * Appendix A of his PhD thesis "Monocular Simultaneous Localisation and
 * Mapping" which can be found at: http://mi.eng.cam.ac.uk/~ee231/thesis_revised.pdf
 * 
 * \param p Parameters p=(phi,theta,psi,t1,t2,t3) to minimize, with (phi,theta,psi) euclidean rotation angles and t=(t1,t2,t3) 
 *          translation. 
 * \param jac Jacobian Matrix 
 * \param m Dimension of parameter vector \arg p, i.e. m = 6
 * \param n Dimension of measurement vector, i.e. twice the number of object points.
 * \param data Pointer to pinhole_model_data_t struct.
 * 
 * \sa struct pinhole_model_data_t
 * \sa void jac_pinhole_model(T *p, T *jac, int m, int n, void *data)
 * \sa void approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);
 */
template<typename T>
void approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);

/**
 * \brief Calculates an approximation of the Jacobian matrix of the pinhole camera model using ideal point coordinates.
 * \sa void approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);
 */
template<typename T>
void ideal_approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);

/**
 * \brief Template function to wrap levmar with analytic Jacobian.
 * 
 * For the meaning of the arguments have a look at the levmar documentation.
 */
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

/**
 * \brief Estimate camera pose from 3D object points and their projected image points.
 * \param[in] object_points Vector of 3D object points.
 * \param[in] dst_keypoints Vector of keypoints containing the image coordinates.
 * \param[in] matches Vector containing the matches between the object and the image points.
 * \param[in] camera_matrix Matrix of intrinsic camera parameters.
 * \param[in] distortion_coefficients Vector containing the distortion coefficients.
 * \param[in] matches_mask Filter mask. Only matches with a non zero entry are considered.
 *                         An empty mask means all matches are considered
 * \param[in,out] parameter_vector The parameter vector containing the 3 euler angles (rad) and
 *                                 the 3 translations. The values given are used as a starting
 *                                 point for the iteration process.
 * \param[in] max_iterations Maximum number of iterations.
 * \return Number of iterations. A negative value means an error occured.
 */
template<typename T>
int estimate_pose(const std::vector< cv::Point3_<T> > &object_points,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	std::vector<T> &parameter_vector,
	const std::vector<char> &matches_mask = std::vector<char>(),
	const unsigned int max_iterations = 100);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------

template<typename T>
cv::Point3_<T> camera_translation(const std::vector< cv::Point3_<T> >& objectpoints,
	const std::vector<cv::KeyPoint>& keypoints,
	const T distance,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat& camera_matrix,
	const cv::Mat& distortion_coefficients,
	const std::vector<char>& mask) {

	if(matches.empty()) return cv::Point3_<T>(0, 0, 0);
	assert(matches.size() == mask.size());

	std::vector<T> x_differences,
		y_differences,
		z_differences;
	x_differences.reserve( matches.size() );
	y_differences.reserve( matches.size() );
	z_differences.reserve( matches.size() );

	for(size_t i = 0; i < matches.size(); i++) {
		if(mask[i] == 0) continue;
		
		const unsigned int fi = matches[i].queryIdx;
		const unsigned int si = matches[i].trainIdx;

		cv::Point2f undist_point = undistort_n2i(keypoints[si].pt, camera_matrix, distortion_coefficients);
		x_differences.push_back( objectpoints[fi].x - (undist_point.x*distance) );
		y_differences.push_back( objectpoints[fi].y - (undist_point.y*distance) );
		z_differences.push_back(objectpoints[fi].z - distance);
	}

	cv::Point3_<T> translation;
	translation.x = hub::_median(x_differences);
	translation.y = hub::_median(y_differences);
	translation.z = hub::_median(z_differences);
	
	return translation;
}

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

	Vector<3,T,Reference> rotation_vector(p);
	Vector<3,T,Reference> translation_vector(p+3);
	SE3<T> transformation_matrix(rotation_vector, translation_vector);
	Matrix<3,Dynamic>  rotated_points = transformation_matrix.get_rotation() * object_points;

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

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
	const T sigma = _robust_sigma<T>(residuals);
	for(int i=0; i<n; i++) {
		hx[i] *= tukey_weight<T>(hx[i], sigma);
	}
}

template<typename T>
void ideal_pinhole_model(T *p, T *hx, int m, int n, void *data) {
	using namespace TooN;

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points_vector = pinhole_model_data->object_points;
	std::vector< cv::Point_<T> > &image_points = pinhole_model_data->image_points;

	const int num_points = n/2;
	Matrix<3,Dynamic,T,Reference::RowMajor> object_points(&(object_points_vector[0]), 3, num_points);

	Vector<3,T,Reference> rotation_vector(p);
	Vector<3,T,Reference> translation_vector(p+3);
	SE3<T> transformation_matrix(rotation_vector, translation_vector);
	Matrix<3,Dynamic>  rotated_points = transformation_matrix.get_rotation() * object_points;

	// calculate residuals
	for(int i=0; i<num_points; i++) {
		T z = rotated_points[2][i] + translation_vector[2];
		if(z <= 0.01) z = 1.0;
		// hx = measured image point - projected(transformed(object_point))
		hx[i] = image_points[i].x;
		hx[i] -= (rotated_points[0][i] + translation_vector[0]) / z;
		hx[num_points+i] = image_points[i].y;
		hx[num_points+i] -= (rotated_points[1][i] + translation_vector[1]) / z;
	}

	// apply M-estimator
	std::vector<T> residuals(hx, hx+n);
	const T sigma = _robust_sigma<T>(residuals);
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
void ideal_jac_pinhole_model(T *p, T *jac, int m, int n, void *data) {
	using namespace TooN;
	assert(p);
	assert(jac);
	assert(m == 6);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points_vector = pinhole_model_data->object_points;

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
		jac[j] = -((x2*(sphi_spsi + cphi_cpsi_stheta) + x3*(cphi_spsi - cpsi_sphi_stheta)))/y3 + ((x2*cphi_ctheta - x3*ctheta_sphi)*y1)/y3_square;
		// partial v / partial phi
		jac[v_offset+j++] = ((x2*(cpsi_sphi - cphi_spsi_stheta) + x3*(cphi_cpsi + sphi_spsi_stheta)))/y3 + ((x2*cphi_ctheta - x3*ctheta_sphi)*y2)/y3_square;
		// partial u / partial theta
		jac[j] = -((x3*cphi_cpsi_ctheta - x1*cpsi_stheta + x2*cpsi_ctheta_sphi))/y3 - ((x1*ctheta + x3*cphi_stheta + x2*sphi_stheta)*y1)/y3_square;
		// partial v / partial theta
		jac[v_offset+j++] = -((x3*cphi_ctheta_spsi - x1*spsi_stheta + x2*ctheta_sphi_spsi))/y3 - ((x1*ctheta + x3*cphi_stheta + x2*sphi_stheta)*y2)/y3_square;
		// partial u / partial psi
		jac[j] = ((x2*(cphi_cpsi + sphi_spsi_stheta) - x3*(cpsi_sphi - cphi_spsi_stheta) + x1*ctheta_spsi))/y3;
		// partial v / partial psi
		jac[v_offset+j++] = -((x3*(sphi_spsi + cphi_cpsi_stheta) - x2*(cphi_spsi - cpsi_sphi_stheta) + x1*cpsi_ctheta))/y3;
		// partial u / partial x
		jac[j] = -1.0/y3;
		// partial v / partial x
		jac[v_offset+j++] = 0;
		// partial u / partial y
		jac[j] = 0;
		// partial v / partial y
		jac[v_offset+j++] = -1.0/y3;
		// partial u / partial z
		jac[j] = y1/y3_square;
		// partial v / partial z
		jac[v_offset+j++] = y2/y3_square;
	}
}

template<typename T>
void approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data) {
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

template<typename T>
void ideal_approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data) {
	using namespace TooN;
	assert(m >= 6);
	const int num_points = n/2;

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points_vector = pinhole_model_data->object_points;
	Matrix<3,Dynamic,T,Reference::RowMajor> object_points(&(object_points_vector[0]), 3, num_points);

	Vector<3,T,Reference> rotation_vector(p);
	Vector<3,T,Reference> translation_vector(p+3);
	SE3<> transformation_matrix(rotation_vector, translation_vector);
	Matrix<3,Dynamic> rotated_points = transformation_matrix.get_rotation() * object_points;

	const unsigned int v_offset = m*num_points;
	int j = 0;
	for(int i=0; i<num_points; i++) {
		const T y1 = rotated_points[0][i] + translation_vector[0];
		const T y2 = rotated_points[1][i] + translation_vector[1];
		const T y3 = rotated_points[2][i] + translation_vector[2];
		const T y3_square = y3*y3;

		jac[j] = (y1*y2)/y3_square;
		jac[v_offset+j++] = 1+(y2*y2)/y3_square;
		jac[j] = -(1+(y1*y1)/y3_square);
		jac[v_offset+j++] = -(y1*y2)/y3_square;
		jac[j] = y2/y3;
		jac[v_offset+j++] = -y1/y3;
		jac[j] = -1/y3;
		jac[v_offset+j++] = 0;
		jac[j] = 0;
		jac[v_offset+j++] = -1/y3;
		jac[j] = y1/y3_square;
		jac[v_offset+j++] = y2/y3_square;
	}
}

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
	std::vector<T> &parameter_vector,
	const std::vector<char> &matches_mask,
	const unsigned int max_iterations) {

	assert(parameter_vector.size() == 6);
	if(matches.empty()) return 0;
	if(object_points.empty()) return -1;
	if(dst_keypoints.empty()) return -1;

	unsigned int num_matches; //number of valid matches
	if(matches_mask.empty()) { //empty mask => all matches are valid
		num_matches = matches.size();
	} else if(matches_mask.size() != matches.size()) { //mask doesn't fit the matches
		return -2;
	} else { //size of matches and mask are equal => count number of valid matches
		num_matches = 0;
		for(size_t i = 0; i < matches.size(); i++) {
			if( matches_mask[i] == 0) continue;
			num_matches++;
		}
	}
	if(num_matches == 0) //no matches found
		return 0;
		
	std::vector<T> matched_object_points(3*num_matches); // allocate matrix 3 x num_matches
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
		matched_image_points[next_point_index].x = dst_keypoint.pt.x;
		matched_image_points[next_point_index].y = dst_keypoint.pt.y;

		next_point_index++;
	}

	std::vector< cv::Point_<T> > undistorted_points(num_matches);
	//undistortPoints produces ideal point coordinates
	undistortPoints(matched_image_points, undistorted_points, camera_matrix, distortion_coefficients);

	pinhole_model_data_t<T> pinhole_model_data(matched_object_points,
		undistorted_points,
		camera_matrix);

// 	T info[LM_INFO_SZ];

	int rc = wrap_levmar_der<T>(
		ideal_pinhole_model<T>,
// 		ideal_jac_pinhole_model<T>,
		ideal_approx_jac_pinhole_model<T>,
		&(parameter_vector[0]),	//parameter vector
		NULL,	//measurement vector (is part of pinhole_model_data)
		6,	//parameter vector dimension
		2*num_matches,	//measurement vector dimension
		max_iterations,	//max. number of iterations
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
