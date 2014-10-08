#ifndef _HUB_POSE_H_
#define _HUB_POSE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#if CV_MINOR_VERSION >= 2

#include <vector>
#include <cmath>	//sin, cos
#include <iostream>     // cout

#include <levmar/levmar.h>

#include <lib/hub/condition.h>
#include "lib/hub/math.h"
#include "lib/hub/utility.h"
#include "lib/slam/features.h"
#include "lib/slam/camera.h"

namespace hub {
namespace slam {

/**
 * \brief Struct containing the 3D object, their corresponding 2D image points and the intrinsic parameters.
 */
template<typename Precision = float>
struct pinhole_model_data_t {
	pinhole_model_data_t(std::vector<Precision> &object_points,
		std::vector< cv::Point_<Precision> > &image_points,
		const cv::Mat &camera_matrix = cv::Mat()) :
			object_points(object_points),
			image_points(image_points),
			camera_matrix(camera_matrix) {};

	std::vector<Precision> &object_points;
	std::vector< cv::Point_<Precision> > &image_points;
	const cv::Mat &camera_matrix;
};

/**
 * \brief Approximate 3D camera translation from matched feature.
 */
template<typename T>
void estimate_translation_by_features(const std::vector<cv::Point2f>& src_features,
	const std::vector<cv::Point2f>& dst_features,
	const T src_distance,
	const T dst_distance,
	const std::vector<cv::DMatch>& matches,
	T translation[3],
	const std::vector<char>& mask = std::vector<char>());

/**
 * \brief Approximate 3D camera translation from matched object- and idealpoints.
 */ 
template<typename T>
void estimate_translation_by_objects(const std::vector< cv::Point3_<T> >& objectpoints,
	const std::vector<cv::Point2f>& idealpoints,
	const T avg_distance,
	const std::vector<cv::DMatch>& matches,
	T translation[3],
	const std::vector<char>& mask = std::vector<char>());

// TODO implement efficient second order minimization
// template<typename T>
// int esm_pose_optimization();

/**
 * \brief Estimate camera pose from 3D object points and their projected ideal image points.
 * 
 * \tparam T Base type used for points.
 * \tparam F Camera model function to project object points to image plane.
 * \tparam JAC_F Jacobian of \arg F.
 * \param[in] object_points Vector of 3D object points.
 * \param[in] image_points Vector of image coordinates.
 * \param[in] matches Vector containing the matches between the object and the image points.
 * \param[in,out] parameter_vector The parameter vector containing rotation and transaltion
 *                                 information. The values given are used as a starting
 *                                 point for the iteration process.
 * \param[in] matches_mask Filter mask. Only matches with a non zero entry are considered.
 *                         An empty mask means all matches are considered
 * \param[in] max_iterations Maximum number of iterations.
 * \param[out] info Information regarding the minimization
 * \return Number of iterations. A negative value means an error occured.
 */
template<typename T,  void (*F)(T *, T *, int, int, void *), void (*JAC_F)(T *, T *, int, int, void *)>
int lm_pose_optimization(const std::vector< cv::Point3_<T> > &object_points,
	const std::vector<cv::Point2f>& image_points,
	const std::vector<cv::DMatch>& matches,
	std::vector<T> &parameter_vector,
	const cv::Mat &camera_matrix = cv::Mat(),
	const std::vector<char> &matches_mask = std::vector<char>(),
	const unsigned int max_iterations = 100,
	T info[LM_INFO_SZ] = NULL);

template<typename T, void(*F)(T*, T*, T*, size_t)>
void levmar_ideal_pinhole_model(T *p, T *hx, int m, int n, void *data);

template<typename T, void(*F)(T*, T*, cv::Mat &, T*, size_t)>
void levmar_pinhole_model(T *p, T *hx, int m, int n, void *data);

/**
 * \brief Pinhole model function to calculate the residuals using ideal point coordinates.
 * \sa void pinhole_model(T *p, T *hx, int m, int n, void *data)
 */
template<typename T>
void levmar_ideal_pinhole_euler(T *p, T *hx, int m, int n, void *data);

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
void levmar_pinhole_euler(T *p, T *hx, int m, int n, void *data);

template<typename T>
void levmar_ideal_pinhole_quat(T *p, T *hx, int m, int n, void *data);

template<typename T>
inline void levmar_ideal_pinhole_quatvec(T *p, T *hx, int m, int n, void *data);

template<typename T>
void levmar_pinhole_euler(T *p, T *hx, int m, int n, void *data);


template<typename T, void(*JAC_F)(const T*, const T*, T*, T*)>
void levmar_ideal_pinhole_model_jac(T *p, T *jac, int m, int n, void *data);

/**
 * \brief Calculates an approximation of the Jacobian matrix of the pinhole camera model using ideal point coordinates.
 * \sa void approx_jac_pinhole_model(T *p, T *jac, int m, int n, void *data);
 */
template<typename T>
void levmar_approx_ideal_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data);

template<typename T>
void levmar_ideal_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data);

/**
 * \brief Calculates the Jacobian matrix of the pinhole camera model using ideal point coordinates.
 * \sa void jac_pinhole_model(T *p, T *jac, int m, int n, void *data)
 */
template<typename T>
void levmar_ideal_pinhole_quat(T *p, T *hx, int m, int n, void *data);

template<typename T>
void levmar_ideal_pinhole_quat_jac(T *p, T *hx, int m, int n, void *data);

/**
 * \brief
 * This is an optimized version of levmar_ideal_pinhole_model_jac< T, ideal_pinhole_model_quatvec_jac<T> >(p, jac, m, n, data);
 */
template<typename T>
void levmar_ideal_pinhole_quatvec_jac(T *p, T *hx, int m, int n, void *data);

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
void levmar_approx_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data);

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
void levmar_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data);

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
// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
template<typename T>
void estimate_translation_by_features(const std::vector<cv::Point2f>& src_features,
	const std::vector<cv::Point2f>& dst_features,
	const T src_distance,
	const T dst_distance,
	const std::vector<cv::DMatch>& matches,
	T translation[3],
	const std::vector<char>& mask) {

	if(matches.empty()) {
		translation[0] = 0;
		translation[1] = 0;
		translation[2] = 0;
		return;
	}
	assert( matches.size() == mask.size() || mask.empty() );
	assert(translation);

// 	std::vector< hub::indexed_item_t<T> > sq_differences;
// 	sq_differences.reserve( matches.size() );
	std::vector<T> u_differences, v_differences;
	u_differences.reserve( matches.size() );
	v_differences.reserve( matches.size() );

	for(size_t i = 0; i < matches.size(); i++) {
		if(!mask.empty() && mask[i] == 0) continue;

		const unsigned int src_index = matches[i].queryIdx;
		const unsigned int dst_index = matches[i].trainIdx;

// 		const T delta_x = dst_features[si].x - src_features[fi].x;
// 		const T delta_y = dst_features[si].y - src_features[fi].y;
// 		const T sq_distance = delta_x*delta_x + delta_y*delta_y;
// 		sq_differences.push_back( hub::indexed_item_t<T>(sq_distance, i) );

		u_differences.push_back(src_features[src_index].x - dst_features[dst_index].x);
		v_differences.push_back(src_features[src_index].y - dst_features[dst_index].y);
	}
// 	hub::indexed_item_t<T> sq_distance_index_median = hub::_median(sq_differences);
// 	const unsigned int fi = matches[sq_distance_index_median.index].queryIdx;
// 	const unsigned int si = matches[sq_distance_index_median.index].trainIdx;

	const T avg_distance = (dst_distance + src_distance) / 2;
	// an element wise median is more robust than median of sq_differences
	translation[0] = hub::_median(u_differences) * avg_distance;
	translation[1] = hub::_median(v_differences) * avg_distance;
// 	translation[0] = (dst_features[si].x - src_features[fi].x) * avg_distance;
// 	translation[1] = (dst_features[si].y - src_features[fi].y) * avg_distance;
	translation[2] = src_distance - dst_distance;
}

template<typename T>
void estimate_translation_by_objects(const std::vector< cv::Point3_<T> >& objectpoints,
	const std::vector<cv::Point2f>& idealpoints,
	const T avg_distance,
	const std::vector<cv::DMatch>& matches,
	T translation[3],
	const std::vector<char>& mask) {

	if(matches.empty()) {
		translation[0] = 0;
		translation[1] = 0;
		translation[2] = 0;
		return;
	}
	assert( matches.size() == mask.size() || mask.empty() );
	assert(translation);

	std::vector<T> x_differences,
		y_differences,
		z_differences;
	x_differences.reserve( matches.size() );
	y_differences.reserve( matches.size() );
	z_differences.reserve( matches.size() );

	for(size_t i = 0; i < matches.size(); i++) {
		if(!mask.empty() && mask[i] == 0) continue;

		const unsigned int fi = matches[i].queryIdx;
		const unsigned int si = matches[i].trainIdx;

		x_differences.push_back( objectpoints[fi].x - (idealpoints[si].x*avg_distance) );
		y_differences.push_back( objectpoints[fi].y - (idealpoints[si].y*avg_distance) );
		z_differences.push_back( avg_distance - objectpoints[fi].z );
	}

	translation[0] = hub::_median(x_differences);
	translation[1] = hub::_median(y_differences);
	translation[2] = hub::_median(z_differences);
}

template<typename T,  void (*F)(T *, T *, int, int, void *), void (*JAC_F)(T *, T *, int, int, void *)>
int lm_pose_optimization(const std::vector< cv::Point3_<T> > &object_points,
		const std::vector<cv::Point2f>& image_points,
		const std::vector<cv::DMatch>& matches,
		std::vector<T> &parameter_vector,
		const cv::Mat &camera_matrix,
		const std::vector<char> &matches_mask,
		const unsigned int max_iterations,
		T info[LM_INFO_SZ]) {

	assert(parameter_vector.size() >= 6);
	if(matches.empty()) return 0;
	if(object_points.empty()) return -1;
	if(image_points.empty()) return -1;

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
	if(num_matches <= 4) //not enough matches found
		return 0;

	// fill vectors containing only the matched features
	std::vector<T> matched_objectpoints(3*num_matches); // allocate matrix 3 x num_matches
	std::vector< cv::Point_<T> > matched_projections(num_matches);
	unsigned int next_point_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if( !matches_mask.empty() && matches_mask[i] == 0) continue;

		const int src_index = matches[i].queryIdx;
		matched_objectpoints[next_point_index*3] = object_points[src_index].x;
		matched_objectpoints[next_point_index*3 + 1] = object_points[src_index].y;
		matched_objectpoints[next_point_index*3 + 2] = object_points[src_index].z;

		const int dst_index = matches[i].trainIdx;
		matched_projections[next_point_index] = image_points[dst_index];

		next_point_index++;
	}

	// fill structure for levmar algo
	pinhole_model_data_t<T> pinhole_model_data(matched_objectpoints,
		matched_projections,
		camera_matrix);

	// run Levenberg-Marquardt
	return wrap_levmar_der<T>(
		F,
		JAC_F,
		&(parameter_vector[0]),	//parameter vector
		NULL,	//measurement vector (is part of pinhole_model_data)
		6,	//parameter vector dimension
		2*num_matches,	//measurement vector dimension
		max_iterations,	//max. number of iterations
		NULL,	//opts
		info,	//info
		NULL,	//work
		NULL,	//covar
		(void*)&pinhole_model_data);	//data
}

template<typename T, void(*F)(const T*, const T*, T*, const size_t)>
void levmar_ideal_pinhole_model(T *p, T *hx, int /*m*/, int n, void *data) {

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points = pinhole_model_data->object_points;
	std::vector< cv::Point_<T> > &image_points = pinhole_model_data->image_points;

	const int num_points = n/2;
	std::vector<T> model_points(num_points*2);
	F(&object_points[0],
		p,
		&model_points[0],
		num_points);

	// calculate residuals
	for(int i=0; i<num_points; i++) {
		// hx = measured image point - projected(transformed(object_point))
		hx[i] = image_points[i].x - model_points[i*2];
		hx[num_points+i] = image_points[i].y - model_points[i*2+1];
	}

	// apply M-estimator
	std::vector<T> u_residuals(hx, hx+num_points);
	const T u_sigma = _robust_sigma<T>(u_residuals);
	for(int i=0; i<num_points; i++) {
		hx[i] *= tukey_weight<T>(hx[i], u_sigma);
	}
	std::vector<T> v_residuals(hx+num_points, hx+n);
	const T v_sigma = _robust_sigma<T>(v_residuals);
	for(int i=num_points; i<n; i++) {
		hx[i] *= tukey_weight<T>(hx[i], v_sigma);
	}
}

template<typename T, void(*F)(const T*, const T*, const cv::Mat &, T*, const size_t)>
void levmar_pinhole_model(T *p, T *hx, int /*m*/, int n, void *data) {

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points = pinhole_model_data->object_points;
	std::vector< cv::Point_<T> > &image_points = pinhole_model_data->image_points;
	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;
	if( camera_matrix.empty() ) return;

	const int num_points = n/2;
	std::vector<T> model_points(num_points*2);
	F(&object_points[0],
		p,
		camera_matrix,
		&model_points[0],
		num_points);

	// calculate residuals
	for(int i=0; i<num_points; i++) {
		// hx = measured image point - projected(transformed(object_point))
		hx[i] = image_points[i].x - model_points[i*2];
		hx[num_points+i] = image_points[i].y - model_points[i*2+1];
	}

	// apply M-estimator
	std::vector<T> u_residuals(hx, hx+num_points);
	const T u_sigma = _robust_sigma<T>(u_residuals);
	for(int i=0; i<num_points; i++) {
		hx[i] *= tukey_weight<T>(hx[i], u_sigma);
	}
	std::vector<T> v_residuals(hx+num_points, hx+n);
	const T v_sigma = _robust_sigma<T>(v_residuals);
	for(int i=num_points; i<n; i++) {
		hx[i] *= tukey_weight<T>(hx[i], v_sigma);
	}
}

template<typename T>
inline void levmar_ideal_pinhole_euler(T *p, T *hx, int m, int n, void *data) {
	levmar_ideal_pinhole_model< T, ideal_pinhole_model<T, rotation_matrix_rad> >(p, hx, m, n, data);
}

template<typename T>
inline void levmar_ideal_pinhole_quat(T *p, T *hx, int m, int n, void *data) {
	levmar_ideal_pinhole_model< T, ideal_pinhole_model_quat<T> >(p, hx, m, n, data);
}

template<typename T>
inline void levmar_ideal_pinhole_quatvec(T *p, T *hx, int m, int n, void *data) {
	levmar_ideal_pinhole_model< T, ideal_pinhole_model<T, rotation_matrix_quatvec> >(p, hx, m, n, data);
}

template<typename T>
inline void levmar_pinhole_euler(T *p, T *hx, int m, int n, void *data) {
	levmar_pinhole_model< T, pinhole_model<T, rotation_matrix_rad> >(p, hx, m, n, data);
}

// ----------------------------------------------------------------------------
// Jacobians
// ----------------------------------------------------------------------------
/*FIXME signature of JAC_F has changed
template<typename T, void(*JAC_F)(const T*, const T*, T*, T*)>
void levmar_ideal_pinhole_model_jac(T *p, T *jac, int m, int n, void *data) {

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &objectpoints = pinhole_model_data->object_points;

	const int num_points = n/2;
	int j = 0;
	const unsigned int v_offset = m*num_points;
	for(int i=0; i<num_points; i++) {
		JAC_F(&objectpoints[i*3],
		p,
		&jac[j],
		&jac[v_offset+j]);
		j+=6;
	}
}*/

template<typename T>
void levmar_ideal_pinhole_quat_jac(T */*p*/, T */*jac*/, int /*m*/, int /*n*/, void */*data*/) {
	//TODO
	assert(0);
}

template<typename T>
void levmar_ideal_pinhole_quatvec_jac(T *p, T *jac, int m, int n, void *data) {
	assert(p);
	assert(jac);
	assert(m == 6);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points = pinhole_model_data->object_points;

	const T q1 = p[0];
	const T q2 = p[1];
	const T q3 = p[2];
	const T q1q1 = q1*q1;
	const T q2q2 = q2*q2;
	const T q3q3 = q3*q3;
	const T q0 = sqrt(1.0-q1q1-q2q2-q3q3);
	const T t56 = -q1/q0;
	const T t175 = -q2/q0;
	const T t240 = -q3/q0;

	const int num_points = n/2;
	int j = 0;
	const unsigned int v_offset = m*num_points;
	for(int i=0; i<num_points; i++) {
		const T p0 = object_points[i*3];
		const T p1 = object_points[i*3 + 1];
		const T p2 = object_points[i*3 + 2];

		const T t51 = -q1*p0-q2*p1-q3*p2;
		const T t65 = q0*p0+q2*p2-p1*q3;
		const T t70 = t56*p0;
		const T t75 = t56*p1-p2;
		const T t81 = q0*p1+q3*p0-q1*p2;
		const T t87 = t56*p2+p1;
		const T t93 = q0*p2+q1*p1-q2*p0;
		const T t107 = p0*q2+t56*t81+q0*t75-t87*q1-t93+t70*q3;
		const T t118 = p0*q3+t56*t93+q0*t87-t70*q2+q1*t75+t81;
		const T t126 = -q3*t51+q0*t93-t65*q2+t81*q1+p[5];
		const T t127 = 1/t126;
		const T t145 = t126*t126;
		const T t147 = (-q1*t51+q0*t65-t81*q3+t93*q2+p[3])/t145;
		// partial q1
		jac[j] = (p0*q1-t51+t56*t65+q0*t70-q3*t75+q2*t87)*t127-t118*t147;
		const T t159 = (-t51*q2+q0*t81-t93*q1+t65*q3+p[4])/t145;
		jac[v_offset+j++] = t107*t127-t159*t118;
		const T t180 = t175*p0+p2;
		const T t185 = t175*p1;
		const T t192 = t175*p2-p0;
		const T t206 = p1*q2-t51+t175*t81+q0*t185-t192*q1+q3*t180;
		const T t216 = p1*q3+t93*t175+q0*t192-t180*q2-t65+t185*q1;
		// partial q2
		jac[j] = (p1*q1+t175*t65+t180*q0-t185*q3+t192*q2+t93)*t127-t147*t216;
		jac[v_offset+j++] = t206*t127-t159*t216;
		const T t245 = t240*p0-p1;
		const T t250 = p1*t240+p0;
		const T t257 = t240*p2;
		const T t271 = p2*q2+t81*t240+q0*t250-t257*q1+t245*q3+t65;
		const T t281 = p2*q3-t51+t240*t93+q0*t257-t245*q2+t250*q1;
		// partial q3
		jac[j] = (p2*q1+t240*t65+q0*t245-t250*q3-t81+t257*q2)*t127-t147*t281;
		jac[v_offset+j++] = t271*t127-t159*t281;
		// partial t1
		jac[j] = t127;
		jac[v_offset+j++] = 0.0;
		// partial t2
		jac[j] = 0.0;
		jac[v_offset+j++] = t127;
		// partial t3
		jac[j] = -t147;
		jac[v_offset+j++] = -t159;

#if 0
		// test condition of jacobi matrix 
		using namespace TooN;
		Matrix<2, 6, T> matrix_jac;
		const unsigned int jj = i*6;
		for(unsigned int ii = 0; ii < 6; ii++) {
			matrix_jac(0, ii) = jac[jj + ii];
			matrix_jac(1, ii) = jac[v_offset + jj + ii];
		}
		const T c = hub::condition<2, 6, T>(matrix_jac);
		if(c > 20)
			std::cerr << "ERROR: Jacobian matrix " << matrix_jac << " has a bad condition " << c << std::endl;
#endif
	}
}

template<typename T>
void levmar_approx_ideal_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data) {
	assert(m >= 6);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points = pinhole_model_data->object_points;

	const int num_points = n/2;
	T rotation_matrix[9];
	rotation_matrix_rad(p, rotation_matrix);
	T transformed_point[3];

	const unsigned int v_offset = m*num_points;
	int j = 0;
	for(int i=0; i<num_points; i++) {
		multiply(rotation_matrix, &object_points[i*3], transformed_point);

		const T y1 = transformed_point[0] + p[3];
		const T y2 = transformed_point[1] + p[4];
		transformed_point[2] += p[5];
		if(transformed_point[2] <= 0.0001) transformed_point[2] = 1.0;
		const T inv_y3 = 1.0/transformed_point[2];
		const T inv_y3_square = inv_y3*inv_y3;

		jac[j] = -y1*y2*inv_y3_square;
		jac[v_offset+j++] = -(1+y2*y2*inv_y3_square);
		jac[j] = 1+y1*y1*inv_y3_square;
		jac[v_offset+j++] = y1*y2*inv_y3_square;
		jac[j] = -y2*inv_y3;
		jac[v_offset+j++] = y1*inv_y3;
		jac[j] = inv_y3;
		jac[v_offset+j++] = 0;
		jac[j] = 0;
		jac[v_offset+j++] = inv_y3;
		jac[j] = -y1*inv_y3_square;
		jac[v_offset+j++] = -y2*inv_y3_square;
	}
}

template<typename T>
void levmar_ideal_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data) {
	assert(p);
	assert(jac);
	assert(m == 6);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;
	std::vector<T> &object_points = pinhole_model_data->object_points;

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

	const T a1 = sphi_spsi + cphi_cpsi_stheta;
	const T a2 = cphi_spsi - cpsi_sphi_stheta;
	const T a3 = cphi_cpsi + sphi_spsi_stheta;
	const T a4 = cpsi_sphi - cphi_spsi_stheta;

	const int num_points = n/2;
	int j = 0;
	const unsigned int v_offset = m*num_points;
	for(int i=0; i<num_points; i++) {
		const T x1 = object_points[i*3];
		const T x2 = object_points[i*3 + 1];
		const T x3 = object_points[i*3 + 2];

		const T b1 = x2*a4 + x3*a3;
		const T b2 = x2*a1 + x3*a2;
		const T b3 = x2*cphi_ctheta - x3*ctheta_sphi;
		const T b4 = x3*cphi_cpsi_ctheta - x1*cpsi_stheta + x2*cpsi_ctheta_sphi;
		const T b5 = x1*ctheta + x3*ctheta_sphi + x2*sphi_stheta;
		const T b6 = x3*cphi_ctheta_spsi - x1*spsi_stheta + x2*ctheta_sphi_spsi;

		const T y1 = t1 - x2*(cphi_spsi - cpsi_sphi_stheta) + x3*(sphi_spsi + cphi_cpsi_stheta) + x1*cpsi_ctheta;
		const T y2 = t2 + x2*(cphi_cpsi + sphi_spsi_stheta) - x3*(cpsi_sphi - cphi_spsi_stheta) + x1*ctheta_spsi;
		const T inv_y3 = 1.0/(t3 - x1*stheta + x3*cphi_ctheta + x2*ctheta_sphi);
		const T inv_y3_square = inv_y3*inv_y3;

		// partial u / partial phi
		jac[j] = b2*inv_y3 - b3*y1*inv_y3_square;
		// partial v / partial phi
		jac[v_offset+j++] = -(b1*inv_y3 + b3*y2*inv_y3_square);
		// partial u / partial theta
		jac[j] = b4*inv_y3 + b5*y1*inv_y3_square;
		// partial v / partial theta
		jac[v_offset+j++] = b6*inv_y3 + b5*y2*inv_y3_square;
		// partial u / partial psi
		jac[j] = (t2-y2)*inv_y3;
		// partial v / partial psi
		jac[v_offset+j++] = (y1-t1)*inv_y3;
		// partial u / partial x
		jac[j] = inv_y3;
		// partial v / partial x
		jac[v_offset+j++] = 0;
		// partial u / partial y
		jac[j] = 0;
		// partial v / partial y
		jac[v_offset+j++] = inv_y3;
		// partial u / partial z
		jac[j] = -y1*inv_y3_square;
		// partial v / partial z
		jac[v_offset+j++] = -y2*inv_y3_square;
	}
}

template<typename T>
void levmar_approx_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data) {
	levmar_approx_ideal_pinhole_euler_jac(p, jac, m, n, data);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;

	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const int num_points = n/2;
	const unsigned int v_offset = m*num_points;
	// multiply partial u / partial p with fx
	for(unsigned int i=0; i<v_offset; i++) {
		jac[i] *= fx;
	}
	// multiply partial v / partial p with fy
	for(unsigned int i=v_offset; i<2*v_offset; i++) {
		jac[i] *= fy;
	}
}

template<typename T>
void levmar_pinhole_euler_jac(T *p, T *jac, int m, int n, void *data) {
	levmar_ideal_pinhole_euler_jac(p, jac, m, n, data);

	const pinhole_model_data_t<T> *pinhole_model_data = static_cast< pinhole_model_data_t<T>* >(data);
	if(!pinhole_model_data) return;

	const cv::Mat &camera_matrix = pinhole_model_data->camera_matrix;
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const int num_points = n/2;
	const unsigned int v_offset = m*num_points;
	// multiply partial u / partial p with fx
	for(unsigned int i=0; i<v_offset; i++) {
		jac[i] *= fx;
	}
	// multiply partial v / partial p with fy
	for(unsigned int i=v_offset; i<2*v_offset; i++) {
		jac[i] *= fy;
	}
}

// ----------------------------------------------------------------------------
// Wrapper
// ----------------------------------------------------------------------------
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

	if(jacf)
		return slevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata);

	return slevmar_dif(func, p, x, m, n, itmax, opts, info, work, covar, adata);
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

	if(jacf)
		return dlevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata);

	return dlevmar_dif(func, p, x, m, n, itmax, opts, info, work, covar, adata);
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_POSE_H_
