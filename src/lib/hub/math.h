/****************************************************************************
** Copyright 2012 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file math.h
 * \date created at 2012/05/16
 *
 * \brief Library of math routines.
 */

#ifndef _HUB_MATH_H_
#define _HUB_MATH_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <cerrno>
#include <cmath>	//pow
#include <cstdlib>	//abs
#include <vector>
#include <algorithm>	//nth_element
#include <limits>

namespace hub {

/// Estimation of \f$\pi\f$
const double pi = 3.1415926535897932384626433832795028841971693993751058209749;

/**
 * \brief Convert degrees to radians.
 *
 * Does the simple math of multiplying the input by pi and dividing by 180.
 * \param[in] d Angle in degrees.
 * \return Angle in radians.
 * \sa T rad2deg
 */
template <typename T>
T deg2rad(const T &d);

/**
 * \brief Converts euler angles to quaternions.
 * \param[in] euler_angle Euler angles as array.
 * \param[out] quaternion Quaternion as array.
 * \return Pointer to quaternion array.
 * \sa quaternion_to_euler
 */
template <typename T>
T* euler_to_quaternion(const T euler_angle[3], T quaternion[4]);

/**
 * \brief Check if value val is in range (low <= x <= high)
 * \param[in] value x
 * \param[in] low range limit
 * \param[in] high range limit
 * \return true (1) if x is in specified range
 */
template <typename T>
int in_range(const T &x, const T &low, const T &high);

/**
 * \brief Calculates the intersection of a line and a plane.
 * \param[in] plane_point
 * \param[in] plane_normal_vector
 * \param[in] line_point
 * \param[in] line_direction
 * \param[out] intersection 3D intersection point of given line and plane.
 * \return 0 if line intersects plane, otherwise -1.
 */
template <typename T>
int intersection(const T plane_point[3],
	const T plane_normal_vector[3],
	const T line_point[3],
	const T line_direction[3],
	T intersection[3]);

/**
 * \brief Calculates MAD (Median Absolute Deviation).
 *
 * Keep in mind the cost of the copy constructor which is needed to make this
 * function safe to use.
 * \param[in] values Data vector.
 * \return MAD
 * \sa _mad(std::vector<T>&)
 */
template <typename T>
T mad(std::vector<T> values);

/**
 * \brief Calculates MAD (Median Absolute Deviation) and median.
 *
 * Keep in mind the cost of the copy constructor which is needed to make this
 * function safe to use.
 * \param[in] values Data vector.
 * \param[out] median Median of values.
 * \return MAD
 * \sa _mad(std::vector<T>&,T&)
 */
template <typename T>
T mad(std::vector<T> values, T &median);

/**
 * \brief Calculates MAD (Median Absolute Deviation) without copy constructor.
 * \param[in,out] values Data vector.
 * \return MAD.
 * \warning The input vector gets altered.
 * \sa mad(std::vector<T> values);
 */
template <typename T>
T _mad(std::vector<T> &values);

/**
 * \brief Calculates MAD (Median Absolute Deviation) and median without copy constructor.
 * \param[in,out] values Data vector.
 * \param median[out] Median of values.
 * \return MAD
 * \warning The input vector gets altered.
 * \sa mad(std::vector<T>,T&);
 */
template <typename T>
T _mad(std::vector<T> &values, T &median);

/**
 * \brief Calculates mean of data vector.
 * \param[in] values Data vector.
 * \return Mean.
 */
template <typename T>
T mean(const std::vector<T> &values);

/**
 * \brief Find median in linear time.
 *
 * Determine the median of the input vector. Keep in mind the cost of the
 * copy constructor which is needed to make this function safe to use.
 * \param[in] values Data vector.
 * \sa _median(std::vector<T>&);
 */
template <typename T>
T median(std::vector<T> values);

/**
 * \brief Find median in linear time without copy constructor.
 * \param[in,out] values Data vector.
 * \return Median
 * \warning The input vector gets altered.
 * \sa median(std::vector<T>)
 */
template <typename T>
T _median(std::vector<T> &values);

/**
 * \brief Multiply 3x3 matrix with 3D vector.
 * \param[in] matrix 3x3 multiplication matrix.
 * \param[in] input 3D input vector.
 * \param[out] output 3D output vector as a result of the multiplication.
 */
template<typename T>
void multiply(const T matrix[9], const T input[3], T output[3]);

/**
 * \brief Multiply 2 matrices \a C = \a A \a B.
 */
template <typename T>
T* multiply_matrix(const T A[9], const T B[9], T C[9]);

/**
 * \brief Multiply 2 quaternions.
 * \param[in] lhs Left hand side factor.
 * \param[in] rhs Right hand side factor.
 * \param[out] product product = lhs*rhs.
 * \return Pointer to product.
 */
template <typename T>
T* multiply_quaternion(const T lhs[4], const T rhs[4], T product[4]);

/**
 * \brief Normalize data vector so it has unit length.
 * \param[in,out] data Data vector
 * \param[in] n Number of elemnts in \a data.
 */
template <typename T>
void normalize(T *data, const size_t n);

/**
 * \brief Converts quaternions to euler angles.
 * \param[in] quaternion Quaternion as array.
 * \param[out] euler_angle Euler angles as array.
 * \return Pointer to \a euler_angle array.
 * \sa euler_to_quaternion(const T euler_angle[3], T quaternion[4])
 */
template <typename T>
T* quaternion_to_euler(const T quaternion[4], T euler_angle[3]);

/**
 * \brief Convert radians to degrees.
 *
 * Does the simple math of multiplying the input by 180 and dividing by pi.
 * \param[in] r Angle in radians.
 * \return Angle in degrees.
 * \sa deg2rad(const T &d)
 */
template <typename T>
T rad2deg(const T &r);

/**
 * \brief Estimate deviation using MAD.
 *
 * Estimate the deviation of the input vector using MAD which
 * is known to be robust.
 * \param[in] values Data vector.
 * \sa _robust_sigma(std::vector<T> &values)
 */
template<typename T>
T robust_sigma(const std::vector<T> &values);

/**
 * \brief Estimate deviation using MAD.
 * 
 * Estimate the deviation of the input vector using MAD which 
 * is known to be robust and return also the median.
 * \param[in] values Data vector.
 * \param[out] median Median of \a values.
 * \sa _robust_sigma(std::vector<T> &values, T &median)
 */
template<typename T>
T robust_sigma(const std::vector<T> &values, T &median);

/**
 * \copydoc robust_sigma(const std::vector<T>&)
 * \param[in] values Data vector.
 * \warning The input vector gets altered.
 * \sa robust_sigma(const std::vector<T> &values)
 */
template<typename T>
T _robust_sigma(std::vector<T> &values);

/**
 * \copydoc robust_sigma(const std::vector<T>&,T&)
 * \param[in] values Data vector.
 * \param[out] median Median of \a values.
 * \sa robust_sigma(std::vector<T> &values, T &median)
 */
template<typename T>
T _robust_sigma(std::vector<T> &values, T &median);

/**
 * \brief Calculates rotation matrix.
 *
 * Based on <a href="http://gentlenav.googlecode.com/files/EulerAngles.pdf">EulerAngles.pdf</a>.
 * The rotations are applied in the following order:
 * <ol>
 * <li>Rotate about z-Axis through yaw-angle (\a rotation_vector_deg[2]).</li>
 * <li>Rotate about y-Axis through pitch-angle (\a rotation_vector_deg[1]).</li>
 * <li>Rotate about x-Axis through roll-angle (\a rotation_vector_deg[0]).</li>
 * </ol>
 * \param[in] rotation_vector_deg 3D rotation vector with euler angles in degrees.
 * \param[out] rotation_matrix 3x3 rotation matrix.
 * \sa rotation_matrix_quat
 * \sa rotation_matrix_quatvec
 * \sa rotation_matrix_rad
 */
template<typename T>
void rotation_matrix_deg(const T rotation_vector_deg[3], T rotation_matrix[9]);

/**
 * \copydoc rotation_matrix_deg
 * \param[in] rotation_quaternion unit quaternion describing rotation.
 * \param[out] rotation_matrix 3x3 rotation matrix.
 * \sa rotation_matrix_deg
 * \sa rotation_matrix_quatvec
 * \sa rotation_matrix_rad
 */
template<typename T>
void rotation_matrix_quat(const T rotation_quaternion[4], T rotation_matrix[9]);

/**
 * \copydoc rotation_matrix_deg
 * \param[in] rotation_quatvec Quaternion vector part describing rotation.
 * \param[out] rotation_matrix 3x3 rotation matrix.
 * \sa rotation_matrix_deg
 * \sa rotation_matrix_quat
 * \sa rotation_matrix_rad
 */
template<typename T>
void rotation_matrix_quatvec(const T rotation_quatvec[3], T rotation_matrix[9]);

/**
 * \copydoc rotation_matrix_deg
 * \param[in] rotation_vector_rad 3D rotation vector with euler angles in radians.
 * \param[out] rotation_matrix 3x3 rotation matrix.
 * \sa rotation_matrix_deg
 * \sa rotation_matrix_quat
 * \sa rotation_matrix_quatvec
 */
template<typename T>
void rotation_matrix_rad(const T rotation_vector_rad[3], T rotation_matrix[9]);

/**
 * \brief Estimate standard deviation.
 *
 * Estimates standard deviation by uncorrected sample standard deviation formula.
 * \param[in] values Data vector.
 * \return Standard deviation of \a values.
 * \warning This is known to be not robust.
 * \sa robust_sigma(const std::vector<T>&)
 */
template <typename T>
T std_dev(const std::vector<T> &values);

/**
 * \brief Calculate weight of Tukey M-estimator.
 * \param[in] x Data value (residual)
 * \param[in] sigma Deviation of the set \a x is belonging to.
 * \return Weight
 */
template<typename T>
T tukey_weight(const T x, const T sigma = 1.0);

/**
 * \brief Transforms 3-dimensional quaternion vector to 4-dimension unit quaternion by adding scalar part.
 * \param[in] v Vector part of quaternion
 * \param[out] q Unit quaternion consisting of scalar and vector part.
 */
template<typename T>
void vec2quat(const T v[3], T q[4]);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
template <typename T>
inline T deg2rad(const T &d) {
	return (d*pi) / 180;
}

template <typename T>
T* euler_to_quaternion(const T euler_angle[3], T quaternion[4]) {
	const T crh = cos(euler_angle[0]/2); // cosinus roll (phi) half
	const T cph = cos(euler_angle[1]/2); // cosinus pitch (theta) half
	const T cyh = cos(euler_angle[2]/2); // cosinus yaw (psi) half
	const T srh = sin(euler_angle[0]/2); // sinus roll (phi) half
	const T sph = sin(euler_angle[1]/2); // sinus pitch (theta) half
	const T syh = sin(euler_angle[2]/2); // sinus yaw (psi) half

	const T crhcph = crh*cph;
	const T crhsph = crh*sph;
	const T srhsph = srh*sph;
	const T srhcph = srh*cph;

	quaternion[0] = crhcph*cyh + srhsph*syh;
	quaternion[1] = srhcph*cyh - crhsph*syh;
	quaternion[2] = crhsph*cyh + srhcph*syh;
	quaternion[3] = crhcph*syh - srhsph*cyh;

// 	quaternion[0] = crhcph*cyh - srhsph*syh;
// 	quaternion[1] = crhsph*syh + srhcph*cyh;
// 	quaternion[2] = crhsph*cyh - srhcph*syh;
// 	quaternion[3] = crhcph*syh + srhsph*cyh;

// 	quaternion[0] = crhcph*cyh - srhsph*syh;
// 	quaternion[1] = crhsph*syh + srhcph*cyh;
// 	quaternion[2] = crhsph*cyh + srhcph*syh;
// 	quaternion[3] = crhcph*syh - srhsph*cyh;

	// normalization not necessary
// 	normalize(quaternion, 4);

	return quaternion;
}

template <typename T>
inline int in_range(const T &x, const T &low, const T &high) {
	return x >= low && x <= high;
}

template <typename T>
int intersection(const T plane_point[3],
	const T plane_normal_vector[3],
	const T line_point[3],
	const T line_direction[3],
	T intersection[3]) {

	T distance = line_direction[0]*plane_normal_vector[0]
		+ line_direction[1]*plane_normal_vector[1]
		+ line_direction[2]*plane_normal_vector[2];

	if(distance == 0) { // denominator is zero
		distance = (plane_point[0]-line_point[0])*plane_normal_vector[0]
			+ (plane_point[1]-line_point[1])*plane_normal_vector[1]
			+ (plane_point[2]-line_point[2])*plane_normal_vector[2];

		if(distance == 0) {// numerator is zero
			//line is is outsite and parallel to plane
			return -1;
		}

		// line lies in plane
		intersection[0] = line_point[0];
		intersection[1] = line_point[1];
		intersection[2] = line_point[2];

		return 0;
	}

	distance = ( (plane_point[0]-line_point[0])*plane_normal_vector[0]
		+ (plane_point[1]-line_point[1])*plane_normal_vector[1]
		+ (plane_point[2]-line_point[2])*plane_normal_vector[2] )
		/ distance;

	intersection[0] = line_point[0] + distance*line_direction[0];
	intersection[1] = line_point[1] + distance*line_direction[1];
	intersection[2] = line_point[2] + distance*line_direction[2];

	return 0;
}

template <typename T>
inline T mad(std::vector<T> values) {
	return _mad(values);
}

template <typename T>
inline T mad(std::vector<T> values, T &median) {
	return _mad(values, median);
}

template <typename T>
T _mad(std::vector<T> &values) {
	T med = _median(values);
	for(typename std::vector<T>::iterator iter = values.begin(); iter != values.end(); ++iter) {
		*iter = std::abs(*iter - med);
	}
	return _median(values);
}

template <typename T>
T _mad(std::vector<T> &values, T &median) {
	median = _median(values);
	for(typename std::vector<T>::iterator iter = values.begin(); iter != values.end(); ++iter) {
		*iter = std::abs(*iter - median);
	}
	return _median(values);
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

template <typename T>
inline T median(std::vector<T> values) {
	return _median(values);
}

template <typename T>
T _median(std::vector<T> &values) {
	typename std::vector<T>::iterator first = values.begin();
	typename std::vector<T>::iterator last = values.end();
	typename std::vector<T>::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last);
	return *middle;
}

template<typename T>
inline void multiply(const T matrix[9], const T input[3], T output[3]) {
	output[0] = matrix[0]*input[0] + matrix[1]*input[1] + matrix[2]*input[2];
	output[1] = matrix[3]*input[0] + matrix[4]*input[1] + matrix[5]*input[2];
	output[2] = matrix[6]*input[0] + matrix[7]*input[1] + matrix[8]*input[2];
}

template <typename T>
T* multiply_matrix(const T A[9], const T B[9], T C[9]) {
	C[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
	C[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
	C[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];

	C[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
	C[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
	C[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];

	C[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
	C[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
	C[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];

	return C;
}

template <typename T>
T* multiply_quaternion(const T lhs[4], const T rhs[4], T product[4]) {

	const T t1 = (lhs[0]+lhs[1])*(rhs[0]+rhs[1]);
	const T t2 = (lhs[3]-lhs[2])*(rhs[2]-rhs[3]);
	const T t3 = (lhs[1]-lhs[0])*(rhs[2]+rhs[3]);
	const T t4 = (lhs[2]+lhs[3])*(rhs[1]-rhs[0]);
	const T t5 = (lhs[1]+lhs[3])*(rhs[1]+rhs[2]);
	const T t6 = (lhs[1]-lhs[3])*(rhs[1]-rhs[2]);
	const T t7 = (lhs[0]+lhs[2])*(rhs[0]-rhs[3]);
	const T t8 = (lhs[0]-lhs[2])*(rhs[0]+rhs[3]);
	const T t9 = 0.5*(t5-t6+t7+t8);

	product[0] = t2 + t9-t5;
	product[1] = t1 - t9-t6;
	product[2] = -t3 + t9-t8;
	product[3] = -t4 + t9-t7;

	return product;
}

template <typename T>
void normalize(T *data, const size_t n) {

	// calculate norm
	T norm = 0;
	for(unsigned int i=0; i<n; i++) {
		norm += data[i]*data[i];
	}
	norm = sqrt(norm);

	//avoid division by 0
	if( norm <= std::numeric_limits<T>::min() )
		return;

	//normalize
	for(unsigned int i=0; i<n; i++) {
		data[i] /= norm;
	}
}

template <typename T>
T* quaternion_to_euler(const T quaternion[4], T euler_angle[3]) {
	const T q0 = quaternion[0];
	const T q1 = quaternion[1];
	const T q2 = quaternion[2];
	const T q3 = quaternion[3];

	const T singularity_test = q0*q2 - q1*q3;
	if(singularity_test > 0.499) {
		euler_angle[0] = 2*atan2(q0, q2);
		euler_angle[1] = pi/2;
		euler_angle[2] = 0;
		return euler_angle;
	}
	if(singularity_test < -0.499) {
		euler_angle[0] = 2*atan(q0/q2);
		euler_angle[1] = -pi/2;
		euler_angle[2] = 0;
		return euler_angle;
	}

	const T q0q0 = q0*q0;
	euler_angle[0] = atan2(q0*q1 + q2*q3, q0q0 + q3*q3 - 0.5);
	euler_angle[1] = asin( 2*singularity_test );
	euler_angle[2] = atan2(q0*q3 + q1*q2, q0q0 + q1*q1 - 0.5);

	return euler_angle;
}

template <typename T>
inline T rad2deg(const T &r) {
	return (r*180) / pi;
}

template<typename T>
inline T robust_sigma(const std::vector<T> &values) {
	const T sigma = 1.4862 * mad(values);
	return sigma;
}

template<typename T>
inline T robust_sigma(const std::vector<T> &values, T &median) {
	const T sigma = 1.4862 * mad(values, median);
	return sigma;
}

template<typename T>
inline T _robust_sigma(std::vector<T> &values) {
        T sigma = 1.4862 * _mad(values);
        return sigma;
}

template<typename T>
inline T _robust_sigma(std::vector<T> &values, T &median) {
        T sigma = 1.4862 * _mad(values, median);
        return sigma;
}

template<typename T>
void rotation_matrix_deg(const T rotation_vector_deg[3], T rotation_matrix[9]) {
	T rotation_vector_rad[3];
	rotation_vector_rad[0] = deg2rad(rotation_vector_deg[0]);
	rotation_vector_rad[1] = deg2rad(rotation_vector_deg[1]);
	rotation_vector_rad[2] = deg2rad(rotation_vector_deg[2]);

	rotation_matrix_rad(rotation_vector_rad, rotation_matrix);
}

template<typename T>
void rotation_matrix_quat(const T rotation_quaternion[4], T rotation_matrix[9]) {
	const T q0 =rotation_quaternion[0];
	const T q1 =rotation_quaternion[1];
	const T q2 =rotation_quaternion[2];
	const T q3 =rotation_quaternion[3];

	const T q0q0 = q0*q0;
	const T q1q1 = q1*q1;
	const T q2q2 = q2*q2;
	const T q3q3 = q3*q3;
	rotation_matrix[0] = 2*(q0q0 + q1q1) - 1;
	rotation_matrix[4] = 2*(q0q0 + q2q2) - 1;
	rotation_matrix[8] = 2*(q0q0 + q3q3) - 1;
	const T q0q3 = q0*q3;
	const T q1q2 = q1*q2;
	rotation_matrix[1] = -2*(q0q3 - q1q2);
	rotation_matrix[3] = 2*(q0q3 + q1q2);
	const T q0q2 = q0*q2;
	const T q1q3 = q1*q3;
	rotation_matrix[2] = 2*(q0q2 + q1q3);
	rotation_matrix[6] = -2*(q0q2 - q1q3);
	const T q0q1 = q0*q1;
	const T q2q3 = q2*q3;
	rotation_matrix[5] = -2*(q0q1 - q2q3);
	rotation_matrix[7] = 2*(q0q1 + q2q3);
}

template<typename T>
inline void rotation_matrix_quatvec(const T rotation_quatvec[3], T rotation_matrix[9]) {
	T quaternion[4];
	vec2quat(rotation_quatvec, quaternion);
	rotation_matrix_quat(quaternion, rotation_matrix);
}

template<typename T>
void rotation_matrix_rad(const T rotation_vector_rad[3], T rotation_matrix[9]) {
	const T phi_rad = rotation_vector_rad[0];
	const T theta_rad = rotation_vector_rad[1];
	const T psi_rad = rotation_vector_rad[2];

	rotation_matrix[0] = cos(theta_rad)*cos(psi_rad);
	rotation_matrix[1] = -cos(phi_rad)*sin(psi_rad) + sin(phi_rad)*sin(theta_rad)*cos(psi_rad);
	rotation_matrix[2] = sin(phi_rad)*sin(psi_rad) + cos(phi_rad)*sin(theta_rad)*cos(psi_rad);

	rotation_matrix[3] = cos(theta_rad)*sin(psi_rad);
	rotation_matrix[4] = cos(phi_rad)*cos(psi_rad) + sin(phi_rad)*sin(theta_rad)*sin(psi_rad);
	rotation_matrix[5] = -sin(phi_rad)*cos(psi_rad) + cos(phi_rad)*sin(theta_rad)*sin(psi_rad);

	rotation_matrix[6] = -sin(theta_rad);
	rotation_matrix[7] = sin(phi_rad)*cos(theta_rad);
	rotation_matrix[8] = cos(phi_rad)*cos(theta_rad);
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

template<typename T>
inline T tukey_weight(const T x, const T sigma) {
	const double c = sigma*4.6851;

	if(std::abs(x) > c)
		return  0.0;

	const T tmp = 1 - (x*x)/(c*c);
	return tmp*tmp;
}

template<typename T>
inline void vec2quat(const T v[3], T q[4]) {
	q[1] = v[0];
	q[2] = v[1];
	q[3] = v[2];
	errno = 0;
	q[0] = sqrt(1.0 - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
	if(EDOM == errno)
		q[0] = 0.0;
}

} // namespace hub

#endif // _HUB_MATH_H_
