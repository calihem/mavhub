/****************************************************************************
** Copyright 2013 Humboldt-Universitaet zu Berlin
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
 * \file camera.h
 * \date created at 2013/02/13
 * \author Michael Schulz
 *
 * \brief Library to handle camera projections.
 * 
 * This library contains methods to calculate the projections of 3D points
 * to an camera image using the pinhole model. It supports both euler and
 * quaternion rotation representations and allows the computing of the
 * corresponding jacobians.
 * 
 * The used coordinate system follows an East-North-Down convention, i.e.
 * \code 
 * y ^
 *   |
 *   |
 *   X---> x
 *  z
 * \endcode
 * For the methods using the euler angles this means, that the elements of the 
 * parameter vector p = [phi theta psi x y z] describes a rotation around x-axis (phi),
 * y-axis (theta) and z-axis (psi) and a translation of [x y z].
 */

#ifndef _HUB_CAMERA_H_
#define _HUB_CAMERA_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

// FIXME remove dependency of opencv
#ifdef HAVE_OPENCV2
#include <opencv/cv.h>
#endif // HAVE_OPENCV2

//TODO swap position of rotation and translation information in parameter vector so that model functions can be more generalized by templates

#include "lib/hub/math.h"

namespace hub {
namespace slam {
// ----------------------------------------------------------------------------
// Ideal pinhole model functions
// ----------------------------------------------------------------------------
/**
 * \brief Calculates the projectios of 3D points to ideal 2D image points using euler angles.
 * \param[in] objectpoints Pointer to 3D object points.
 * \param[in] rt Rotation + translation vector.
 * \param[out] idealpoints Pointer to 2D projections of objectpoints.
 * \param[in] n number of objectpoints
 */
template<typename T, void(*R)(const T[3], T[9])>
void ideal_pinhole_model(const T *objectpoints,
		const T rt[6],
		T *idealpoints,
		const size_t n = 1);

/**
 * \brief Calculates the jacobian of the ideal pinhole projection using euler angles.
 * \param[in] objectpoint 3D object point
 * \param[in] rt Rotation + translation vector
 * \param[out] jacmRT Jacobian of rotation and translation \f$ \frac{\partial f}{\partial qr \partial r}\f$.
 * \param[out] jacmS Jacobian of structure \f$ \frac{\partial f}{\partial opject\_point}\f$.
 */
template<typename T>
void ideal_pinhole_model_euler_jac(const T objectpoints[3],
		const T rt[6],
		T jac_u[6],
		T jac_v[6]);

/**
 * \brief Inverse camera model to use with euler angles or quatvec.
 * \param[in] idealpoints Image projections of in ideal coordinates.
 * \param[in] rt Parameter vector containing rotation and translation 
 * \param[in] distance Meassured distance to observed object, i.e. objectpoints[i].z = distance - rt[5].
 * \param[out] objectpoints 3D coordinates of \a idealpoints.
 * \param[in] n Number of idealpoints
 * \sa inverse_ideal_pinhole_model_quat
 */
template<typename T, void(*R)(const T[3], T[9])>
void inverse_ideal_pinhole_model(const T *idealpoints,
		const T rt[6],
		const T &distance,
		T *objectpoints,
		const size_t n = 1);

/**
 * \brief Calculates the projection of a 3D point to ideal 2D image point using quaternions.
 * \param[in] objectpoint 3D object point.
 * \param[in] qt quaternions + translation vector.
 * \param[out] idealpoint 2D projection of objectpoint.
 */
template<typename T>
void ideal_pinhole_model_quat(const T objectpoint[3],
		const T qt[7],
		T idealpoint[2]);

/**
 * \brief Calculates the projections of 3D points to ideal 2D image points using quaternions.
 * \param[in] objectpoints Pointer to 3D object points.
 * \param[in] qt quaternions + translation vector.
 * \param[out] idealpoints Pointer to 2D projections of objectpoints.
 * \param[in] n number of objectpoints
 */
template<typename T>
void ideal_pinhole_model_quat(const T *objectpoints,
		const T qt[7],
		T *idealpoints,
		const size_t n);

/**
 * \brief Inverse camera model to use with euler angles or quatvec.
 * \param[in] idealpoints Image projections of in ideal coordinates.
 * \param[in] qt Parameter vector containing quaternions and translation 
 * \param[in] distance Meassured distance to observed object, i.e. objectpoints[i].z = distance - rt[5].
 * \param[out] objectpoints 3D coordinates of \a idealpoints.
 * \param[in] n Number of idealpoints
 * \sa inverse_ideal_pinhole_model
 */
template<typename T>
void inverse_ideal_pinhole_model_quat(const T *idealpoints,
		const T qt[7],
		const T &distance,
		T *objectpoints,
		const size_t n = 1);

/**
 * \brief Calculates the jacobian of the ideal pinhole projection.
 * \param[in] objectpoint 3D object point
 * \param[in] qr rotation quaternion vector part
 * \param[in] t translation vector
 * \param[out] jacmRT Jacobian of rotation and translation \f$ \frac{\partial f}{\partial qr \partial r}\f$.
 * \param[out] jacmS Jacobian of structure \f$ \frac{\partial f}{\partial opject\_point}\f$.
 */
template<typename T>
void ideal_pinhole_model_quatvec_jac(const T objectpoint[3],
		const T qr[3],
		const T t[3],
		T jacmRT[2][6],
		T jacmS[2][3]);

// ----------------------------------------------------------------------------
// Pinhole model functions
// ----------------------------------------------------------------------------
#if CV_MINOR_VERSION >= 2
template<typename T>
void pinhole_model_euler(const T *objectpoints,
		const T rt[6],
		const cv::Mat &camera_matrix,
		T *imagepoints,
		const size_t n = 1);

/**
 * \brief Calculates the projection of a 3D point to a 2D image point.
 * \param[in] objectpoint 3D object point
 * \param[in] q rotation quaternion
 * \param[in] t translation vector
 * \param[in] camera_matrix intrinsic params
 * \param[out] imagepoint 2D projection of M
 */
template<typename T>
void pinhole_model_quat(const T *objectpoints,
		const T qt[7],
		const cv::Mat &camera_matrix,
		T *imagepoints,
		const size_t n = 1);

template<typename T>
inline void pinhole_model_quatvec(const T *objectpoints,
		const T qt[6],
		const cv::Mat &camera_matrix,
		T *imagepoints,
		size_t n = 1);

/**
 * \brief Calculates the jacobian of the pinhole projection.
 * \param[in] objectpoint 3D object point
 * \param[in] qr rotation quaternion vector part
 * \param[in] t translation vector
 * \param[in] camera_matrix intrinsic params
 * \param[in] qr0 quaternion rotation estimation
 * \param[out] jacmRT Jacobian of rotation and translation \f$ \frac{\partial f}{\partial qr \partial r}\f$.
 * \param[out] jacmS Jacobian of structure \f$ \frac{\partial f}{\partial opject\_point}\f$.
 */
template<typename T>
void pinhole_model_quatvec_jac(const T objectpoint[3],
		const T qr[3],
		const T t[3],
		const cv::Mat &camera_matrix,
		T jacmRT[2][6],
		T jacmS[2][3]);
#endif // CV_MINOR_VERSION

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
template<typename T, void(*R)(const T[3], T[9])>
void ideal_pinhole_model(const T *objectpoints,
		const T rt[6],
		T *idealpoints,
		size_t n) {

	// determine rotation matrix
	T rotation_matrix[9];
	R(rt, rotation_matrix);

	T rotated_point[3];
	for(size_t i=0; i<n; i++) {
		// rotate 3D points
		multiply(rotation_matrix, &objectpoints[i*3], rotated_point);

		rotated_point[2] += rt[5];
		idealpoints[i*2] = rotated_point[0] + rt[3];
		idealpoints[i*2 + 1] = rotated_point[1] + rt[4];
		// avoid divding by (almost) zero for points near camera
		if(rotated_point[2] <= 0.001)
			continue;

		idealpoints[i*2] /= rotated_point[2];
		idealpoints[i*2 + 1] /= rotated_point[2];
	}
}

template<typename T>
void ideal_pinhole_model_euler_jac(const T objectpoints[3],
		const T rt[6],
		T jac_u[6],
		T jac_v[6]) {

	const T phi = rt[0];
	const T theta = rt[1];
	const T psi = rt[2];
	const T t1 = rt[3];
	const T t2 = rt[4];
	const T t3 = rt[5];

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

	const T x1 = objectpoints[0];
	const T x2 = objectpoints[1];
	const T x3 = objectpoints[2];

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
	jac_u[0] = -b2*inv_y3 + b3*y1*inv_y3_square;
	// partial v / partial phi
	jac_v[0] = b1*inv_y3 + b3*y2*inv_y3_square;
	// partial u / partial theta
	jac_u[1] = -(b4*inv_y3 + b5*y1*inv_y3_square);
	// partial v / partial theta
	jac_v[1] = -(b6*inv_y3 + b5*y2*inv_y3_square);
	// partial u / partial psi
	jac_u[2] = (y2-t2)*inv_y3;
	// partial v / partial psi
	jac_v[2] = (t1-y1)*inv_y3;
	// partial u / partial x
	jac_u[3] = -inv_y3;
	// partial v / partial x
	jac_v[3] = 0;
	// partial u / partial y
	jac_u[4] = 0;
	// partial v / partial y
	jac_v[4] = -inv_y3;
	// partial u / partial z
	jac_u[5] = y1*inv_y3_square;
	// partial v / partial z
	jac_v[5] = y2*inv_y3_square;
}

template<typename T, void(*R)(const T[3], T[9])>
void inverse_ideal_pinhole_model(const T *idealpoints,
		const T rt[6],
		const T &distance,
		T *objectpoints,
		const size_t n) {

	const T t1 = rt[3];
	const T t2 = rt[4];
	const T t3 = rt[5];

	// rotation matrix is orthogonal => R^{-1} = R^t
	T rotation_matrix[9];
	R(&rt[0], rotation_matrix);
	const T r11 = rotation_matrix[0]; const T r12 = rotation_matrix[1]; const T r13 = rotation_matrix[2];
	const T r21 = rotation_matrix[3]; const T r22 = rotation_matrix[4]; const T r23 = rotation_matrix[5];
	const T r31 = rotation_matrix[6]; const T r32 = rotation_matrix[7]; const T r33 = rotation_matrix[8];

	const T x3 = distance - t3;
	const T y3_numerator = x3 + r13*t1 + r23*t2 + r33*t3;

	for(unsigned int i=0; i<n; i++) {
		const T u = idealpoints[i*2];
		const T v = idealpoints[i*2+1];
		const T y3 = y3_numerator / (r13*u + r23*v + r33);
		const T tmp1 = y3*u-t1;
		const T tmp2 = y3*v-t2;
		const T tmp3 = y3-t3;

		objectpoints[i*3]   = r11*tmp1 + r21*tmp2 + r31*tmp3;
		objectpoints[i*3+1] = r12*tmp1 + r22*tmp2 + r32*tmp3;
		objectpoints[i*3+2] = x3;
	}
}

template<typename T>
void ideal_pinhole_model_quat(const T objectpoint[3],
		const T qt[7],
		T idealpoint[2]) {

	const T q0 = qt[0];
	const T q1 = qt[1];
	const T q2 = qt[2];
	const T q3 = qt[3];

	const T p0 = objectpoint[0];
	const T p1 = objectpoint[1];
	const T p2 = objectpoint[2];

	const T t1 = -p0*q1-q2*p1-q3*p2;
	const T t2 = q0*p0+q2*p2-q3*p1;
	const T t3 = p1*q0+q3*p0-p2*q1;
	const T t4 = q0*p2+p1*q1-q2*p0;

	const T x = -q1*t1+q0*t2-t3*q3+q2*t4+qt[4];
	const T y = -q2*t1+q0*t3-t4*q1+q3*t2+qt[5];
	const T z = -t1*q3+q0*t4-q2*t2+q1*t3+qt[6];

// 	const T x = (1-2*(q2*q2+q3*q3))*p0 + 2*(q1*q2-q0*q3)*p1     + 2*(q0*q2+q1*q3)*p2     + t[0];
// 	const T y = 2*(q0*q3+q1*q2)*p0     + (1-2*(q1*q1+q3*q3))*p1 + 2*(q2*q3-q0*q1)*p2     + t[1];
// 	const T z = 2*(q1*q3-q0*q2)*p0     + 2*(q0*q1+q2*q3)*p1     + (1-2*(q1*q1+q2*q2))*p2 + t[2];

	if(z <= 0.001) {
		idealpoint[0] = x;
		idealpoint[1] = y;
		return;
	}

	idealpoint[0] = x/z;
	idealpoint[1] = y/z;
}

template<typename T>
void ideal_pinhole_model_quat(const T *objectpoints,
		const T qt[7],
		T *idealpoints,
		size_t n) {

	T rotation_matrix[9];
	rotation_matrix_quat(qt, rotation_matrix);
	T transformed_point[3];

	for(size_t i=0; i<n; i++) {
		multiply(rotation_matrix, &objectpoints[i*3], transformed_point);
		transformed_point[0] += qt[4];
		transformed_point[1] += qt[5];
		transformed_point[2] += qt[6];
// 		if( abs(transformed_point[2]) <= 0.0001) transformed_point[2] = 1;

		idealpoints[i*2] = transformed_point[0]/transformed_point[2];
		idealpoints[i*2+1] = transformed_point[1]/transformed_point[2];
	}
}

template<typename T>
void inverse_ideal_pinhole_model_quat(const T *idealpoints,
		const T qt[7],
		const T &distance,
		T *objectpoints,
		const size_t n) {

	const T t1 = qt[4];
	const T t2 = qt[5];
	const T t3 = qt[6];

	// rotation matrix is orthogonal => R^{-1} = R^t
	T rotation_matrix[9];
	rotation_matrix_quat(&qt[0], rotation_matrix);
	const T r11 = rotation_matrix[0]; const T r12 = rotation_matrix[1]; const T r13 = rotation_matrix[2];
	const T r21 = rotation_matrix[3]; const T r22 = rotation_matrix[4]; const T r23 = rotation_matrix[5];
	const T r31 = rotation_matrix[6]; const T r32 = rotation_matrix[7]; const T r33 = rotation_matrix[8];

	const T x3 = distance - t3;
	const T y3_numerator = x3 + r13*t1 + r23*t2 + r33*t3;

	for(unsigned int i=0; i<n; i++) {
		const T u = idealpoints[i*2];
		const T v = idealpoints[i*2+1];
		const T y3 = y3_numerator / (r13*u + r23*v + r33);
		const T tmp1 = y3*u-t1;
		const T tmp2 = y3*v-t2;
		const T tmp3 = y3-t3;

		objectpoints[i*3]   = r11*tmp1 + r21*tmp2 + r31*tmp3;
		objectpoints[i*3+1] = r12*tmp1 + r22*tmp2 + r32*tmp3;
		objectpoints[i*3+2] = x3;
	}
}

template<typename T>
void ideal_pinhole_model_quatvec_jac(const T objectpoint[3],
		const T qt[6],
		T jac_u[6],
		T jac_v[6]) {

	const T p0 = objectpoint[0];
	const T p1 = objectpoint[1];
	const T p2 = objectpoint[2];

	const T q1 = qt[0];
	const T q2 = qt[1];
	const T q3 = qt[2];
	const T q0 = sqrt(1.0-q1*q1-q2*q2-q3*q3);

	const T t56 = -q1/q0;
	const T t70 = t56*p0;
	const T t75 = t56*p1-p2;
	const T t81 = q0*p1+q3*p0-q1*p2;
	const T t87 = t56*p2+p1;
	const T t93 = q0*p2+q1*p1-q2*p0;
	const T t107 = p0*q2+t56*t81+q0*t75-t87*q1-t93+t70*q3;
	const T t118 = p0*q3+t56*t93+q0*t87-t70*q2+q1*t75+t81;
	const T t51 = -q1*p0-q2*p1-q3*p2;
	const T t65 = q0*p0+q2*p2-p1*q3;
	const T t126 = -q3*t51+q0*t93-t65*q2+t81*q1+qt[5];
	const T t127 = 1/t126;
	const T t145 = t126*t126;
	const T t147 = (-q1*t51+q0*t65-t81*q3+t93*q2+qt[3])/t145;
	jac_u[0] = -(p0*q1-t51+t56*t65+q0*t70-q3*t75+q2*t87)*t127+t118*t147;
	const T t141 = -t51*q2+q0*t81-t93*q1+t65*q3+qt[4];
	const T t159 = t141/t145;
	jac_v[0] = -t107*t127+t159*t118;
	const T t175 = -q2/q0;
	const T t180 = t175*p0+p2;
	const T t185 = t175*p1;
	const T t192 = t175*p2-p0;
	const T t206 = p1*q2-t51+t175*t81+q0*t185-t192*q1+q3*t180;
	const T t216 = p1*q3+t93*t175+q0*t192-t180*q2-t65+t185*q1;
	jac_u[1] = -(p1*q1+t175*t65+t180*q0-t185*q3+t192*q2+t93)*t127+t147*t216;
	jac_v[1] = -t206*t127+t159*t216;
	const T t240 = -q3/q0;
	const T t245 = t240*p0-p1;
	const T t250 = p1*t240+p0;
	const T t257 = t240*p2;
	const T t271 = p2*q2+t81*t240+q0*t250-t257*q1+t245*q3+t65;
	const T t281 = p2*q3-t51+t240*t93+q0*t257-t245*q2+t250*q1;
	jac_u[2] = -(p2*q1+t240*t65+q0*t245-t250*q3-t81+t257*q2)*t127+t147*t281;
	jac_v[2] = -t271*t127+t159*t281;
	jac_u[3] = -t127;
	jac_v[3] = 0.0;
	jac_u[4] = 0.0;
	jac_v[4] = jac_u[3];
	jac_u[5] = t147;
	jac_v[5] = t159;
}


template<typename T>
void ideal_pinhole_model_quatvec_jac(const T objectpoint[3],
		const T qr[3],
		const T t[3],
		T jacmRT[2][6],
		T jacmS[2][3]) {

	const T p0 = objectpoint[0];
	const T p1 = objectpoint[1];
	const T p2 = objectpoint[2];

	const T q1 = qr[0];
	const T q2 = qr[1];
	const T q3 = qr[2];
	const T q1q1 = q1*q1;
	const T q2q2 = q2*q2;
	const T q3q3 = q3*q3;
	const T q0q0 = 1.0-q1q1-q2q2-q3q3;
	const T q0 = sqrt(q0q0);

	const T t51 = -q1*p0-q2*p1-q3*p2;
	const T t56 = -q1/q0;
	const T t65 = q0*p0+q2*p2-p1*q3;
	const T t70 = t56*p0;
	const T t75 = t56*p1-p2;
	const T t81 = q0*p1+q3*p0-q1*p2;
	const T t87 = t56*p2+p1;
	const T t93 = q0*p2+q1*p1-q2*p0;
	const T t107 = p0*q2+t56*t81+q0*t75-t87*q1-t93+t70*q3;
	const T t118 = p0*q3+t56*t93+q0*t87-t70*q2+q1*t75+t81;
	const T t126 = -q3*t51+q0*t93-t65*q2+t81*q1+t[2];
	const T t127 = 1/t126;
	const T t141 = -t51*q2+q0*t81-t93*q1+t65*q3+t[1];
	const T t145 = t126*t126;
	const T t147 = (-q1*t51+q0*t65-t81*q3+t93*q2+t[0])/t145;
	jacmRT[0][0] = (p0*q1-t51+t56*t65+q0*t70-q3*t75+q2*t87)*t127-t118*t147;
	const T t159 = t141/t145;
	jacmRT[1][0] = t107*t127-t159*t118;
	const T t175 = -q2/q0;
	const T t180 = t175*p0+p2;
	const T t185 = t175*p1;
	const T t192 = t175*p2-p0;
	const T t206 = p1*q2-t51+t175*t81+q0*t185-t192*q1+q3*t180;
	const T t216 = p1*q3+t93*t175+q0*t192-t180*q2-t65+t185*q1;
	jacmRT[0][1] = (p1*q1+t175*t65+t180*q0-t185*q3+t192*q2+t93)*t127-t147*t216;
	jacmRT[1][1] = t206*t127-t159*t216;
	const T t240 = -q3/q0;
	const T t245 = t240*p0-p1;
	const T t250 = p1*t240+p0;
	const T t257 = t240*p2;
	const T t271 = p2*q2+t81*t240+q0*t250-t257*q1+t245*q3+t65;
	const T t281 = p2*q3-t51+t240*t93+q0*t257-t245*q2+t250*q1;
	jacmRT[0][2] = (p2*q1+t240*t65+q0*t245-t250*q3-t81+t257*q2)*t127-t147*t281;
	jacmRT[1][2] = t271*t127-t159*t281;
	jacmRT[0][3] = t127;
	jacmRT[1][3] = 0.0;
	jacmRT[0][4] = 0.0;
	jacmRT[1][4] = t127;
	jacmRT[0][5] = -t147;
	jacmRT[1][5] = -t159;
	const T t299 = q1*q2;
	const T t302 = -q0*q3;
	const T t303 = 2.0*(t299-t302);
	const T t305 = q1*q3;
	const T t306 = -q0*q2;
	const T t309 = t305+2.0*t306+q3*q1;
	jacmS[0][0] = (q1q1+q0q0-q3q3-q2q2)*t127-t147*t309;
	jacmS[1][0] = t303*t127-t159*t309;
	const T t325 = q2q2+q0q0-q1q1-q3q3;
	const T t327 = q2*q3;
	const T t330 = -q0*q1;
	const T t331 = 2.0*(t327-t330);
	jacmS[0][1] = 2.0*(t302+t299)*t127-t147*t331;
	jacmS[1][1] = t325*t127-t159*t331;
	const T t347 = t327+2.0*t330+t327;
	const T t350 = q3q3+q0q0-q2q2-q1q1;
	jacmS[0][2] = 2.0*(t305-t306)*t127-t147*t350;
	jacmS[1][2] = t347*t127-t159*t350;
}

#if CV_MINOR_VERSION >= 2
template<typename T>
inline void pinhole_model_euler(const T *objectpoints,
		const T rt[6],
		const cv::Mat &camera_matrix,
		T *imagepoints,
		size_t n) {

	ideal_pinhole_model<T, rotation_matrix_rad>(objectpoints, rt, imagepoints, n);

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	for(size_t i=0; i<n; i++) {
		imagepoints[i*2] = fx*imagepoints[i*2] + cx;
		imagepoints[i*2 + 1] = fy*imagepoints[i*2 + 1] + cy;
	}
}

template<typename T>
void pinhole_model_quat(const T *objectpoints,
		const T qt[7],
		const cv::Mat &camera_matrix,
		T *imagepoints,
		size_t n) {

	ideal_pinhole_model_quat(objectpoints, qt, imagepoints, n);

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	for(size_t i=0; i<n; i++) {
		imagepoints[i*2] = fx*imagepoints[i*2] + cx;
		imagepoints[i*2 + 1] = fy*imagepoints[i*2 + 1] + cy;
	}
}

template<typename T>
inline void pinhole_model_quatvec(const T *objectpoints,
		const T qt[6],
		const cv::Mat &camera_matrix,
		T *imagepoints,
		size_t n) {

	T pose[7];
	vec2quat(qt, pose);
	pose[4] = qt[3];
	pose[5] = qt[4];
	pose[6] = qt[5];
	pinhole_model_quat(objectpoints, pose, camera_matrix, imagepoints, n);
}

template<typename T>
void pinhole_model_quatvec_jac(const T objectpoint[3],
		const T qr[3],
		const T t[3],
		const cv::Mat &camera_matrix,
		T jacmRT[2][6],
		T jacmS[2][3]) {

	const T p0 = objectpoint[0];
	const T p1 = objectpoint[1];
	const T p2 = objectpoint[2];

	const T q1 = qr[0];
	const T q2 = qr[1];
	const T q3 = qr[2];
	const T q1q1 = q1*q1;
	const T q2q2 = q2*q2;
	const T q3q3 = q3*q3;
	const T q0q0 = 1.0-q1q1-q2q2-q3q3;
	const T q0 = sqrt(q0q0);

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const T t56 = -q1/q0;
	const T t70 = t56*p0;
	const T t75 = t56*p1-p2;
	const T t81 = q0*p1+q3*p0-q1*p2;
	const T t87 = t56*p2+p1;
	const T t93 = q0*p2+q1*p1-q2*p0;
	const T t107 = p0*q2+t56*t81+q0*t75-t87*q1-t93+t70*q3;
	const T t118 = p0*q3+t56*t93+q0*t87-t70*q2+q1*t75+t81;
	const T t51 = -q1*p0-q2*p1-q3*p2;
	const T t65 = q0*p0+q2*p2-p1*q3;
	const T t126 = -q3*t51+q0*t93-t65*q2+t81*q1+t[2];
	const T t127 = 1/t126;
	const T t141 = -t51*q2+q0*t81-t93*q1+t65*q3+t[1];
	const T t145 = t126*t126;
	const T t147 = (fx*(-q1*t51+q0*t65-t81*q3+t93*q2+t[0])+t126*cx)/t145;
	jacmRT[0][0] = (fx*(p0*q1-t51+t56*t65+q0*t70-q3*t75+q2*t87)+cx*t118)*t127-t118*t147;
	const T t159 = (fy*t141+t126*cy)/t145;
	jacmRT[1][0] = (t107*fy+cy*t118)*t127-t159*t118;
	const T t175 = -q2/q0;
	const T t180 = t175*p0+p2;
	const T t185 = t175*p1;
	const T t192 = t175*p2-p0;
	const T t206 = -p1*q2-t51+t175*t81+q0*t185-t192*q1+q3*t180;
	const T t216 = -p1*q3+t93*t175+q0*t192-t180*q2-t65+t185*q1;
	jacmRT[0][1] = (fx*(-p1*q1+t175*t65+t180*q0-t185*q3+t192*q2+t93)+cx*t216)*t127-t147*t216;
	jacmRT[1][1] = (fy*t206+cy*t216)*t127-t159*t216;
	const T t240 = -q3/q0;
	const T t245 = t240*p0-p1;
	const T t250 = p1*t240+p0;
	const T t257 = t240*p2;
	const T t271 = p2*q2+t81*t240+q0*t250-t257*q1+t245*q3+t65;
	const T t281 = p2*q3-t51+t240*t93+q0*t257-t245*q2+t250*q1;
	jacmRT[0][2] = (fx*(p2*q1+t240*t65+q0*t245-t250*q3-t81+t257*q2)+t281*cx)*t127-t147*t281;
	jacmRT[1][2] = (fy*t271+t281*cy)*t127-t159*t281;
	jacmRT[0][3] = t127*fx;
	jacmRT[1][3] = 0.0;
	jacmRT[0][4] = 0.0;
	jacmRT[1][4] = fy*t127;
	jacmRT[0][5] = cx*t127-t147;
	jacmRT[1][5] = cy*t127-t159;
	const T t299 = q1*q2;
	const T t302 = -q0*q3;
	const T t303 = 2.0*t299+q0*q3-t302;
	const T t305 = q1*q3;
	const T t306 = -q0*q2;
	const T t309 = t305+2.0*t306+q3*q1;
	jacmS[0][0] = (fx*(q1q1+q0q0-q3q3-q2q2)+cx*t309)*t127-t147*t309;
	jacmS[1][0] = (fy*t303+cy*t309)*t127-t159*t309;
	const T t325 = q2q2+q0q0+q1q1-q3q3;
	const T t327 = q3*q2;
	const T t330 = -q0*q1;
	const T t331 = 2.0*t327+q0*q1-t330;
	jacmS[0][1] = (fx*(t299+2.0*t302+q1*q2)+cx*t331)*t127-t147*t331;
	jacmS[1][1] = (fy*t325+cy*t331)*t127-t159*t331;
	const T t347 = t327+2.0*t330+q2*q3;
	const T t350 = q3q3+q0q0-q2q2-q1q1;
	jacmS[0][2] = (fx*(2.0*t305+q0*q2-t306)+t350*cx)*t127-t147*t350;
	jacmS[1][2] = (fy*t347+cy*t350)*t127-t159*t350;
}
#endif // CV_MINOR_VERSION

} // namespace slam
} // namespace hub

#endif // _HUB_CAMERA_H_
