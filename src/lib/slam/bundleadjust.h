#ifndef _HUB_BUNDLEADJUST_H_
#define _HUB_BUNDLEADJUST_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#if CV_MINOR_VERSION >= 2

#include <sba/sba.h>

namespace hub {
namespace slam {

static const int num_params_per_cam = 6;	// cnp
static const int num_params_per_point = 3;	// pnp
static const int num_params_per_measuremnt = 2;	// mnp
static const int quaternion_size = 4;

/**
 * \brief Struct containing the intrinsic camera parameters and the initial rotaion guess.
 */
template<typename T=double>
struct sba_model_data_t {
	sba_model_data_t(const cv::Mat &camera_matrix, const std::vector<T> &initial_rotations) :
		camera_matrix(camera_matrix),
		initial_rotations(initial_rotations) {};

	const cv::Mat &camera_matrix;			/// intrinsic camera parameters
	const std::vector<T> &initial_rotations;	/// inititial rotations expressed in quaternions
};

/**
 * functional relation describing measurements. Given a parameter vector p,
 * computes a prediction of the measurements \hat{x}. p is (m*cnp + n*pnp)x1,
 * \hat{x} is (n*m*mnp)x1, maximum
 * rcidxs, rcsubs are max(m, n) x 1, allocated by the caller and can be used
 * as working memory
 */
void sba_pinhole_model(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);

/** 
 * function to evaluate the sparse Jacobian dX/dp.
 * The Jacobian is returned in jac as
 * (dx_11/da_1, ..., dx_1m/da_m, ..., dx_n1/da_1, ..., dx_nm/da_m,
 *  dx_11/db_1, ..., dx_1m/db_1, ..., dx_n1/db_n, ..., dx_nm/db_n), or
 * (using HZ's notation),
 * jac=(A_11, B_11, ..., A_1m, B_1m, ..., A_n1, B_n1, ..., A_nm, B_nm)
 * Notice that depending on idxij, some of the A_ij and B_ij might be missing.
 * Note also that A_ij and B_ij are mnp x cnp and mnp x pnp matrices resp. and
 * should be stored in jac in row-major order.
 * rcidxs, rcsubs are max(m, n) x 1, allocated by the caller and can be used
 * as working memory
 */
void sba_pinhole_model_jac(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_BUNDLEADJUST_H_
