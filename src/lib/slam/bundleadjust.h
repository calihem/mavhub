#ifndef _HUB_BUNDLEADJUST_H_
#define _HUB_BUNDLEADJUST_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#if CV_MINOR_VERSION >= 2

#include <sba/sba.h>
#include "lib/hub/math.h"

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
	sba_model_data_t(const cv::Mat &camera_matrix) :
		camera_matrix(camera_matrix) {};

	const cv::Mat &camera_matrix;			/// intrinsic camera parameters
};

/**
 * functional relation describing measurements. Given a parameter vector p,
 * computes a prediction of the measurements \hat{x}. p is (m*cnp + n*pnp)x1,
 * \hat{x} is (n*m*mnp)x1, maximum
 * rcidxs, rcsubs are max(m, n) x 1, allocated by the caller and can be used
 * as working memory
 */
void sba_ideal_pinhole_model(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);

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
void sba_ideal_pinhole_model_jac(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata);

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

/**
 * \brief Bundle adjustment on camera and structure parameters using the sparse Levenberg-Marquardt as described in HZ p. 568
 *
 * This function is a copy of sba_motstr_levmar_x from the sba library and extended by a weighting function for the residuals. 
 *
 * Returns the number of iterations (>=0) if successfull, SBA_ERROR if failed
 * \param n number of points
 * \param ncon number of points (starting from the 1st) whose parameters should not be modified.
 * 	All B_ij (see below) with i< ncon are assumed to be zero
 * \param m number of images
 * \param mcon number of images (starting from the 1st) whose parameters should not be modified.
 * 	All A_ij (see below) with j< mcon are assumed to be zero
 * \param vmask visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise. nxm
 * \param p initial parameter vector p0: (a1, ..., am, b1, ..., bn).
 * 	aj are the image j parameters, bi are the i-th point parameters,
 * 	size m*cnp + n*pnp
 * \param cnp number of parameters for ONE camera; e.g. 6 for Euclidean cameras
 * \param pnp number of parameters for ONE point; e.g. 3 for Euclidean points
 * \param x measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
 * 	x_ij is the projection of the i-th point on the j-th image.
 * 	NOTE: some of the x_ij might be missing, if point i is not visible in image j;
 * 	see vmask[i, j], max. size n*m*mnp
 * \param covx measurements covariance matrices: (Sigma_x_11, .. Sigma_x_1m, ..., Sigma_x_n1, .. Sigma_x_nm),
 * 	where Sigma_x_ij is the mnp x mnp covariance of x_ij stored row-by-row. Set to NULL if no
 * 	covariance estimates are available (identity matrices are implicitly used in this case).
 *	 NOTE: a certain Sigma_x_ij is missing if the corresponding x_ij is also missing;
 * 	see vmask[i, j], max. size n*m*mnp*mnp
 * \param mnp number of parameters for EACH measurement; usually 2
 * \param func functional relation describing measurements. Given a parameter vector p,
 * 	computes a prediction of the measurements \hat{x}. p is (m*cnp + n*pnp)x1,
 * 	\hat{x} is (n*m*mnp)x1, maximum
 * 	rcidxs, rcsubs are max(m, n) x 1, allocated by the caller and can be used
 * 	as working memory
 * \param fjac function to evaluate the sparse jacobian dX/dp.
 * 	The Jacobian is returned in jac as
 * 	(dx_11/da_1, ..., dx_1m/da_m, ..., dx_n1/da_1, ..., dx_nm/da_m,
 * 	dx_11/db_1, ..., dx_1m/db_1, ..., dx_n1/db_n, ..., dx_nm/db_n), or (using HZ's notation),
 * 	jac=(A_11, B_11, ..., A_1m, B_1m, ..., A_n1, B_n1, ..., A_nm, B_nm)
 * 	Notice that depending on idxij, some of the A_ij and B_ij might be missing.
 * 	Note also that A_ij and B_ij are mnp x cnp and mnp x pnp matrices resp. and they
 * 	should be stored in jac in row-major order.
 * 	rcidxs, rcsubs are max(m, n) x 1, allocated by the caller and can be used
 * 	as working memory
 *
 * 	If NULL, the jacobian is approximated by repetitive func calls and finite
 * 	differences. This is computationally inefficient and thus NOT recommended.
 * \param fweight function to weight residuals x-\hat{x}
 * 	Set to NULL for w(x-\hat{x}) with w = 1.
 * \param adata pointer to possibly additional data, passed uninterpreted to func, fjac
 * \param itmax I: maximum number of iterations. itmax==0 signals jacobian verification followed by immediate return
 * \param verbose I: verbosity
 * \param opts[SBA_OPTSSZ] I: minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \epsilon4]. Respectively the scale factor for initial \mu,
 * 	stopping thresholds for ||J^T e||_inf, ||dp||_2, ||e||_2 and (||e||_2-||e_new||_2)/||e||_2
 * \param info[SBA_INFOSZ] O: information regarding the minimization. Set to NULL if don't care
 * 	info[0]=||e||_2 at initial p.
 * 	info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||dp||_2, mu/max[J^T J]_ii ], all computed at estimated p.
 * 	info[5]= # iterations,
 * 	info[6]=reason for terminating: 1 - stopped by small gradient J^T e
 * 	                                2 - stopped by small dp
 * 	                                3 - stopped by itmax
 * 	                                4 - stopped by small relative reduction in ||e||_2
 * 	                                5 - stopped by small ||e||_2
 * 	                                6 - too many attempts to increase damping. Restart with increased mu
 * 	                                7 - stopped by invalid (i.e. NaN or Inf) "func" values. This is a user error
 * 	info[7]= # function evaluations
 * 	info[8]= # jacobian evaluations
 * 	info[9]= # number of linear systems solved, i.e. number of attempts	for reducing error
 */
int sba_motstr_levmar_w(
		const int n,
		const int ncon,
		const int m,
		const int mcon,
		char *vmask,
		double *p,
		const int cnp,
		const int pnp,
		double *x,
		double *covx,
		const int mnp,
		void (*func)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata),
		void (*fjac)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata),
		void (*fweight)(double *r, const unsigned int n),
		void *adata,
		const int itmax,
		const int verbose,
		const double opts[SBA_OPTSSZ],
		double info[SBA_INFOSZ]);

template<typename T>
void sba_tukey_estimator(T *r, const unsigned int n) {
	const int n_half = n >> 1;
	std::vector<T> u_residuals(n_half);
	std::vector<T> v_residuals(n_half);

	for(unsigned int i=0; i<n; i+=2) {
		const unsigned int index = i >> 1; // index = i/2
		u_residuals[index] = r[i];
		v_residuals[index] = r[i+1];
	}

	// apply M-estimator
	const T u_sigma = _robust_sigma<T>(u_residuals);
	const T v_sigma = _robust_sigma<T>(v_residuals);
	for(unsigned int i=0; i<n; i+=2) {
		r[i] *= tukey_weight<T>(r[i], u_sigma);
		r[i+1] *= tukey_weight<T>(r[i+1], v_sigma);
	}
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_BUNDLEADJUST_H_
