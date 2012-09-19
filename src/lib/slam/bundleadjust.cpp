#include "bundleadjust.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

namespace hub {
namespace slam {

/// transforms 3-dimensional quaternion vector to 4-dimension unit quaternion by adding scalar part
inline static void vec2quat(const double v[3], double q[quaternion_size]) {
	q[1] = v[0];
	q[2] = v[1];
	q[3] = v[2];
	q[0] = sqrt(1.0 - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]); 
}

template <typename T>
inline T* multiply_quaternion(const T lhs[quaternion_size], const T rhs[quaternion_size], T product[quaternion_size]) {
	
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

/**
 * \brief Calculates the projection of a 3D point to a 2D image point.
 * \param[in] object_point 3D object point
 * \param[in] q rotation quaternion
 * \param[in] t translation vector
 * \param[in] camera_matrix intrinsic params
 * \param[out] image_point 2D projection of M
 */
template<typename T>
void quaternion_pinhole_model(const T object_point[3],
		const T q[4],
		const T t[3],
		const cv::Mat &camera_matrix,
		T image_point[2]) {
	
	const T q0 = q[0];
	const T q1 = q[1];
	const T q2 = q[2];
	const T q3 = q[3];

	const T p0 = object_point[0];
	const T p1 = object_point[1];
	const T p2 = object_point[2];

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const T t1 = -p0*q1-q2*p1-q3*p2;
	const T t2 = q0*p0+q2*p2-q3*p1;
	const T t3 = p1*q0+q3*p0-p2*q1;
	const T t4 = q0*p2+p1*q1-q2*p0;

	const T x = -q1*t1+q0*t2-t3*q3+q2*t4+t[0];
	const T y = -q2*t1+q0*t3-t4*q1+q3*t2+t[1];
	const T z = -t1*q3+q0*t4-q2*t2+q1*t3+t[2];

	image_point[0] = (fx*x)/z + cx;
	image_point[1] = (fy*y)/z + cy;
}

/**
 * \brief Calculates the jacobian of the pinhole projection.
 * \param[in] object_point 3D object point
 * \param[in] qr rotation quaternion vector part
 * \param[in] t translation vector
 * \param[in] camera_matrix intrinsic params
 * \param[in] qr0 quaternion rotation estimation
 * \param[out] jacmRT
 * \param[out] jacmS
 */
template<typename T>
void quaternion_pinhole_model_jac(const T object_point[3],
		const T qr[3],
		const T t[3],
		const cv::Mat &camera_matrix,
		const T qr0[4],
		T jacmRT[2][6],
		T jacmS[2][3]) {

	const T p0 = object_point[0];
	const T p1 = object_point[1];
	const T p2 = object_point[2];

	const T q1 = qr[0];
	const T q2 = qr[1];
	const T q3 = qr[2];
	const T q0 = sqrt(1.0-q1*q1-q2*q2-q3*q3);

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const T g0 = qr0[0];
	const T g1 = qr0[1];
	const T g2 = qr0[2];
	const T g3 = qr0[3];

	const T t10 = 1/q0;
	const T t12 = g1*t10;
	const T t15 = -t12*q1+g0;
	const T t19 = g2*t10;
	const T t22 = -t19*q1-g3;
	const T t25 = t10*g3;
	const T t27 = -t25*q1+g2;
	const T t30 = -t15*p0-t22*p1-t27*p2;
	const T t35 = -q0*g1-q1*g0-q2*g3+q3*g2;
	const T t43 = q0*g2+q2*g0+q3*g1-q1*g3;
	const T t49 = q0*g3+q3*g0+q1*g2-g1*q2;
	const T t51 = t35*p0-t43*p1-t49*p2;
	const T t54 = t10*g0;
	const T t56 = -t54*q1-g1;
	const T t61 = q0*g0-q1*g1-q2*g2-q3*g3;
	const T t65 = t61*p0+t43*p2-p1*t49;
	const T t70 = t56*p0+t22*p2-p1*t27;
	const T t75 = t56*p1+t27*p0-p2*t15;
	const T t81 = t61*p1+t49*p0+t35*p2;
	const T t87 = t56*p2+p1*t15-t22*p0;
	const T t93 = t61*p2-t35*p1-t43*p0;
	const T t107 = -t30*t43-t22*t51+t56*t81+t61*t75+t87*t35-t93*t15+t70*t49+t27*t65;
	const T t118 = -t30*t49-t27*t51+t56*t93+t61*t87-t70*t43-t65*t22-t35*t75+t81*t15;
	const T t126 = -t49*t51+t61*t93-t65*t43-t81*t35+t[2];
	const T t127 = 1/t126;
	const T t141 = -t51*t43+t61*t81+t93*t35+t65*t49+t[1];
	const T t145 = t126*t126;
	const T t146 = 1/t145;
	const T t147 = (fx*(t35*t51+t61*t65-t81*t49+t93*t43+t[0])+t126*cx)*t146;
	jacmRT[0][0] = (fx*(t30*t35-t15*t51+t56*t65+t61*t70-t49*t75-t81*t27+t43*t87+t93*t22)+cx*t118)*t127-t118*t147;
	const T t159 = (fy*t141+t126*cy)*t146;
	jacmRT[1][0] = (t107*fy+cy*t118)*t127-t159*t118;
	const T t162 = -t12*q2+g3;
	const T t165 = -t19*q2+g0;
	const T t168 = -t25*q2-g1;
	const T t170 = -t162*p0-t165*p1-t168*p2;
	const T t175 = -t54*q2-g2;
	const T t180 = t175*p0+t165*p2-t168*p1;
	const T t185 = t175*p1+t168*p0-t162*p2;
	const T t192 = t175*p2+t162*p1-t165*p0;
	const T t206 = -t170*t43-t51*t165+t175*t81+t61*t185+t192*t35-t93*t162+t49*t180+t65*t168;
	const T t216 = -t170*t49-t51*t168+t93*t175+t61*t192-t180*t43-t65*t165-t185*t35+t81*t162;
	jacmRT[0][1] = (fx*(t170*t35-t162*t51+t175*t65+t180*t61-t185*t49-t81*t168+t192*t43+t93*t165)+cx*t216)*t127-t147*t216;
	jacmRT[1][1] = (fy*t206+cy*t216)*t127-t159*t216;
	const T t227 = -t12*q3-g2;
	const T t230 = -t19*q3+g1;
	const T t233 = -t25*q3+g0;
	const T t235 = -t227*p0-p1*t230-t233*p2;
	const T t240 = -t54*q3-g3;
	const T t245 = t240*p0+t230*p2-t233*p1;
	const T t250 = p1*t240+t233*p0-t227*p2;
	const T t257 = t240*p2+t227*p1-t230*p0;
	const T t271 = -t235*t43-t51*t230+t81*t240+t61*t250+t257*t35-t93*t227+t245*t49+t65*t233;
	const T t281 = -t235*t49-t51*t233+t240*t93+t61*t257-t245*t43-t230*t65-t250*t35+t81*t227;
	jacmRT[0][2] = (fx*(t235*t35-t227*t51+t240*t65+t61*t245-t250*t49-t81*t233+t257*t43+t93*t230)+t281*cx)*t127-t147*t281;
	jacmRT[1][2] = (fy*t271+t281*cy)*t127-t159*t281;
	jacmRT[0][3] = t127*fx;
	jacmRT[1][3] = 0.0;
	jacmRT[0][4] = 0.0;
	jacmRT[1][4] = fy*t127;
	jacmRT[0][5] = cx*t127-t147;
	jacmRT[1][5] = cy*t127-t159;
	const T t293 = t35*t35;
	const T t294 = t61*t61;
	const T t296 = t43*t43;
	const T t299 = -t35*t43;
	const T t302 = -t61*t49;
	const T t303 = 2.0*t299+t61*t49-t302;
	const T t305 = -t35*t49;
	const T t306 = -t61*t43;
	const T t309 = t305+2.0*t306-t49*t35;
	jacmS[0][0] = (fx*(t293+t294-t49*t49-t296)+cx*t309)*t127-t147*t309;
	jacmS[1][0] = (fy*t303+cy*t309)*t127-t159*t309;
	const T t324 = t49*t49;
	const T t325 = t296+t294-t35*t35-t324;
	const T t327 = t49*t43;
	const T t330 = t61*t35;
	const T t331 = 2.0*t327-t61*t35-t330;
	jacmS[0][1] = (fx*(t299+2.0*t302-t35*t43)+cx*t331)*t127-t147*t331;
	jacmS[1][1] = (fy*t325+cy*t331)*t127-t159*t331;
	const T t347 = t327+2.0*t330+t43*t49;
	const T t350 = t324+t294-t43*t43-t293;
	jacmS[0][2] = (fx*(2.0*t305+t61*t43-t306)+t350*cx)*t127-t147*t350;
	jacmS[1][2] = (fy*t347+cy*t350)*t127-t159*t350;
}

void sba_pinhole_model(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata) {
	const sba_model_data_t<double> *sba_data = static_cast< sba_model_data_t<double>* >(adata);
	if(!sba_data) return;

	const cv::Mat &camera_matrix = sba_data->camera_matrix;
	
	const int num_cols = idxij->nc;
	double *p_pose = p;
	double *p_points = p+num_cols*num_params_per_cam;

	double rotation_quat[quaternion_size];
	double total_rot_quat[quaternion_size];

	for(int j=0; j<num_cols; ++j) {
		// get j-th camera parameters
		double *p_quat = p_pose + j*num_params_per_cam;
		double *p_transl = p_quat + 3;	// rotation vector part has 3 elements
		// get rotation estimation
		vec2quat(p_quat, rotation_quat);
		const double *p_rotation_guess = &(sba_data->initial_rotations[j*quaternion_size]);
		multiply_quaternion(rotation_quat, p_rotation_guess, total_rot_quat);

		// find number of nonzero hx_ij, i=0...n-1 
		int num_nonzero = sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); 
		for(int i=0; i<num_nonzero; ++i) {
			double *p_point = p_points + rcsubs[i]*num_params_per_point;
			double *p_measurement = hx + idxij->val[rcidxs[i]]*num_params_per_measuremnt; // p_measurement = hx_ij
			
			quaternion_pinhole_model<double>(p_point, total_rot_quat, p_transl, camera_matrix, p_measurement);
		}
	}
}

void sba_pinhole_model_jac(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata) {
	const sba_model_data_t<double> *sba_data = static_cast< sba_model_data_t<double>* >(adata);
	if(!sba_data) return;

	const cv::Mat &camera_matrix = sba_data->camera_matrix;

	const int num_cols = idxij->nc;
	double *p_pose = p;
	double *p_points = p+num_cols*num_params_per_cam;

	static const int size_A = num_params_per_measuremnt*num_params_per_cam;
	static const int size_B = num_params_per_measuremnt*num_params_per_point;
	static const int size_AB = size_A+size_B;
	
	for(int j=0; j<num_cols; ++j) {
		// get j-th camera parameters
		double *p_quat = p_pose + j*num_params_per_cam;
		double *p_transl = p_quat + 3;	// rotation vector part has 3 elements
		const double *p_rotation_guess = &(sba_data->initial_rotations[j*quaternion_size]);

		// find number of nonzero hx_ij, i=0...n-1 
		int num_nonzero = sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); 
		for(int i=0; i<num_nonzero; ++i) {
			double *p_point = p_points + rcsubs[i]*num_params_per_point;
			double *p_Aij = jac + idxij->val[rcidxs[i]]*size_AB;
			double *p_Bij = p_Aij + size_A;

			quaternion_pinhole_model_jac<double>(p_point,
				p_quat,
				p_transl,
				camera_matrix,
				p_rotation_guess,
				(double (*)[6])p_Aij,
				(double (*)[3])p_Bij);
		}
	}
}

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
