#include "lib/slam/pose.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <boost/test/unit_test.hpp>
#include <cstdlib>

#include "lib/hub/utility.h"
#include "lib/hub/time.h"

#define PRECISION float
// #define PRECISION double

const PRECISION pose_point_array[] = {
	0.0, 0.0, 10.0,
	-20.0, -25.0, 20.0,
	-25.0, 35.0, 30.0,
	22.0, 17.0, 40.0,
	10.0, -20.0, 40.0,
	10.0, -20.0, 50.0,
	-15.0, 6.0, 60.0,
	5.0, -28.0, 80.0
};
const size_t num_points = sizeof(pose_point_array)/sizeof(PRECISION)/3;

static const float epsilon = 0.0001;
#define CHECK_ARRAYS(expected_array, observed_array, size, margin) \
	for(unsigned int i=0; i<size; i++) { \
		if(std::fabs(expected_array[i]) < epsilon) \
			BOOST_CHECK_SMALL(observed_array[i], epsilon); \
		else \
			BOOST_CHECK_CLOSE(expected_array[i], observed_array[i], margin); \
	}

using namespace hub;
using namespace hub::slam;

PRECISION estimation_error(const std::vector<PRECISION> &parameters,
		std::vector<PRECISION> estimated_parameters) {

	PRECISION sq_error = 0;
	for(unsigned int i=0; i<std::min(parameters.size(), estimated_parameters.size()); i++) {
		const PRECISION e = parameters[i] - estimated_parameters[i];
		sq_error += e*e;
	}
	return sqrt(sq_error);
}

BOOST_AUTO_TEST_SUITE(hub_pose_tests)

BOOST_AUTO_TEST_CASE(jacobian_test) {
	PRECISION euler_pose[6] = {0.1, 0.2, 0.3, 1.0, -1.0, 0.5};
	PRECISION levmar_jac[num_points*2*6];
	std::vector<PRECISION> objectpoints;
	for(unsigned int i=0; i<num_points; i++) {
		objectpoints.push_back(pose_point_array[i*3]);
		objectpoints.push_back(pose_point_array[i*3+1]);
		objectpoints.push_back(pose_point_array[i*3+2]);
	}
	std::vector< cv::Point_<PRECISION> > imagepoints; // projections aren't needed for jacobians => let them empty
	pinhole_model_data_t<PRECISION> model_data(objectpoints, imagepoints);

	//
	// Test euler implementations
	//
	levmar_ideal_pinhole_euler_jac(euler_pose, levmar_jac, 6, 2*num_points, (void*)&model_data);

	PRECISION cam_jac[2][6];
	for(unsigned int point_idx=0; point_idx<num_points; point_idx++) {
		ideal_pinhole_model_euler_jac<PRECISION>(&pose_point_array[point_idx*3], euler_pose, cam_jac);

		CHECK_ARRAYS( (&levmar_jac[point_idx*6]), cam_jac[0], 6, 0.05);
		CHECK_ARRAYS( (&levmar_jac[(num_points+point_idx)*6]), cam_jac[1], 6, 0.05);
	}
/*	// test approximation
	levmar_approx_ideal_pinhole_euler_jac(euler_pose, levmar_jac, 6, 2*num_points, (void*)&model_data);
	for(unsigned int point_idx=0; point_idx<num_points; point_idx++) {
		ideal_pinhole_model_euler_jac<PRECISION>(&pose_point_array[point_idx*3], euler_pose, cam_jac);

		CHECK_ARRAYS( (&levmar_jac[point_idx*6]), cam_jac[0], 6, 100);
		CHECK_ARRAYS( (&levmar_jac[(num_points+point_idx)*6]), cam_jac[1], 6, 100);
	}*/

	//
	// Test quaternion vector implementations
	//
	PRECISION quaternion[4];
	euler_to_quaternion(euler_pose, quaternion);
	PRECISION quatvec_pose[6];
	for(unsigned int i=0; i<3; i++) {
		quatvec_pose[i] = quaternion[i+1];
		quatvec_pose[i+3] = euler_pose[i+3];
	}
	levmar_ideal_pinhole_quatvec_jac(quatvec_pose, levmar_jac, 6, 2*num_points, (void*)&model_data);

	for(unsigned int point_idx=0; point_idx<num_points; point_idx++) {
		ideal_pinhole_model_quatvec_jac<PRECISION>(&pose_point_array[point_idx*3], quatvec_pose, cam_jac);

		CHECK_ARRAYS( (&levmar_jac[point_idx*6]), cam_jac[0], 6, 0.05);
		CHECK_ARRAYS( (&levmar_jac[(num_points+point_idx)*6]), cam_jac[1], 6, 0.05);
	}
}

BOOST_AUTO_TEST_CASE(estimate_translation_test) {
	std::vector< cv::Point3_<PRECISION> > objectpoints;
	for(unsigned int i=0; i<num_points; i++) {
		objectpoints.push_back( cv::Point3_<PRECISION>(pose_point_array[i*3], pose_point_array[i*3+1], pose_point_array[i*3+2]) );
	}
	BOOST_CHECK_EQUAL(num_points, objectpoints.size());

	// set reference points
	std::vector<cv::Point2f> src_ideal_points(2*num_points);
	PRECISION rt[6] = { 0 };
	ideal_pinhole_model<PRECISION, rotation_matrix_quatvec>(pose_point_array,
		rt,
		&src_ideal_points[0].x,
		num_points);

	// apply translation
	std::vector<cv::Point2f> dst_ideal_points(2*num_points);
	rt[3] = 5.0;
	rt[4] = 25.0;
	rt[5] = 10.0;
	ideal_pinhole_model<PRECISION, rotation_matrix_quatvec>(pose_point_array,
		rt,
		&dst_ideal_points[0].x,
		num_points);

	//TODO apply some error
	std::vector<cv::DMatch> matches;
	matches.reserve(num_points);
	for(unsigned int index = 0; index < num_points; index++) {
		matches.push_back( cv::DMatch(index, index, 0) );
	}

	PRECISION avg_depth = mean<PRECISION, 3>(pose_point_array, 3*num_points);
	PRECISION transl_estimation[3] = { 0 };

	// estimate translation by objects
	estimate_translation_by_objects(objectpoints,
		dst_ideal_points,
		avg_depth + rt[5],
		matches,
		transl_estimation);
	CHECK_ARRAYS( (rt+3), transl_estimation, 3, 12)

	// estimate translation by features
	estimate_translation_by_features(src_ideal_points,
		dst_ideal_points,
		avg_depth,
		avg_depth + rt[5],
		matches,
		transl_estimation);
	CHECK_ARRAYS( (rt+3), transl_estimation, 3, 13)

}

BOOST_AUTO_TEST_CASE(guess_euler_pose_test) {

	std::vector< cv::Point3_<PRECISION> > objectpoints;
	for(unsigned int i=0; i<num_points; i++) {
		objectpoints.push_back( cv::Point3_<PRECISION>(pose_point_array[i*3], pose_point_array[i*3+1], pose_point_array[i*3+2]) );
	}
	BOOST_CHECK_EQUAL(num_points, objectpoints.size());

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);

	std::vector<PRECISION> euler_pose(6);
	euler_pose[0] = 0.1;	//phi
	euler_pose[1] = 0.2;	//theta
	euler_pose[2] = 0.3;	//psi
	euler_pose[3] = 1.0;	//t1
	euler_pose[4] = -1.0;	//t2
	euler_pose[5] = 0.5;	//t3

	std::vector< cv::Point_<PRECISION> > imagepoints( objectpoints.size() );
	pinhole_model_euler(
		reinterpret_cast<PRECISION*>( &(objectpoints[0].x) ),
		&euler_pose[0],
		camera_matrix,
		reinterpret_cast<PRECISION*>( &(imagepoints[0].x) ),
		objectpoints.size() );
// 	objectpoints_to_imagepoints(objectpoints,
// 		euler_pose,
// 		camera_matrix,
// 		imagepoints);

	//apply some error to image points
	imagepoints[0].x = 0;

	std::vector<cv::DMatch> matches;
	for(unsigned int index = 0; index < objectpoints.size(); index++) {
		matches.push_back( cv::DMatch(index, index, 0) );
	}

	std::vector<PRECISION> pose_guess(6, 0);
	std::vector<char> matches_mask;
	PRECISION info[LM_INFO_SZ];

	uint64_t start_time = get_time_us();
	int rc = lm_pose_optimization<PRECISION, levmar_pinhole_euler<PRECISION>, levmar_pinhole_euler_jac<PRECISION> >(
		objectpoints,
		imagepoints,
		matches,
		pose_guess,
		camera_matrix,
		matches_mask,
		100,
		info);
	uint64_t stop_time = get_time_us();
	BOOST_CHECK(rc >= 0);
	BOOST_TEST_MESSAGE("levmar returned " << rc
		<< " in " << info[5]
		<< " iterations, reason " << info[6]
		<< ", error " << info[1] / objectpoints.size()
		<< " [initial " << info[0] / objectpoints.size()
		<< "], " << (int)info[7] << "/" << (int)info[8]
		<< " func/fjac evals, " << (int)info[9] << " lin. systems.");
	BOOST_TEST_MESSAGE("Elapsed time: " << (stop_time-start_time) << "us");

	BOOST_TEST_MESSAGE("_Pose Estimation_ (" << rc << ")" );
	BOOST_TEST_MESSAGE("phi:   " << pose_guess[0] << " (" << euler_pose[0] << ")" );
	BOOST_TEST_MESSAGE("theta: " << pose_guess[1] << " (" << euler_pose[1] << ")" );
	BOOST_TEST_MESSAGE("psi:   " << pose_guess[2] << " (" << euler_pose[2] << ")" );
	BOOST_TEST_MESSAGE("dx:    " << pose_guess[3] << " (" << euler_pose[3] << ")" );
	BOOST_TEST_MESSAGE("dy:    " << pose_guess[4] << " (" << euler_pose[4] << ")" );
	BOOST_TEST_MESSAGE("dz:    " << pose_guess[5] << " (" << euler_pose[5] << ")" );
	PRECISION estim_err = estimation_error(pose_guess, euler_pose);
	BOOST_TEST_MESSAGE("error: " <<  estim_err);
// 	BOOST_CHECK(estim_err <= 0.1);

	//test empty image points
	rc = lm_pose_optimization<PRECISION, levmar_pinhole_euler<PRECISION>, levmar_pinhole_euler_jac<PRECISION> >(
		objectpoints,
		std::vector< cv::Point_<PRECISION> >(),
		matches,
		pose_guess,
		camera_matrix,
		matches_mask,
		20);
	BOOST_CHECK(rc < 0);

	//test empty object points
	rc = lm_pose_optimization<PRECISION, levmar_pinhole_euler<PRECISION>, levmar_pinhole_euler_jac<PRECISION> >(
		std::vector< cv::Point3_<PRECISION> >(),
		imagepoints,
		matches,
		pose_guess,
		camera_matrix,
		matches_mask,
		20);
	BOOST_CHECK(rc < 0);

	//test empty matches
	rc = lm_pose_optimization<PRECISION, levmar_pinhole_euler<PRECISION>, levmar_pinhole_euler_jac<PRECISION> >(
		objectpoints,
		imagepoints,
		std::vector<cv::DMatch>(),
		pose_guess,
		camera_matrix,
		matches_mask,
		20);
	BOOST_CHECK(rc == 0);
}

BOOST_AUTO_TEST_CASE(guess_quaternion_pose_test) {
	std::vector< cv::Point3_<PRECISION> > objectpoints;
	for(unsigned int i=0; i<num_points; i++) {
		objectpoints.push_back( cv::Point3_<PRECISION>(pose_point_array[i*3], pose_point_array[i*3+1], pose_point_array[i*3+2]) );
	}

	PRECISION euler_angles[3] = {0.1, 0.2, 0.3};
	PRECISION quaternion[4];
	euler_to_quaternion(euler_angles, quaternion);
	std::vector<PRECISION> quatvec_pose(6);
	quatvec_pose[0] = quaternion[1];
	quatvec_pose[1] = quaternion[2];
	quatvec_pose[2] = quaternion[3];
	quatvec_pose[3] = 1.0;	//t1
	quatvec_pose[4] = -1.0;	//t2
	quatvec_pose[5] = 0.5;	//t3

	std::vector< cv::Point_<PRECISION> > idealpoints(num_points);
	ideal_pinhole_model<PRECISION, rotation_matrix_quatvec>(&objectpoints[0].x,
		&quatvec_pose[0],
		&idealpoints[0].x,
		num_points);

	//apply some error to image points
	idealpoints[0].x = 0;

	std::vector<cv::DMatch> matches;
	for(unsigned int index = 0; index < objectpoints.size(); index++) {
		matches.push_back( cv::DMatch(index, index, 0) );
	}

	std::vector<PRECISION> pose_guess(6, 0);
	std::vector<char> matches_mask;
	PRECISION info[LM_INFO_SZ];

	uint64_t start_time = get_time_us();
	int rc = lm_pose_optimization<PRECISION, levmar_ideal_pinhole_quatvec<PRECISION>, levmar_ideal_pinhole_quatvec_jac<PRECISION> >(
		objectpoints,
		idealpoints,
		matches,
		pose_guess,
		cv::Mat(),
		matches_mask,
		100,
		info);
	uint64_t stop_time = get_time_us();
	BOOST_CHECK(rc >= 0);
	BOOST_TEST_MESSAGE("levmar returned " << rc
		<< " in " << info[5]
		<< " iterations, reason " << info[6]
		<< ", error " << info[1] / objectpoints.size()
		<< " [initial " << info[0] / objectpoints.size()
		<< "], " << (int)info[7] << "/" << (int)info[8]
		<< " func/fjac evals, " << (int)info[9] << " lin. systems.");
	BOOST_TEST_MESSAGE("Elapsed time: " << (stop_time-start_time) << "us");

	BOOST_TEST_MESSAGE("_Pose Estimation_ (" << rc << ")" );
	BOOST_TEST_MESSAGE("q1:    " << pose_guess[0] << " (" << quatvec_pose[0] << ")" );
	BOOST_TEST_MESSAGE("q2:    " << pose_guess[1] << " (" << quatvec_pose[1] << ")" );
	BOOST_TEST_MESSAGE("q3:    " << pose_guess[2] << " (" << quatvec_pose[2] << ")" );
	BOOST_TEST_MESSAGE("dx:    " << pose_guess[3] << " (" << quatvec_pose[3] << ")" );
	BOOST_TEST_MESSAGE("dy:    " << pose_guess[4] << " (" << quatvec_pose[4] << ")" );
	BOOST_TEST_MESSAGE("dz:    " << pose_guess[5] << " (" << quatvec_pose[5] << ")" );
	PRECISION estim_err = estimation_error(pose_guess, quatvec_pose);
	BOOST_TEST_MESSAGE("error: " <<  estim_err);

/*
	// translation only
	quatvec_pose[0] = 0;
	quatvec_pose[1] = 0;
	quatvec_pose[2] = 0;
	quatvec_pose[3] = 25.0;	//t1
	quatvec_pose[4] = -13.0;	//t2
	quatvec_pose[5] = 7;	//t3

	ideal_pinhole_model_quatvec(&objectpoints[0].x,
		&quatvec_pose[0],
		&idealpoints[0].x,
		num_points);

	pose_guess = std::vector<PRECISION>(6, 0);
	matches_mask = std::vector<char>( matches.size(), 1);
	cv::Point3_<PRECISION> translation = estimate_translation_by_objects<PRECISION>(objectpoints,
		idealpoints,
		41.25,
		matches,
		matches_mask);
	pose_guess[3] = translation.x;
	pose_guess[4] = translation.y;
	pose_guess[5] = translation.z;
	rc = lm_pose_optimization<PRECISION, levmar_ideal_pinhole_quatvec<PRECISION>, levmar_ideal_pinhole_quatvec_jac<PRECISION> >(
// 	rc = lm_pose_optimization<PRECISION, levmar_ideal_pinhole_euler<PRECISION>, levmar_ideal_pinhole_euler_jac<PRECISION> >(
// 	rc = lm_pose_optimization<PRECISION, levmar_ideal_pinhole_quatvec<PRECISION>, levmar_ideal_pinhole_model_jac<PRECISION,new_ideal_pinhole_model_quatvec_jac> >(
		objectpoints,
		idealpoints,
		matches,
		pose_guess,
		matches_mask,
		100,
		info);
	BOOST_TEST_MESSAGE("_Pose Estimation_ (" << rc << ")" );
	BOOST_TEST_MESSAGE("q1:    " << pose_guess[0] << " (" << quatvec_pose[0] << ")" );
	BOOST_TEST_MESSAGE("q2:    " << pose_guess[1] << " (" << quatvec_pose[1] << ")" );
	BOOST_TEST_MESSAGE("q3:    " << pose_guess[2] << " (" << quatvec_pose[2] << ")" );
	BOOST_TEST_MESSAGE("dx:    " << pose_guess[3] << " (" << quatvec_pose[3] << ")" );
	BOOST_TEST_MESSAGE("dy:    " << pose_guess[4] << " (" << quatvec_pose[4] << ")" );
	BOOST_TEST_MESSAGE("dz:    " << pose_guess[5] << " (" << quatvec_pose[5] << ")" );
	BOOST_TEST_MESSAGE("error: " << estimation_error(pose_guess, quatvec_pose) );
*/
}

BOOST_AUTO_TEST_SUITE_END()

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
