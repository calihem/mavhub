#include "tracker.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include "lib/slam/pose.h"

#define VERBOSE 0

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout << typeid(*this).name() << ": "
#else
#define dout if(0) std::cout
#endif

#include <typeinfo>

namespace hub {
namespace slam {

uint16_t Tracker::run_counter = 0;

Tracker::Tracker(const int image_width,
	const int image_height,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients) :
		camera_matrix(camera_matrix),
		distortion_coefficients(distortion_coefficients),
		feature_detector(60, 3), //threshold, octaves
		descriptor_extractor(),
		matcher(),
		last_imu_time(0),
		speed(3, 0.0),
		pose(6, 0.0) {

	if(Tracker::camera_matrix.rows < 3 || Tracker::camera_matrix.cols < 3)
		Tracker::camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	if(Tracker::distortion_coefficients.total() < 5)
		Tracker::distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);

	map.set_camera(Tracker::camera_matrix, image_width, image_height);
}

Tracker::~Tracker() {
	map.join();
}

int Tracker::imu_update(const uint64_t &time_ms, const std::vector<float> &imu_data) {

	// update attitude
	pose[0] = imu_data[0];
	pose[1] = imu_data[1];
	pose[2] = imu_data[2];

	float delta_time = (float)(time_ms - last_imu_time)/1000.0;
	if(delta_time <= 0.001 || delta_time > 1.0) {// time in a implausible range
		dout << "delta_time of " << time_ms << " - " << last_imu_time << " =  " << delta_time << " seems to be implausible" << std::endl; 
		//reset speed
		speed[0] = speed[1] = speed[2] = 0.0;
		goto finish;
	}

	// convert body to inertial frame
	float accels_inertial[3];
	float rotation_matrix[9];
	rotation_matrix_quatvec(&imu_data[0], rotation_matrix);
	multiply(rotation_matrix, &imu_data[3], accels_inertial);
	accels_inertial[2] -= 9.81; // remove gravity

	// update position
	pose[3] += delta_time * speed[0];
	pose[4] += delta_time * speed[1];
	pose[5] += delta_time * speed[2];

	// update speed
	speed[0] += delta_time * accels_inertial[0];
	speed[1] += delta_time * accels_inertial[1];
	speed[2] += delta_time * accels_inertial[2];

finish:
	last_imu_time = time_ms;

	return 0;
}

int Tracker::log_debug_data(const std::bitset<8> &debug_mask,
                            const std::vector<float> &parameter_vector) {
	static bool first_run = true;

	if( debug_mask.test(POSE) ) { // log pose estimations
		static const char *pose_filename = "tracker_pose_estimation.m";
		std::ofstream pose_stream;

		if(first_run) // create new file
			pose_stream.open(pose_filename);
		else  // append to existing data
			pose_stream.open(pose_filename, std::ios_base::app);
		if( !pose_stream.is_open() )
			return - 119;

		if(first_run) {// add header
			pose_stream << "%% pose estimations" << std::endl
				<< "%% q1 q2 q3 x_transl y_transl z_transl (body frame)" << std::endl
				<< "%%" << std::endl
				<< "%% add the following line(s) (without %) to the end of the file" << std::endl
				<< "%];" << std::endl
				<< "%pose = reshape(pose, 6, [])';" << std::endl;

			pose_stream << "pose = [";
		} else
			pose_stream << ", ";

		pose_stream << parameter_vector[0] << ", "
			<< parameter_vector[1] << ", "
			<< parameter_vector[2] << ", "
			<< parameter_vector[3] << ", "
			<< parameter_vector[4] << ", "
			<< parameter_vector[5];
		pose_stream.close();
		first_run =false;
	}
	
	return 0;
}

int Tracker::log_debug_data(const std::bitset<8> &debug_mask,
                            const cv::Mat &image,
                            const std::vector<float> &/*parameter_vector*/,
                            const std::vector<cv::KeyPoint> &keypoints) {

	if( debug_mask.test(FEATURE_IMAGE) ) { // save current image with features
		static const char *image_filename_template = "feature_image_%05u.png";
		char *image_filename;
		if( asprintf(&image_filename, image_filename_template, run_counter ) < 0)
			return -123;

		cv::Mat color_image;
		cv::drawKeypoints(image, keypoints, color_image);
		cv::imwrite(image_filename, color_image);
		free(image_filename);
	}

	return 0;
}

int Tracker::log_debug_data(const std::bitset<8> &debug_mask,
                            const std::vector<float> &parameter_vector,
                            const std::vector<cv::Point2f> &ideal_points,
                            const std::vector<cv::Point2f> &op_projections,
                            const std::vector<cv::DMatch> &matches,
                            const std::vector<char> &matches_mask) {

	if( debug_mask.test(MATCHES) ) { // save matches
		static const char *filename_template = "matches_%05u.m";
		char *filename;
		if( asprintf(&filename, filename_template, run_counter ) < 0)
			return -120;
		std::ofstream f_stream(filename);
		free(filename);
		if( !f_stream.is_open() )
			return -121;

		const double cx = camera_matrix.at<double>(0, 2);
		const double cy = camera_matrix.at<double>(1, 2);
		const double fx = camera_matrix.at<double>(0, 0);
		const double fy = camera_matrix.at<double>(1, 1);

		// insert "header"
		f_stream << "%% Current feature matches at pose " << parameter_vector << std::endl
			<< "%% xyuv = x y (source image) x_displacement y_displacement" << std::endl;

		// insert projections and their displacements
		f_stream << "xyuv = [";
		for(unsigned int i=0; i<matches.size(); i++) {
			if(matches_mask[i] == 0) continue;
			const int src_index = matches[i].queryIdx;
			const int dst_index = matches[i].trainIdx;

			f_stream << op_projections[src_index].x*fx+cx << " "
				<< op_projections[src_index].y*fy+cy << " "
				<< (ideal_points[dst_index].x - op_projections[src_index].x)*fx+cx << " "
				<< (ideal_points[dst_index].y - op_projections[src_index].y)*fy+cy << " ";
		}
		f_stream << "];" << std::endl
			<< "xyuv = reshape(xyuv, 4, [])';" << std::endl
			<< "quiver(xyuv(:,1),xyuv(:,2), xyuv(:,3), xyuv(:,4))" << std::endl
			<< "axis(\"equal\")" << std::endl;

		// add label
		f_stream << "title(\"p = " << parameter_vector << "\");" << std::endl;

		// save plotting as image
// 		static const char *plot_template = "matches_%05u.png";
// 		if( asprintf(&filename, plot_template, counter ) < 0)
// 			return -122;
// 		f_stream << "print -dpng " << filename << std::endl;
// 		free(filename);

		f_stream.close();
	}

	return 0;
}

void Tracker::pose_estimation(const std::vector<float>& pose) {
	//reset time + speed
// 	last_imu_time = 0;
// 	speed[0] = speed[1] = speed[2] = 0.0;
	// constrain estimation of speed
	speed[0] /= 2.0; speed[1] /= 2.0; speed[2] /= 2.0;
	Tracker::pose = pose;
	dout << "updated pose to " << Tracker::pose << std::endl;
}

int Tracker::reset() {
	last_imu_time = 0;
	speed.assign(3, 0.0);
	pose.assign(6, 0.0);
	//TODO reset map

	return 0;
}

int Tracker::track_camera(const cv::Mat &image, std::vector<float> &parameter_vector, const float avg_depth, const std::bitset<8> &debug_mask) {
	run_counter++;

	dout << "[input] parameter_vector: " << parameter_vector << ", avg_depth: " << avg_depth << std::endl;

	// get features from image
	std::vector<cv::KeyPoint> keypoints;
	feature_detector.detect(image, keypoints);
	if(keypoints.size() < 6) {
		dout << "only " << keypoints.size() << " features found (not enough)" << std::endl;
		log_debug_data(debug_mask, image, parameter_vector, keypoints);
		return -1;
	}

	// calculate descriptors which can remove or add some of the keypoints
	cv::Mat descriptors;
	descriptor_extractor.compute(image, keypoints, descriptors);
	if(keypoints.size() < 6) {
		dout << "only " << keypoints.size() << " keypoints left after descriptor computation (not enough)" << std::endl;
		log_debug_data(debug_mask, image, parameter_vector, keypoints);
		return -2;
	}
	// log feature images
	log_debug_data(debug_mask, image, parameter_vector, keypoints);

	// undistort points
	std::vector<cv::Point2f> ideal_points;
	undistort_n2i(keypoints, camera_matrix, distortion_coefficients, ideal_points);

	// activate map (multiple starting doesn't harm)
	map.start();

	// get (possible) matching points and descriptors from map
// 	map.set_view(parameter_vector);
	std::vector<float> op_pose(parameter_vector);
	std::vector< cv::Point3_<float> > objectpoints;
	std::vector< cv::Point2f > op_projections;
	cv::Mat op_descriptors;
	map.reference_scene(op_pose, objectpoints, op_projections, op_descriptors);
// 	map.fill_tracking_points(objectpoints, op_projections, op_descriptors);

	if( objectpoints.empty() ) { // no suitable points found in map
		// pose update
		pose_estimation(parameter_vector);
		// add new features to map
		map.add_features(ideal_points,
			descriptors,
			parameter_vector,
			avg_depth);

		log_debug_data(debug_mask, image, parameter_vector, keypoints);

		return 1;
	}

	// match descriptors
	std::vector<cv::DMatch> matches;
	matcher.match(op_descriptors, descriptors, matches);
	if( matches.empty() ) {
		dout << "no matches found" << std::endl;
		log_debug_data(debug_mask, image, parameter_vector, keypoints);
		return -3;
	}
	dout << "found " << matches.size() << " matches " << std::endl;

	// filter matches
	std::vector<char> matches_mask(matches.size(), 1);
	filter_matches_by_robust_distribution(op_projections, ideal_points, matches, matches_mask);

	log_debug_data(debug_mask, parameter_vector, ideal_points, op_projections, matches, matches_mask);
	
	// do a rough translation estimation first
	estimate_translation_by_features(op_projections,
		ideal_points,
		average_depth(objectpoints),
		avg_depth,
		matches,
		&parameter_vector[3],
		matches_mask);
/*
	estimate_translation_by_objects(objectpoints,
		ideal_points,
		avg_depth,
		matches,
		&parameter_vector[3],
		matches_mask);
*/
/*
	// get estimatation of pose
	int rc = lm_pose_optimization< float, levmar_ideal_pinhole_quatvec<float>, levmar_ideal_pinhole_quatvec_jac<float> >(
// 	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_approx_ideal_pinhole_euler_jac<float> >(
// 	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_ideal_pinhole_euler_jac<float> >(
		objectpoints,
		ideal_points,
		matches,
		parameter_vector,
		cv::Mat(),
		matches_mask,
		10);	// max iterations
	if(rc <= 0) {
		dout << "position estimation failed with return code " << rc << std::endl;
		return -4;
	}
*/

	// pose update
	pose_estimation(parameter_vector);

	// log estimations of pose
	log_debug_data(debug_mask, parameter_vector);

	// add features with new estimation to map
	map.add_features(ideal_points,
		descriptors,
		parameter_vector,
		avg_depth);

	return 0;
}

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
