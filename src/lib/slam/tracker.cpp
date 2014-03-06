#include "tracker.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include "lib/slam/pose.h"

#define VERBOSE 0

#define LOG_DEBUG_DATA 0

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout << typeid(*this).name() << ": "
#else
#define dout if(0) std::cout
#endif

#include <typeinfo>

namespace hub {
namespace slam {

Tracker::Tracker(const int image_width,
	const int image_height,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients) :
		camera_matrix(camera_matrix),
		distortion_coefficients(distortion_coefficients),
		feature_detector(60, 3), //threshold, octaves
		descriptor_extractor(),
		matcher() {

	if(Tracker::camera_matrix.rows < 3 || Tracker::camera_matrix.cols < 3)
		Tracker::camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	if(Tracker::distortion_coefficients.total() < 5)
		Tracker::distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);

	map.set_camera(Tracker::camera_matrix, image_width, image_height);
}

Tracker::~Tracker() {
	map.join();
}

int Tracker::track_camera(const cv::Mat &image, std::vector<float> &parameter_vector, const float avg_depth) {

	// get features from image
	std::vector<cv::KeyPoint> keypoints;
	feature_detector.detect(image, keypoints);
	if(keypoints.empty()) {
		dout << "didn't found features" << std::endl;
		return -1;
	}

	// calculate descriptors which can remove or add some of the keypoints
	cv::Mat descriptors;
	descriptor_extractor.compute(image, keypoints, descriptors);
	if(keypoints.empty()) {
		dout << "keypoints empty after descriptor computation" << std::endl;
		return -2;
	}

	// undistort points
	std::vector<cv::Point2f> ideal_points;
	undistort_n2i(keypoints, camera_matrix, distortion_coefficients, ideal_points);

	// activate map (multiple starting doesn't harm)
	map.start();

	// get (possible) matching points and descriptors from map
	map.set_view(parameter_vector);
	std::vector< cv::Point3_<float> > objectpoints;
	std::vector< cv::Point2f > op_projections;
	cv::Mat op_descriptors;
	map.fill_tracking_points(objectpoints, op_projections, op_descriptors);

	if( objectpoints.empty() ) { // no suitable points found in map
		// add new features to map
		map.add_features(ideal_points,
			descriptors,
			parameter_vector,
			avg_depth);

		return 1;
	}

	// match descriptors
	std::vector<cv::DMatch> matches;
	matcher.match(op_descriptors, descriptors, matches);
	if( matches.empty() ) {
		dout << "no matches found" << std::endl;
		return -3;
	}

	// filter matches
	std::vector<char> matches_mask(matches.size(), 1);
	filter_matches_by_robust_distribution(op_projections, ideal_points, matches, matches_mask);

#if LOG_DEBUG_DATA
static uint32_t counter = 0;

static const char *log_template = "log_values.m";
std::ofstream log_stream;
if(counter)
	log_stream.open(log_template, std::ios_base::app);
else
	log_stream.open(log_template);
if( !log_stream.is_open() )
	return - 119;

if(!counter) {// add header
	log_stream << "% add the following lines (without #) to the end of the file" << std::endl
		<< "%];" << std::endl
		<< "%pose = reshape(pose, 6, [])';" << std::endl;

	log_stream << "pose = [";
} else
	log_stream << ", ";
log_stream << parameter_vector[0] << ", "
	<< parameter_vector[1] << ", "
	<< parameter_vector[2] << ", "
	<< parameter_vector[3] << ", "
	<< parameter_vector[4] << ", "
	<< parameter_vector[5];

static const char *filename_template = "matches_%05u.m";
char *filename;
if( asprintf(&filename, filename_template, counter ) < 0)
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
f_stream << "% rpy = [" << rad2deg(parameter_vector[0]) << " "
	<< rad2deg(parameter_vector[1]) << " "
	<< rad2deg(parameter_vector[2]) << "];" << std::endl;
f_stream << "% t = [" << parameter_vector[3] << " "
	<< parameter_vector[4] << " "
	<< parameter_vector[5] << "];" << std::endl;

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
f_stream << "];" << std::endl;;
f_stream << "xyuv = reshape(xyuv, 4, size(xyuv)(2)/4);" << std::endl;
f_stream << "quiver(xyuv(1,:),xyuv(2,:), xyuv(3,:), xyuv(4,:))" << std::endl;

// add label
f_stream << "title(\"p = " << rad2deg(parameter_vector[0]) << ", "
<< rad2deg(parameter_vector[1]) << ", "
<< rad2deg(parameter_vector[2]) << " | "
	<< parameter_vector[3] << ", "
	<< parameter_vector[4] << ", "
	<< parameter_vector[5] << "\");" << std::endl;

// save plotting as image
static const char *plot_template = "matches_%05u.png";
if( asprintf(&filename, plot_template, counter ) < 0)
	return -122;
f_stream << "print -dpng " << filename << std::endl;
free(filename);

// save current image with features
static const char *image_template = "image_%05u.png";
if( asprintf(&filename, image_template, counter ) < 0)
	return -123;
cv::Mat color_image;
cv::drawKeypoints(image, keypoints, color_image);
cv::imwrite(filename, color_image);
free(filename);

f_stream.close();
counter++;
#endif

	//get translation
	guess_translation(objectpoints,
		ideal_points,
		avg_depth,
		matches,
		&parameter_vector[3],
		matches_mask);

/*
	// get estimatation of pose
	int rc = guess_pose< float, levmar_ideal_pinhole_quatvec<float>, levmar_ideal_pinhole_quatvec_jac<float> >(
// 	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_approx_ideal_pinhole_euler_jac<float> >(
// 	int rc = guess_pose< float, levmar_ideal_pinhole_euler<float>, levmar_ideal_pinhole_euler_jac<float> >(
		objectpoints,
		ideal_points,
		matches,
		parameter_vector,
		matches_mask);
	if(rc <= 0) {
		dout << "position estimation failed" << std::endl;
		return -4;
	}
*/

	// add features with new estimatio to map
	map.add_features(ideal_points,
		descriptors,
		parameter_vector,
		avg_depth);

	return 0;
}

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
