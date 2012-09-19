#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <inttypes.h>
#include <fstream>
#include <cmath>	//sqrt

#include "lib/slam/bundleadjust.h"
#include "utility.h"

using namespace mavhub;
using namespace hub::slam;

template<typename T>
std::istream& operator >>(std::istream &is, cv::Point_<T> &point) {
	is >> point.x; is >> point.y;

	return is;
}

template<typename T>
std::istream& operator >>(std::istream &is, cv::Point3_<T> &point) {
	is >> point.x;
	is >> point.y;
	is >> point.z;

	return is;
}

template<typename T = double>
struct feature_t {
	cv::Point_<T> image_point;
	uint32_t point_index;	///index of object point in database
};

// workaround for template typedef
template<typename T = double>
struct scene_t {
	typedef std::list< feature_t<T> > type;
};

void add_feature(const uint32_t index, const feature_t<> &feature, std::list< scene_t<>::type > &scenes) {
	while(scenes.size() < index+1) {
		scenes.push_back( scene_t<>::type() );
	}
	std::list< scene_t<>::type >::iterator scene_iter = scenes.begin();
	for(uint32_t i=0; i<index; i++) scene_iter++;

	BOOST_CHECK( scene_iter != scenes.end() );
	scene_iter->push_back(feature);
}

int read_points(const std::string &fname, std::vector<cv::Point3d> &object_points, std::list< scene_t<>::type > &scenes) {
	std::ifstream fstream(fname.c_str(), std::ifstream::in);
	if(fstream.fail()) return -1;

	object_points.clear();
	scenes.clear();
	uint32_t point_index = 0;

	std::string line;
	while(std::getline(fstream, line)) { //read line by line
		if(line.empty()) continue;

		std::string::size_type start = line.find_first_not_of(" \t\n");
		//skip comment lines
		if(start != std::string::npos && line[start] == '#') 
			continue;

		std::istringstream val_stream(line.substr(start, std::string::npos));

		cv::Point3d object_point;
		val_stream >> object_point;
		object_points.push_back(object_point);
		
		uint32_t num_scenes;
		val_stream >> num_scenes;
		BOOST_CHECK(num_scenes <= 16); // sanity check

		//read image points
		feature_t<> feature;
		feature.point_index = point_index;
		for(uint32_t i=0; i<num_scenes; i++) {
			uint32_t scene_index;
			val_stream >> scene_index;
			val_stream >> feature.image_point;

			add_feature(scene_index, feature, scenes);
		}

		point_index++;
	}

	fstream.close();

	return point_index;
}

void set_visibility_mask(const std::vector<cv::Point3d> &object_points,
	const std::list< scene_t<>::type > &scenes,
	std::vector<char> &visibility_mask) {
	
	uint32_t num_images = scenes.size();
	BOOST_CHECK(num_images < 10);
	visibility_mask.resize(object_points.size()*num_images, 0);

	// vmask[i, j]=1 if point i visible in image j, 0 otherwise	
	char *vmask = &visibility_mask[0];
	uint32_t j = 0;
	for(std::list< scene_t<>::type >::const_iterator scene_iter = scenes.begin();
	scene_iter != scenes.end();
	++scene_iter) { // iterate through scenes
		for(scene_t<>::type::const_iterator feature_iter = scene_iter->begin();
		feature_iter != scene_iter->end();
		++feature_iter) { // iterate through points
			const uint32_t i = feature_iter->point_index;
			vmask[i*num_images + j] = 1;
		}
		j++;
	}
}

int read_pose(const std::string &fname, std::vector<double> &parameters, std::vector<double> &rotations) {

	std::ifstream fstream(fname.c_str(), std::ifstream::in);
	if(fstream.fail()) return -1;

	parameters.clear();

	std::string line;
	while(std::getline(fstream, line)) { //read line by line
		if(line.empty()) continue;

		std::string::size_type start = line.find_first_not_of(" \t\n");
		//skip comment lines
		if(start != std::string::npos && line[start] == '#') 
			continue;

		std::istringstream val_stream(line.substr(start, std::string::npos));
		double value;
		for(int i=0; i<4; i++) { //read rotation
			val_stream >> value;
			rotations.push_back(value);
		}
		for(int i=0; i<3; i++) { //clear rotation in parameters
			parameters.push_back(0.0);
		}
		for(int i=0; i<3; i++) { //read translation
			val_stream >> value;
			parameters.push_back(value);
		}
// 		val_stream >> value;
// 		rotations.push_back(value);
// 		for(int i=0; i<6; i++) {
// 			val_stream >> value;
// 			parameters.push_back(value);
// 			rotations.push_back(value);
// 		}
	}

	return 0;
}

void set_measurements(const std::list< scene_t<>::type > &scenes,
	const uint32_t num_points,
	std::vector<double> &measurements) {
	
	//FIXME: improve performance
	measurements.clear();
	for(uint32_t i=0; i<num_points; i++) {
		for(std::list< scene_t<>::type >::const_iterator scene_iter = scenes.begin();
		scene_iter != scenes.end();
		++scene_iter) { // iterate through scenes
			for(scene_t<>::type::const_iterator feature_iter = scene_iter->begin();
			feature_iter != scene_iter->end();
			++feature_iter) { // iterate through points
				if(feature_iter->point_index == i) {
					measurements.push_back(feature_iter->image_point.x);
					measurements.push_back(feature_iter->image_point.y);
					break;
				}
			}
		}
	}
}

int read_calibration(const std::string &fname, std::vector<double> &intrinsics) {
	std::ifstream fstream(fname.c_str(), std::ifstream::in);
	if(fstream.fail()) return -1;

	intrinsics.clear();

	std::string line;
	while(std::getline(fstream, line)) { //read line by line
		if(line.empty()) continue;

		std::string::size_type start = line.find_first_not_of(" \t\n");
		//skip comment lines
		if(start != std::string::npos && line[start] == '#') 
			continue;

		std::istringstream val_stream(line.substr(start, std::string::npos));
		double value;
		for(int i=0; i<3; i++) {
			val_stream >> value;
			intrinsics.push_back(value);
		}
	}
	
	return 0;
}

BOOST_AUTO_TEST_SUITE(SBATestSuite)

BOOST_AUTO_TEST_CASE(Test_sba)
{

	std::vector<cv::Point3d> object_points;
	std::list< scene_t<>::type > scenes;
	BOOST_CHECK( read_points("../thirdparty/sba/demo/7pts.txt", object_points, scenes) >= 0 );
	unsigned int num_points = object_points.size();
	unsigned int num_images = scenes.size();

	std::vector<char> visibility_mask;
	set_visibility_mask(object_points, scenes, visibility_mask);

	// parameter vector p0: (a1, ..., am, b1, ..., bn). aj are the image j parameters, bi are the i-th point parameters,
	std::vector<double> parameters;
	parameters.reserve(num_images*num_params_per_cam + num_points*num_params_per_point);
	// initial rotations
	std::vector<double> rotations;
	rotations.reserve(num_images*4);
	read_pose("../thirdparty/sba/demo/7cams.txt", parameters, rotations);
	//append object points
	parameters.insert(parameters.end(), &(object_points[0].x), &(object_points[0].x) + 3*object_points.size());
	BOOST_CHECK((unsigned)parameters.size() == num_images*num_params_per_cam + num_points*num_params_per_point);

	std::vector<double> measurements;
	set_measurements(scenes, num_points, measurements);
	BOOST_CHECK((unsigned)measurements.size() <= num_points*num_images*num_params_per_measuremnt);
	const unsigned int num_projections = (unsigned)measurements.size() / num_params_per_measuremnt;

	std::vector<double> intrinsic_parameters(9);
	read_calibration("../thirdparty/sba/demo/calib.txt", intrinsic_parameters);
	cv::Mat camera_matrix(3, 3, CV_64F, &intrinsic_parameters[0]);

	// set up global data
	sba_model_data_t<> g_data(camera_matrix, rotations);

	static const int max_iterations = 100;
	int verbosity = 0;
	double opts[SBA_OPTSSZ];
	opts[0]=SBA_INIT_MU; opts[1]=SBA_STOP_THRESH;
	opts[2]=SBA_STOP_THRESH; opts[3]=SBA_STOP_THRESH;
	opts[4]=0.0;
	double info[SBA_INFOSZ];

	uint64_t start_time = get_time_us();
	int rc = sba_motstr_levmar_x(num_points,	// number of points
			    0,				// number of points not to be modified
			    num_images,			// number of images
			    0,				// number of images not to be modified
			    &visibility_mask[0],	// visibility mask
			    &parameters[0],		// initial parameter vector
			    num_params_per_cam,		// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
			    num_params_per_point,	// number of parameters for ONE point; e.g. 3 for Euclidean points
			    &measurements[0],		// measurement vector
			    NULL,			// measurements covariance matrices
			    num_params_per_measuremnt,	// number of parameters for EACH measurement; usually 2
			    sba_pinhole_model,		// model function
			    sba_pinhole_model_jac,	// jacobian of model function
			    (void*)&g_data,		// additional data
			    max_iterations,		// maximum number of iterations
			    verbosity,			// verbosity
			    opts,			// options
			    info			// information regarding the minimization
	);
	uint64_t stop_time = get_time_us();

	BOOST_TEST_MESSAGE("SBA using " << num_points << " 3D pts, "
		<< num_images << " frames and "
		<< num_projections << " image projections, "
		<< num_images*num_params_per_cam+num_points*num_params_per_point << " variables"
	);
	BOOST_TEST_MESSAGE("SBA returned " << rc
		<< " in " << info[5]
		<< " iterations, reason " << info[6]
		<< ", error " << info[1] / num_projections
		<< " [initial " << info[0] / num_projections
		<< "], " << (int)info[7] << "/" << (int)info[8]
		<< " func/fjac evals, " << (int)info[9] << " lin. systems."
	);
	BOOST_TEST_MESSAGE("Elapsed time: " << (stop_time-start_time)/1000 << "ms");
	BOOST_CHECK( (info[1] / num_projections) < 0.7 );
}

BOOST_AUTO_TEST_SUITE_END()
