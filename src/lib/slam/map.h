#ifndef _HUB_MAP_H_
#define _HUB_MAP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#if CV_MINOR_VERSION >= 2

#include <list>
#include <fstream>

#include "lib/hub/thread.h"
#include "lib/hub/math.h"
#include "lib/hub/time.h"
#include "lib/hub/utility.h"
#include "lib/slam/camera.h"
#include "lib/slam/features.h"
#include "lib/slam/bundleadjust.h"

//FIXME: remove dependency to BRISK
#include "brisk/brisk.h"

#define _HUB_MAP_VERBOSE 0

// little debug makro
#if _HUB_MAP_VERBOSE >= 1
#define dout if(1) std::cout << typeid(*this).name() << ": "
#else
#define dout if(0) std::cout
#endif

namespace hub {
namespace slam {

extern const char *ply_header_start;
extern const char *ply_vertex_element;
extern const char *ply_face_element;
extern const char *ply_header_end;

template<typename T=float>
class Map : public hub::PThread {

	/**
	 * \brief Struct describing (ideal) camera.
	 */
	struct camera_t {
		camera_t() : ideal_width(0), ideal_height(0), fov_radius(0), pose() {};
		camera_t(const T ideal_width,
			const T ideal_height,
			const T radius,
			const std::vector<T> &pose) :
				ideal_width(ideal_width),
				ideal_height(ideal_height),
// 				fov_angle_rad(angle),
				fov_radius(radius),
// 				radius_1m(radius),
				pose(pose) { };

		T ideal_width; /// Image width normalized by focal length.
		T ideal_height; /// Image height normalized by focal length.
// 		T fov_angle_rad; /// Camera angle between optical axis and points which are projected to one image corner
		T fov_radius; /// Radius of field of view (inner circle) normalized by focal length.
// 		T angle; /// Camera angle between optical axis and points which are projected to one image corner
// 		//FIXME 1m radius depends on  the scale chosen in the intrinsic parameters
// 		T radius_1m; /// The radius of the circle a view cone would produce at distance of 1m from the camera.
		std::vector<T> pose;
	};

	/**
	 * \brief Struct containing all the data coming from the tracker.
	 */ 
	struct queue_data_t {
		queue_data_t(const std::vector<cv::Point2f>& projections,
			const cv::Mat& descriptors,
			const std::vector<T>& camera_pose,
			const T average_depth) :
				projections(projections),
				descriptors(descriptors),
				pose(camera_pose),
				avg_depth(average_depth) { };

		std::vector<cv::Point2f> projections;
		cv::Mat descriptors;
		std::vector<T> pose;
		T avg_depth;
	};

	/**
	 * \brief Struct holding information belonging to one point of view (keyframe).
	 */
	struct scene_t {
		scene_t(const std::vector<T> camera_pose,
			std::vector<T> view_point,
			std::vector<cv::Point2f> projections,
			std::vector<uint16_t> point_indices) :
				camera_pose(camera_pose),
				view_point(view_point),
				projections(projections),
				point_indices(point_indices) { };

		std::vector<T> camera_pose; // camera orientaion and position
		std::vector<T> view_point;	// 3D-point where the camera is looking at a predefined plane
		std::vector<cv::Point2f> projections;	// image projections
		std::vector<uint16_t> point_indices;	// corresponding object point indices of the projections
	};

public:
	Map();

	/**
	 * \brief Add new features to map.
	 * \param projections Undistorted image projections of 3D object points in ideal coordinates.
	 * \param descriptors Matrix containing the descriptors. 
	 * \param camera_pose Rotation as quaternion vector part and translation of camera.
	 * \param average_depth Average depth of image scenery.
	 */
	void add_features(const std::vector<cv::Point2f>& projections,
		const cv::Mat& descriptors,
		const std::vector<T>& camera_pose,
		const T average_depth = 100.0);
	/**
	 * \brief Get all points with descriptor from map you would see with the configured camera.
	 */
	void fill_tracking_points(std::vector< cv::Point3_<T> > &objectpoints,
		std::vector<cv::Point2f> &projections,
		cv::Mat &descriptors);

	/**
	 * \brief Save map points (point cloud) as polygon file (ply).
	 */
	int save_points(const std::string &file_name) const;

	/**
	 * \brief Save camera views as polygon file (ply).
	 */
	int save_views(const std::string &file_name) const;

	/**
	 * \brief Parametrize the pinhole camera model.
	 */
	void set_camera(const cv::Mat& camera_matrix, const int width, const int height);
	/**
	 * \brief Set pose (rotation and translation) of camera.
	 */
	void set_view(const std::vector<T>& parameter_vector);

protected:
	virtual void run();

private:
	camera_t camera;

	//TODO: are k-d-trees an alternative?
	std::vector< cv::Point3_<T> > objectpoints;
	// point descriptors
	cv::Mat descriptors;
	// object point reference counter
	std::vector<uint8_t> reference_counters;
	std::list<queue_data_t> working_queue;
	std::list<scene_t> scenes;
	
	// mutexes
	mutable pthread_mutex_t cam_mutex; /// Mutex to protect camera information
	mutable pthread_mutex_t map_mutex; /// Mutex to protect 3D points
	mutable pthread_mutex_t work_mutex; /// Mutex to protect working data
	mutable pthread_mutex_t scene_mutex; /// Mutex to protect scene data

	/**
	 * \brief Add objectpoint to map.
	 *
	 * This function is not thread-safe.
	 * \param point
	 * \param descriptor
	 * \retval index
	 */
	int add_point(const cv::Point3_<T>& point, const cv::Mat& descriptor);

	int check_viewpoint_distance(const std::vector<T> &view_point, const T max_distance) const;

	bool is_keyframe(const std::vector<T> &camera_pose, const std::vector<T> &view_point) const;
	
	void local_bundle_adjust();

	void process_working_task();
	/**
	 * \brief Get all visible points from map with given camera pose.
	 * \warning This function is not thread-safe. Be care of map_mutex and cam_mutex.
	 * \param[in] pose camera pose
	 * \param[out] objectpoints visible objectptoints.
	 * \param[out] projections projections of visible objectpoints in ideal coordinates.
	 * \param[out] descriptors corresponding feature descriptors.
	 */
	void visible_points(const std::vector<T> &pose,
		std::vector< cv::Point3_<T> > &objectpoints,
		std::vector<cv::Point2f> &projections,
		cv::Mat &descriptors,
		std::vector<uint16_t> &map_indices);

};

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------

template<typename T>
Map<T>::Map() {
	//init mutexes
	pthread_mutex_init(&cam_mutex, NULL);
	pthread_mutex_init(&map_mutex, NULL);
	pthread_mutex_init(&work_mutex, NULL);
	pthread_mutex_init(&scene_mutex, NULL);
}

template<typename T>
void Map<T>::add_features(const std::vector<cv::Point2f>& projections,
	const cv::Mat& descriptors,
	const std::vector<T>& camera_pose,
	const T average_depth) {

	if(projections.size() != (unsigned)descriptors.rows)
		return;
	if(camera_pose.size() != 6)
		return;

	queue_data_t working_task(projections, descriptors, camera_pose, average_depth);

	hub::Lock work_lock(work_mutex);
	working_queue.push_back(working_task);

	dout << "added task to working queue" << std::endl;
}

template<typename T>
int Map<T>::add_point(const cv::Point3_<T>& point, const cv::Mat& descriptor) {

	objectpoints.push_back(point);
	descriptors.push_back(descriptor);
//FIXME: resize ref counters
	return objectpoints.size()-1;
}

template<typename T>
void Map<T>::fill_tracking_points(std::vector< cv::Point3_<T> > &objectpoints,
	std::vector<cv::Point2f> &projections,
	cv::Mat &descriptors) {

	hub::Lock map_lock(map_mutex);
	hub::Lock cam_lock(cam_mutex);

	//FIXME: point_indices
	std::vector<uint16_t> point_indices;
	visible_points(camera.pose, objectpoints, projections, descriptors, point_indices);
}

template<typename T>
bool Map<T>::is_keyframe(const std::vector<T> &camera_pose, const std::vector<T> &view_point) const {
	// to show that the current frame is NOT a keyframe we have to find at least one
	// scenery which is near current frame (meaning viewpoints are near) and where
	// the camera pose is above current.

// 	T camera_fov_radius;
// 	{ hub::Lock cam_lock(cam_mutex);
// 		camera_fov_radius = camera.fov_radius;
// 	}

	cv::L2<T> euclidean_distance;
	const T max_distance = 0.5; //FIXME max_distance depends on altitude and camera
	hub::Lock scene_lock(scene_mutex);

	// iterate through scenes in reverse order
	for(typename std::list<scene_t>::const_reverse_iterator scene_iter = scenes.rbegin();
	scene_iter != scenes.rend();
	++scene_iter) {
// 		const T current_fov_radius = camera_fov_radius*camera_pose[5];
// 		const T scene_fov_radius = camera_fov_radius*scene_iter->camera_pose[5];
// 		const T fov_radius = std::min(current_fov_radius, scene_fov_radius);


		const T altitude_distance = camera_pose[5] - scene_iter->camera_pose[5];

		// all viewpoints lie in same plane => 2D euclidean is enough
		const T viewpoint_distance = euclidean_distance(&view_point[0], &(scene_iter->view_point[0]), 2);

// std::cout << "altitude distance: " << altitude_distance << " = " << camera_pose[5] << "-" << scene_iter->camera_pose[5] << ", vp distance: " << viewpoint_distance << std::endl;
		if(altitude_distance <= 0.2 && viewpoint_distance <= max_distance)
			return false;
	}

	return true;
}

template<typename T>
void Map<T>::local_bundle_adjust() {
	//determine local neighborhood
	std::list<scene_t*> local_scenes;
	typename std::list<scene_t>::const_reverse_iterator new_scene = scenes.rbegin();
	std::vector< indexed_item_t<T> > indexed_distances; indexed_distances.reserve( scenes.size() );
	cv::L2<T> euclidean_distance;
	uint16_t j = 0;
	for(typename std::list<scene_t>::const_iterator scene_iter = scenes.begin();
	scene_iter != scenes.end();
	++scene_iter) { // iterate through scenes
		const T distance = euclidean_distance(&(scene_iter->view_point[0]), &(new_scene->view_point[0]), 3);
		indexed_distances.push_back( indexed_item_t<T>(distance, j) );
		j++;
	}
	//sort distances
	std::sort(indexed_distances.begin(), indexed_distances.end());

	//get k-nearest scenes
	j = 0;
	for(typename std::list<scene_t>::iterator scene_iter = scenes.begin();
	scene_iter != scenes.end();
	++scene_iter) { // iterate through scenes
		for(unsigned int i=0; i<indexed_distances.size() && i<3; i++) {
			if(j == indexed_distances[i].index) {
				local_scenes.push_back( &(*scene_iter) );
				break;
			}
		}
		j++;
	}

	const uint16_t num_images = local_scenes.size();
	//FIXME: only use points which are visible in at least one scene
	const uint16_t num_points = objectpoints.size();

	dout << "entering local_bundle_adjust with " << num_images << " images and " << num_points << " points" << std::endl;
	std::vector<char> visibility_mask(num_points*num_images, 0);
	// vmask[i, j]=1 if point i visible in image j, 0 otherwise
	char *vmask = &visibility_mask[0];
	// parameter vector p0: (a1, ..., am, b1, ..., bn). aj are the image j parameters, bi are the i-th point parameters,
	std::vector<double> parameters;
	parameters.reserve(num_images*num_params_per_cam + num_points*num_params_per_point);

// 	dout << "fill image parameters" << std::endl;
	j = 0;
	for(typename std::list<scene_t*>::const_iterator scene_iter = local_scenes.begin();
	scene_iter != local_scenes.end();
	++scene_iter) { // iterate through scenes
		for(std::vector<uint16_t>::const_iterator index_iter = (*scene_iter)->point_indices.begin();
		index_iter != (*scene_iter)->point_indices.end();
		++index_iter) { // iterate through point indices
			const uint16_t i = *index_iter;
			vmask[i*num_images + j] = 1;
		}
		j++;

		// add rotation and translation
		parameters.insert(parameters.end(), (*scene_iter)->camera_pose.begin(), (*scene_iter)->camera_pose.end());
	}
// 	dout << "fill point parameters" << std::endl;
	//append object points
	//FIXME: double <-> float mismatch
// 	parameters.insert(parameters.end(), &(objectpoints[0].x), &(objectpoints[0].x) + 3*objectpoints.size());
	for(unsigned int i=0; i<objectpoints.size(); i++) {
		parameters.push_back(objectpoints[i].x);
		parameters.push_back(objectpoints[i].y);
		parameters.push_back(objectpoints[i].z);
	}

	std::vector<double> measurements;
	//FIXME: improve performance
	for(unsigned int i=0; i<num_points; i++) {
		for(typename std::list<scene_t*>::const_iterator scene_iter = local_scenes.begin();
		scene_iter != local_scenes.end();
		++scene_iter) { // iterate through scenes
			for(unsigned int k=0; k<(*scene_iter)->projections.size(); k++) {
				if((*scene_iter)->point_indices[k] == i) {
					measurements.push_back((*scene_iter)->projections[k].x);
					measurements.push_back((*scene_iter)->projections[k].y);
					break;
				}
			}
		}
	}
	dout << "filled " << measurements.size() << " measurements" << std::endl;

#if _HUB_MAP_VERBOSE
static unsigned int run_counter;
static const char *cams_fn_template = "cams_%02u.txt";
char *filename;
if( asprintf(&filename, cams_fn_template, run_counter ) < 0)
	exit(-1);
std::ofstream cam_stream(filename);
free(filename);
for(unsigned int i=0; i<local_scenes.size(); i++) {
	double quaternion[4];
	vec2quat(&parameters[i*6], quaternion);
	cam_stream << quaternion[0] << " " << quaternion[1] << " "  << quaternion[2] << " "  << quaternion[3];
	cam_stream << " " << -parameters[i*6+3] << " "  << parameters[i*6+4] << " "  << parameters[i*6+5] << std::endl;
}
cam_stream.close();

static const char *pts_fn_template = "pts_%02u.txt";
if( asprintf(&filename, pts_fn_template, run_counter ) < 0)
	exit(-1);
std::ofstream pts_stream(filename);
free(filename);
pts_stream << "# X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ..." << std::endl;
for(unsigned int i=0; i<num_points; i++) {
	unsigned int num_scenes = 0;
	std::stringstream point_line;
	unsigned int scene_number = 0;
	for(typename std::list<scene_t*>::const_iterator scene_iter = local_scenes.begin();
	scene_iter != local_scenes.end();
	++scene_iter) { // iterate through scenes
		for(unsigned int k=0; k<(*scene_iter)->projections.size(); k++) {
			if((*scene_iter)->point_indices[k] == i) {
				point_line << " " << scene_number << " " << (*scene_iter)->projections[k].x << " " << (*scene_iter)->projections[k].y;
				num_scenes++;
				break;
			}
		}
		scene_number++;
	}
	if(num_scenes == 0) continue;
	pts_stream << -objectpoints[i].x << " " << objectpoints[i].y << " " << objectpoints[i].z;
	pts_stream << " " << num_scenes << point_line.str() << std::endl;
}
pts_stream.close();

if(!run_counter) {
	std::ofstream calib_stream("calib.txt");
	calib_stream << "1.0 0.0 0.0" << std::endl
		<< "0.0 1.0 0.0" << std::endl
		<< "0.0 0.0 1.0" << std::endl;
	calib_stream.close();
}
run_counter++;
#endif

// 	assert(parameters.size() == (unsigned)(num_images*num_params_per_cam + num_points*num_params_per_point));
// 	assert(rotations.size() == (unsigned)(num_images*quaternion_size));
	dout << "sba_motstr" << std::endl;
	double opts[SBA_OPTSSZ];
	opts[0]=SBA_INIT_MU; opts[1]=SBA_STOP_THRESH;
	opts[2]=SBA_STOP_THRESH; opts[3]=SBA_STOP_THRESH;
	opts[4]=0.0;
	// run bundleadjust
	int rc = sba_motstr_levmar_w(num_points,// number of points
		0,				// number of points not to be modified
		num_images,			// number of images
		0,				// number of images not to be modified
		&visibility_mask[0],		// visibility mask
		&parameters[0],			// initial parameter vector
		num_params_per_cam,		// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
		num_params_per_point,		// number of parameters for ONE point; e.g. 3 for Euclidean points
		&measurements[0],		// measurement vector
		NULL,				// measurements covariance matrices
		num_params_per_measuremnt,	// number of parameters for EACH measurement; usually 2
		sba_ideal_pinhole_model,	// model function
		sba_ideal_pinhole_model_jac,	// jacobian of model function
		sba_tukey_estimator,		// weighting function
		NULL,				// additional data
		20,				// maximum number of iterations
		0,				// verbosity
		opts,				// options
		NULL				// information regarding the minimization
	);
	if(rc == SBA_ERROR) {
		dout << "SBA_ERROR occured" << std::endl;
		return;
	}
// 	dout << "finished sba_motstr" << std::endl;

	// update poses
	j = 0;
	for(typename std::list<scene_t*>::iterator scene_iter = local_scenes.begin();
	scene_iter != local_scenes.end();
	++scene_iter) { // iterate through scenes
		dout << "|" << j << "|: (" << (*scene_iter)->camera_pose[0] << ", " << (*scene_iter)->camera_pose[1] << ", " << (*scene_iter)->camera_pose[2] << ", " << (*scene_iter)->camera_pose[3] << ", " << (*scene_iter)->camera_pose[4] << ", " << (*scene_iter)->camera_pose[5] << ") -> (" \
			<< parameters[j*num_params_per_cam] << ", " << parameters[j*num_params_per_cam+1] << ", " << parameters[j*num_params_per_cam+2] << ", " << parameters[j*num_params_per_cam+3] << ", " << parameters[j*num_params_per_cam+4] << ", " << parameters[j*num_params_per_cam+5] << ")" << std::endl;
		memcpy( &((*scene_iter)->camera_pose[0]), &parameters[j*num_params_per_cam], 6);
		j++;
	}
	// update points
	const unsigned int param_offset = num_images*num_params_per_cam;
	for(unsigned int i=0; i<objectpoints.size(); i++) {
		dout << "[" << i << "]: (" << objectpoints[i].x << ", " << objectpoints[i].y << ", " << objectpoints[i].z << ") -> (" \
			<< parameters[param_offset + 3*i] << ", " << parameters[param_offset + 3*i + 1] << ", " << parameters[param_offset + 3*i +2] << ")" << std::endl;
		objectpoints[i].x = parameters[param_offset + 3*i];
		objectpoints[i].y = parameters[param_offset + 3*i + 1];
		objectpoints[i].z = parameters[param_offset + 3*i + 2];
	}
	dout << "updated db with refined pose and objectpoints" << std::endl;

}

template<typename T>
void Map<T>::process_working_task() {
	if( working_queue.empty() )
		return;

	hub::Lock work_lock(work_mutex);

	//calculate intersection of optical axis with plane parallel to xy-plane and going through (0,0,100)
	const T plane_point[3] = {0.0, 0.0, 100.0};
	const T z_axis[3] = {0.0, 0.0, 1.0};
	const std::vector<T>& camera_pose = working_queue.front().pose;
	T rotation_matrix[9];
	rotation_matrix_quatvec(&camera_pose[0], rotation_matrix);
	T optical_axis[3];
	multiply(rotation_matrix, z_axis, optical_axis);
	std::vector<T> view_point(3);
	int rc = intersection<T>(plane_point,
		z_axis,
		&camera_pose[3],
		optical_axis,
		&view_point[0]);
	if(rc < 0) { //no intersection
		dout << "camera is not looking at ground, throw image away" << std::endl;
		//throw task away
		working_queue.pop_front();
		return;
	}
	dout << "camera pose:  (" << camera_pose[0] << ", " << camera_pose[1] << ", " << camera_pose[2] << ", " << camera_pose[3] << ", " << camera_pose[4] << ", " << camera_pose[5] << ")" << std::endl;
	dout << "optical axis: (" << optical_axis[0] << ", " << optical_axis[1] << ", " << optical_axis[2] << ")" << std::endl;
	dout << "viewpoint:    (" << view_point[0] << ", " << view_point[1] << ", " << view_point[2] << ")" << std::endl;

	const cv::Mat& descriptors = working_queue.front().descriptors;
	const std::vector<cv::Point2f>& projections = working_queue.front().projections;
	const T average_depth = working_queue.front().avg_depth;

	hub::Lock map_lock(map_mutex);
// 	if(objectpoints.size() != (unsigned)Map<T>::descriptors.rows) {
// 		dout << "ERROR: inconsistent map" << std::endl;
// 		return;
// 	}
	if( objectpoints.empty() ) { // add all projections to map
		objectpoints.resize( projections.size() );
		reference_counters.assign(objectpoints.size(), 0);
		std::vector<uint16_t> indices( projections.size() );
		// fill objectpoints
		//FIXME: use function
		for(unsigned int i=0; i<projections.size(); i++) {
			objectpoints[i] = cv::Point3_<T>(projections[i].x*average_depth,
				projections[i].y*average_depth,
				 average_depth);
			indices[i] = i;
		}
		Map<T>::descriptors = descriptors;

		// add first scene
		hub::Lock scene_lock(scene_mutex);
		scenes.clear();
		scenes.push_back( scene_t(camera_pose, view_point, projections, indices) );

		dout << "added first " << objectpoints.size() << " points to map" << std::endl;
		return;
	}

	if( !is_keyframe(camera_pose, view_point) ) {
		dout << "reject image as keyframe" << std::endl;
		//throw task away
		working_queue.pop_front();
		return;
	}

/*
	T view_radius;
	{
		hub::Lock cam_lock(cam_mutex);
		view_radius = camera.radius_1m; //1m equals 100.0
	}
	//FIXME viewpoint distance is not enough to decide for keyframe (think about ascending or descending)
	rc = check_viewpoint_distance(view_point, view_radius);
	if(rc <= 0) { // we already have a near enough keyframe
		dout << "reject image as keyframe" << std::endl;
		//throw task away
		working_queue.pop_front();
		return;
	}
*/
	// only take visible objectpoints into account
	std::vector< cv::Point3_<T> > visible_objectpoints;
	std::vector<cv::Point2f> visible_projections;
	std::vector<uint16_t> visible_map_indices;
	cv::Mat visible_descriptors;
	{
		hub::Lock cam_lock(cam_mutex);
		visible_points(camera_pose,
			visible_objectpoints,
			visible_projections,
			visible_descriptors,
			visible_map_indices);
	}

	//scenery data
	std::vector<uint16_t> point_indices( projections.size(), 0 );

	//for every image projection find nearest map point (in 2D)
	std::vector< indexed_item_t<T> > indexed_distances; indexed_distances.reserve( visible_projections.size() );
	cv::L2<T> euclidean_distance;
#ifdef HAVE_SSSE3
	//FIXME: distance operator depends on descriptor and hasn't to be hamming
	cv::HammingSse hamming_distance;
#else
	cv::Hamming hamming_distance;
#endif
	static const T distance_threshold = 15.0/250.0; //approx 5px (FIXME: make generic)
	dout << "match new points with existing points" << std::endl;
	for(unsigned int i=0; i<projections.size(); i++) {
		indexed_distances.clear();
		for(unsigned int j=0; j<visible_projections.size(); j++) {
			const T distance = euclidean_distance(&(projections[i].x), &(visible_projections[j].x), 2);
			if(distance > distance_threshold) continue;

			indexed_distances.push_back( indexed_item_t<T>(distance, j) );
		}

		//projection not near a map point
		if( indexed_distances.empty() ) continue;

		//sort distances
		std::sort(indexed_distances.begin(), indexed_distances.end());
		//choose point with best fitting descriptor distance
		std::vector<int> hamming_distances; hamming_distances.reserve(3);
		//take k-nearest points into account
		for(unsigned int k=0; k<indexed_distances.size(); k++) {
// 		for(unsigned int k=0; k<3 && k<indexed_distances.size(); k++) {
			int distance = hamming_distance( visible_descriptors.ptr(indexed_distances[k].index), descriptors.ptr(i), 16 );
			hamming_distances.push_back(distance);
		}
		//get minimal descrptor distance
		typename std::vector<int>::iterator min_iter = std::min_element(hamming_distances.begin(), hamming_distances.end());
		const int distance_index = min_iter - hamming_distances.begin();

		//point already in db => only add projection
		point_indices[i] = indexed_distances[distance_index].index;;
		//FIXME: increment ref counter
// 		reference_counters[point_index]++;
	}

	dout << "reproject new points to objectpoints" << std::endl;
	//FIXME: idealpoints_to_objectpoints with mask
	std::vector< cv::Point3_<T> > new_points;
	idealpoints_to_objectpoints<rotation_matrix_quatvec>(
		projections,
		average_depth,
		camera_pose,
		new_points);

	dout << "put remaining points as new points to db" << std::endl;
#if _HUB_MAP_VERBOSE
	unsigned int new_counter = 0;
#endif
	// assume remaining points as new points
	for(unsigned int i=0; i<projections.size(); i++) {
		if(point_indices[i] != 0) continue;

		//add new point to map
		point_indices[i] = add_point( new_points[i], descriptors.row(i) );
#if _HUB_MAP_VERBOSE
		new_counter++;
#endif
	}
#if _HUB_MAP_VERBOSE
	dout << "added " << new_counter << " new points of " << projections.size() << " total projections to db" << std::endl;
#endif

	//add scenery
	hub::Lock scene_lock(scene_mutex);
	scenes.push_back( scene_t(camera_pose, view_point, projections, point_indices) );
	dout << "added scenery" << std::endl;

	local_bundle_adjust();

	// task finished
	working_queue.pop_front();
	dout << "finished task" << std::endl;
}

template<typename T>
void Map<T>::run() {
	useconds_t sleep_time_us = 100000;

	while( !interrupted() ) {
		dout << "work" << std::endl;
		process_working_task();

		if( working_queue.empty() ) // reduce speed by 10%
			sleep_time_us += sleep_time_us/10;
		else // increase speed by factor 2
			sleep_time_us /= 2;
		dout << "sleep " << sleep_time_us << "us" << std::endl;
		usleep(sleep_time_us);
	}
}

template<typename T>
int Map<T>::save_points(const std::string &file_name) const {
	if( objectpoints.empty() )
		return 1;

	std::ofstream f_stream( file_name.c_str() );
	if( !f_stream.is_open() )
		return -1;

	// insert header
	char *vertex_string;
	if( asprintf(&vertex_string, ply_vertex_element, objectpoints.size() ) < 0)
		return -2;
	f_stream << ply_header_start << vertex_string << ply_header_end;
	free(vertex_string);

	std::vector<uint8_t> saved_point( objectpoints.size(), 0);
	cv::Mat color_map = (cv::Mat_<uint8_t>(7,3) << 255, 0, 0,
		0, 255, 0,
		0, 0, 255,
		255, 255, 0,
		255, 0, 255,
		0, 255, 255,
		127, 127, 127);
	uint8_t color_index = 0;
	for(typename std::list<scene_t>::const_iterator scene_iter = scenes.begin();
	scene_iter != scenes.end();
	++scene_iter) {
		for(unsigned int i=0; i<scene_iter->point_indices.size(); i++) {
			const uint16_t point_index = scene_iter->point_indices[i];
			if( saved_point[point_index] != 0) // point alread saved
				continue;
			f_stream << objectpoints[point_index].x << " "
				<< objectpoints[point_index].y << " "
				<< objectpoints[point_index].z << " "
				<< (int)color_map.at<uint8_t>(color_index, 0) << " "
				<< (int)color_map.at<uint8_t>(color_index, 1) << " "
				<< (int)color_map.at<uint8_t>(color_index, 2) << std::endl;
			saved_point[point_index] = 1;
		}

		//take next color
		color_index++; color_index %= color_map.rows;
	}

	return 0;
}

template<typename T>
int Map<T>::save_views(const std::string &file_name) const {
	if( scenes.empty() )
		return 1;

	std::ofstream f_stream( file_name.c_str() );
	if( !f_stream.is_open() )
		return -1;

	// insert header
	f_stream << ply_header_start;
	char *element_string;
	if( asprintf(&element_string, ply_vertex_element, 2*scenes.size() ) < 0)
		return -2;
	f_stream << element_string;
	free(element_string);
// 	if( asprintf(&element_string, ply_face_element, scenes.size() ) < 0)
// 		return -3;
// 	f_stream << element_string;
// 	free(element_string);
	f_stream << ply_header_end;


	// add views
	for(typename std::list<scene_t>::const_iterator scene_iter = scenes.begin();
	scene_iter != scenes.end();
	++scene_iter) {
		f_stream << scene_iter->camera_pose[3] << " "
			<< scene_iter->camera_pose[4] << " "
			<< scene_iter->camera_pose[5] << " 0 0 0" << std::endl;
		f_stream << scene_iter->view_point[0] << " "
			<< scene_iter->view_point[1] << " "
			<< scene_iter->view_point[2] << " 255 255 255" << std::endl;
	}

	// add optical axis
// 	for(unsigned int i=0; i<scenes.size(); i++) {
// 		f_stream << "2 " << 2*i << " " << 2*i+1 << std::endl;
// 	}

	return 0;
}

template<typename T>
void Map<T>::set_camera(const cv::Mat& camera_matrix, const int width, const int height) {
	hub::Lock cam_lock(cam_mutex);

// 	const T cx = camera_matrix.at<double>(0, 2);
// 	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	camera.ideal_width = (T)width/fx;
	camera.ideal_height = (T)height/fy;

	// use incircle of image
	const T ideal_size = std::min((T)width/fx, (T)height/fy);
// 	camera.fov_angle_rad = std::atan(ideal_size/2);
	camera.fov_radius = ideal_size/2;
	
// 	const T ideal_x_max = (T)(width-cx)/fx;
// 	const T ideal_y_max = (T)(height-cy)/fy;

// 	camera.radius_1m = std::min(ideal_x_max, ideal_y_max); //camera radius at 1cm
	// use circumcircle of image
// 	camera.radius_1m = std::sqrt(ideal_x_max*ideal_x_max + ideal_y_max*ideal_y_max); //camera radius at 1cm

// 	camera.angle = rad2deg( std::atan(camera.radius_1m) ); //angle of camera cone
// 	camera.radius_1m *= 100.0; //scale to 1m

	dout << "configured camera with dimension (" << camera.ideal_width << ", " << camera.ideal_height << ") and fov radius " << camera.fov_radius << std::endl;
}

template<typename T>
void Map<T>::set_view(const std::vector<T>& parameter_vector) {
	if(parameter_vector.size() < 6) return;

	hub::Lock cam_lock(cam_mutex);
	camera.pose = parameter_vector;
}

template<typename T>
void Map<T>::visible_points(const std::vector<T> &pose,
	std::vector< cv::Point3_<T> > &objectpoints,
	std::vector<cv::Point2f> &projections,
	cv::Mat &descriptors,
	std::vector<uint16_t> &map_indices) {

	if( Map<T>::objectpoints.empty() ) {
		objectpoints.clear();
		projections.clear();
		descriptors.resize(0);
		map_indices.clear();
		return;
	}

	std::vector<cv::Point2f> idealpoints( Map<T>::objectpoints.size() );
	ideal_pinhole_model<T, rotation_matrix_quatvec>(&Map<T>::objectpoints[0].x,
		&pose[0],
		&idealpoints[0].x,
		Map<T>::objectpoints.size() );

	const T x_max = camera.ideal_width/2, x_min = -x_max;
	const T y_max = camera.ideal_height/2, y_min = -y_max;

	objectpoints.clear(); objectpoints.reserve( Map<T>::objectpoints.size() );
	projections.clear(); projections.reserve( Map<T>::objectpoints.size() );
	descriptors.create(0, Map<T>::descriptors.cols, Map<T>::descriptors.type()); descriptors.reserve( Map<T>::descriptors.rows );
	map_indices.clear(); map_indices.reserve( Map<T>::objectpoints.size() );

	for(unsigned int i=0; i<Map<T>::objectpoints.size(); i++) {
		//FIXME: is abs(ideal_point.x) <= x_max faster?
		if( in_range(idealpoints[i].x, x_min, x_max)
		&& in_range(idealpoints[i].y, y_min, y_max) ) {
			objectpoints.push_back( Map<T>::objectpoints[i] );
			projections.push_back( idealpoints[i] );
			descriptors.push_back( Map<T>::descriptors.row(i) );
			map_indices.push_back(i);
		}
	}
	dout << "filled " << objectpoints.size() << " visible object points" << std::endl;
}

} // namespace slam
} // namespace hub

#undef dout

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_MAP_H_
