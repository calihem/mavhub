#ifndef _HUB_MAP_H_
#define _HUB_MAP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#if CV_MINOR_VERSION >= 2

#include <list>

#include "lib/hub/thread.h"
#include "lib/hub/math.h"
#include "lib/slam/features.h"

namespace hub {
namespace slam {

template<typename T=float>
class Map : public cpp_pthread::PThread {

	/**
	 * \brief Struct describing camera.
	 */
	struct camera_t {
		camera_t() : camera_matrix(), width(0), height(0), angle(0), radius_1m(0), pose() {};
		camera_t(const cv::Mat &camera_matrix,
			const int width,
			const int height,
			const T angle,
			const T radius,
			const std::vector<T> &pose) :
				camera_matrix(camera_matrix),
				width(width),
				height(height),
				radius_1m(radius),
				pose(pose) { };

		cv::Mat camera_matrix; /// Matrix of intrinsic parameters.
		int width; /// Image width.
		int height; /// Image height.
		T angle; /// Camera angle between optical axis and points which are projected to one image corner
		T radius_1m; /// The radius of the circle a view cone would produce at distance of 1m from the camera.
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
	 * \param camera_pose Rotation and translation of camera.
	 * \param average_depth Average depth of image scenery.
	 */
	void add_features(const std::vector<cv::Point2f>& projections,
		const cv::Mat& descriptors,
		const std::vector<T>& camera_pose,
		const T average_depth = 100.0);
	/**
	 * \brief Get all points with descriptor from map you would see with the configured camera.
	 */
	void fill_tracking_points(std::vector< cv::Point3_<T> > &objectpoints, cv::Mat &descriptors);
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

	int add_point(const cv::Point3_<T>& point, const cv::Mat& descriptor);
	int check_viewpoint_distance(const std::vector<T> &view_point, const T max_distance) const;
	void process_working_task();
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

	if(projections.size() != descriptors.rows)
		return;
	if(camera_pose.size() != 6)
		return;

	queue_data_t working_task(projections, descriptors, camera_pose, average_depth);

	cpp_pthread::Lock work_lock(work_mutex);
	working_queue.push_back(working_task);
}

template<typename T>
int Map<T>::check_viewpoint_distance(const std::vector<T> &view_point, const T max_distance) const {
	cv::L2<T> euclidean_distance;
	cpp_pthread::Lock scene_lock(scene_mutex);

	for(typename std::list<scene_t>::const_iterator scene_iter = scenes.begin();
	scene_iter != scenes.end();
	++scene_iter) {
		const T distance = euclidean_distance(&view_point[0], &(scene_iter->view_point[0]), 3);
		if(distance <= max_distance) {
			return 0;
		}
	}

	return 1;
}

template<typename T>
void Map<T>::fill_tracking_points(std::vector< cv::Point3_<T> > &objectpoints, cv::Mat &descriptors) {
	
	cpp_pthread::Lock cam_lock(cam_mutex);

	std::vector<cv::Point2f> image_points;
	{ // map mutex scope
		cpp_pthread::Lock map_lock(map_mutex);
		objectpoints_to_idealpoints(Map<T>::objectpoints,
			camera.pose,
			image_points);
	}

	const cv::Mat &camera_matrix = camera.camera_matrix;
	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);
	const T x_min = -cx/fx, x_max = (camera.width -cx)/fx;
	const T y_min = -cy/fy, y_max = (camera.height -cy)/fy;

	objectpoints.clear(); objectpoints.reserve( Map<T>::objectpoints.size() );
	descriptors.create(0, Map<T>::descriptors.cols, Map<T>::descriptors.type()); descriptors.reserve( Map<T>::descriptors.rows );

	for(unsigned int i=0; i<Map<T>::objectpoints.size(); i++) {
		if( in_range(image_points[i].x, x_min, x_max)
		&& in_range(image_points[i].y, y_min, y_max) ) {
			objectpoints.push_back( Map<T>::objectpoints[i] );
			descriptors.push_back( Map<T>::descriptors.row(i) );
		}
	}
}

template<typename T>
void Map<T>::process_working_task() {
	if( working_queue.empty() )
		return;

	cpp_pthread::Lock work_lock(work_mutex);

	//calculate intersection of optical axis with plane parallel to xy-plane and going through (0,0,100)
	const std::vector<T>& camera_pose = working_queue.front().pose;
	const T z_axis[3] = {0.0, 0.0, 1.0};
	const T plane_point[3] = {0.0, 0.0, 100.0};
	std::vector<T> view_point(3);

	int rc = intersection<T>(plane_point,
		z_axis,
		&camera_pose[3],
		&camera_pose[0],
		&view_point[0]);
	if(rc < 0) { //no intersection
		//throw task away
		working_queue.pop_front();
		return;
	}

	const std::vector<cv::Point2f>& projections = working_queue.front().projections;
	const cv::Mat& descriptors = working_queue.front().descriptors;
	const T average_depth = working_queue.front().avg_depth;

	cpp_pthread::Lock map_lock(map_mutex);
	if( objectpoints.empty() ) { // add all projections to map
		objectpoints.reserve( 8*projections.size() );
		reference_counters.assign(objectpoints.size(), 0);
		std::vector<uint16_t> indices; indices.reserve( projections.size() );
		// fill objectpoints
		for(unsigned int i=0; i<projections.size(); i++) {
			objectpoints[i] = cv::Point3_<T>(projections[i].x*average_depth,
				projections[i].y*average_depth,
				 average_depth);
			indices[i] = i;
		}

		// add first scene
		cpp_pthread::Lock scene_lock(scene_mutex);
		scenes.clear();
		scenes.push_back( scene_t(camera_pose, view_point, projections, indices) );

		return;
	}

	T view_radius;
	{
		cpp_pthread::Lock cam_lock(cam_mutex);
		view_radius = camera.radius_1m; //1m equals 100.0
	}
	rc = check_viewpoint_distance(view_point, view_radius);
	if(rc <= 0) { // we already have a near enough keyframe
		//throw task away
		working_queue.pop_front();
		return;
	}

	//project object points with given pose
	std::vector<cv::Point2f> projected_objectpoints;
	objectpoints_to_idealpoints(objectpoints, camera_pose, projected_objectpoints);

	//scenery data
	std::vector<uint16_t> point_indices( projections.size() );

	//for every image projection find best fitting map point
	std::vector<T> distances( projected_objectpoints.size() );
	for(unsigned int i=0; i<projections.size(); i++) {
		//calculate distances
		for(unsigned int j=0; i<projected_objectpoints.size(); j++) {
			const T delta_x = projections[i].x - projected_objectpoints[j].x;
			const T delta_y = projections[i].y - projected_objectpoints[j].y;
			distances[j] = delta_x*delta_x + delta_y*delta_y;
		}
		//get minimal distance
		typename std::vector<T>::iterator min_iter = std::min_element(distances.begin(), distances.end());

		//check distance
		uint16_t point_index;
		if(*min_iter <= 2.5) { // is in epsilon environment
			//point already in db => only add projection
			point_index = min_iter - distances.begin();
		} else { //not in epsilon environment
			//add new point to map
			point_index = add_point( cv::Point3_<T>(projections[i].x*average_depth, projections[i].y*average_depth, average_depth),
				descriptors.row(i) );
		}
		point_indices[i] = point_index;
		reference_counters[point_index]++;
	}
	
	//add scenery
	cpp_pthread::Lock scene_lock(scene_mutex);
	scenes.push_back( scene_t(camera_pose, view_point, projections, point_indices) );
}

template<typename T>
void Map<T>::run() {
	while( !interrupted() ) {
		process_working_task();
	}
}

template<typename T>
void Map<T>::set_camera(const cv::Mat& camera_matrix, const int width, const int height) {
	cpp_pthread::Lock cam_lock(cam_mutex);
	camera.camera_matrix = camera_matrix;
	camera.width = width;
	camera.height = height;

	const T cx = camera_matrix.at<double>(0, 2);
	const T cy = camera_matrix.at<double>(1, 2);
	const T fx = camera_matrix.at<double>(0, 0);
	const T fy = camera_matrix.at<double>(1, 1);

	const T tmp1 = (width-cx)/fx;
	const T tmp2 = (height-cy)/fy;
	
	camera.radius_1m = std::sqrt(tmp1*tmp1 + tmp2*tmp2); //camera radius at 1cm
	camera.angle = rad2deg( std::atan(camera.radius_1m) ); //angle of camera cone
	camera.radius_1m *= 100.0; //scale to 1m
}

template<typename T>
void Map<T>::set_view(const std::vector<T>& parameter_vector) {
	cpp_pthread::Lock cam_lock(cam_mutex);
	camera.pose = parameter_vector;
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_MAP_H_
