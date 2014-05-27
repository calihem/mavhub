#ifndef _HUB_TRACKER_H_
#define _HUB_TRACKER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#if CV_MINOR_VERSION >= 2

#include <bitset>

//FIXME remove brisk dependency
#include <brisk/brisk.h>
#include "lib/slam/features.h"
#include "lib/slam/map.h"

namespace hub {
namespace slam {

//FIXME: make it more generic template class
class Tracker {
public:
	/**
	 * \brief Enumeration of possible debug values.
	 */
	enum debug_flags_t { POSE = 0,
		FEATURE_IMAGE = 1,
		MATCHES = 2
	};

	/**
	 * \brief Constructor
	 */
	Tracker(const int image_width,
		const int image_height,
		const cv::Mat &camera_matrix = cv::Mat(),
		const cv::Mat &distortion_coefficients = cv::Mat() );

	/**
	 * \brief Destructor
	 */	
	~Tracker();

	/**
	 * \brief Perform dead reckoning on IMU accels.
	 * \param[in] time_ms Timestamp in milliseconds of sensor data.
	 * \param[in] imu_data Quaternion vector part (inertial frame) + accelerometer (body frame) [q1 q2 q3 accel_x accel_y accel_z]
	 */
	int imu_update(const uint64_t &time_ms, const std::vector<float> &imu_data);

	/**
	 * \brief Get current pose estimation.
	 * \return 6D-Vector containing quaternion vector part and translation.
	 */
	const std::vector<float>& pose_estimation() const;

	/**
	 * \brief Set current pose estimation.
	 * \param[in] pose 6D-Vector containing quaternion vector part and translation.
	 */
	void pose_estimation(const std::vector<float>& pose);

	/**
	 * \brief Reset inner states of tracker.
	 */
	int reset();

	/**
	 * \brief Save map points (point cloud) and camera views as polygon files (ply).
	 */
	int save_map(const std::string &name) const;

	/**
	 * \brief Get pose of camera by identifying the image in the map.
	 * \param[in] image Camera image.
	 * \param[in,out] parameter_vector 6D-Vector containing quaternion vector part and translation. Input will be used as a first guess.
	 * \param[in] avg_depth Average distance between camera and objects.
	 */
	int track_camera(const cv::Mat &image, std::vector<float> &parameter_vector, const float avg_depth = 100.0, const std::bitset<8> &debug_mask = 0);

protected:

private:
	cv::Mat camera_matrix; ///< Camera matrix of intrinsic parameters.
	cv::Mat distortion_coefficients; ///< distortion coefficients of camera.

	cv::BriskFeatureDetector feature_detector; ///< BRISK feature detector using AGAST
	cv::BriskDescriptorExtractor descriptor_extractor;
#ifdef HAVE_SSSE3
	// see slam_app.h
	// cv::BruteForceMatcher<cv::HammingSse> matcher;
	cv::BruteForceMatcher<cv::Hamming> matcher;
#else
	cv::BruteForceMatcher<cv::Hamming> matcher;
#endif
	Map<> map;
	uint64_t last_imu_time; ///< Timestamp of last IMU data
	std::vector<float> speed; ///< Current speed estimation based on IMU data
	std::vector<float> pose; ///< Current pose estimation based on IMU and localization
	static uint16_t run_counter;	///< Counter which gets incremented with each call of track_camera

	int log_debug_data(const std::bitset<8> &debug_mask,
		const std::vector<float> &parameter_vector);
	int log_debug_data(const std::bitset<8> &debug_mask,
		const cv::Mat &image,
		const std::vector<float> &parameter_vector,
		const std::vector<cv::KeyPoint> &keypoints);
	int log_debug_data(const std::bitset<8> &debug_mask,
		const std::vector<float> &parameter_vector,
		const std::vector<cv::Point2f> &ideal_points,
		const std::vector<cv::Point2f> &op_projections,
		const std::vector<cv::DMatch> &matches,
		const std::vector<char> &matches_mask);
};

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
inline const std::vector<float>& Tracker::pose_estimation() const {
	return pose;
}

inline int Tracker::save_map(const std::string &name) const {
	map.save_points(name+"_map.ply");
	return map.save_views(name+"_view.ply");
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_TRACKER_H_
