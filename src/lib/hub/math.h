#ifndef _HUB_MATH_H_
#define _HUB_MATH_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <cmath>	//pow
#include <cstdlib>	//abs
#include <algorithm>	//nth_element

namespace hub {

const double pi = 3.1415926535897932384626433832795028841971693993751058209749;

/**
 * \brief Convert degree to radian.
 */
template <typename T>
T deg2rad(const T d);

/**
 * Check if value val is in range (low <= x <= high)
 *
 * @param value x
 * @param low range limit
 * @param high range limit
 * @return true (1) if x is in specified range
 */
template <typename T>
int in_range(const T &x, const T &low, const T &high);

/**
 * \brief Calculates the intersection of a line and a plane.
 * \param[in] plane_point
 * \param[in] plane_normal_vector
 * \param[in] line_point
 * \param[in] line_direction
 * \param[out] intersection 3D intersection point of given line and plane.
 * \return 0 if line intersects plane, otherwise -1.
 */
template <typename T>
int intersection(const T plane_point[3],
	const T plane_normal_vector[3],
	const T line_point[3],
	const T line_direction[3],
	T intersection[3]);

/**
 * \brief Calculates MAD (Median Absolute Deviation).
 * \param[in] values Data vector.
 * \sa _mad(std::vector<T> &values)
 */
template <typename T>
T mad(std::vector<T> values);

template <typename T>
T mad(std::vector<T> values, T &median);

/**
 * Calculate MAD (Median Absolute Deviation) without copy constructor.
 * \param[in] values Data vector.
 * \warning The input vector gets altered.
 * \sa mad(std::vector<T> values);
 */
template <typename T>
T _mad(std::vector<T> &values);

template <typename T>
T _mad(std::vector<T> &values, T &median);

template <typename T>
T mean(const std::vector<T> &values);

/**
 * \brief Find median in linear time.
 * Determine the median of the input vector. Keep in mind the cost of the
 * copy constructor which is needed to make this function safe to use.
 * \param[in] values Data vector.
 * \sa _median(std::vector<T> &values);
 */
template <typename T>
T median(std::vector<T> values);

/**
 * \brief Find median in linear time without copy constructor.
 * \param[in] values Data vector.
 * \sa median(std::vector<T> values)
 * \warning The input vector gets altered.
 */
template <typename T>
T _median(std::vector<T> &values);

/**
 * \brief Convert radian to degree.
 */
template <typename T>
T rad2deg(const T r);

/**
 * \brief Estimate deviation using MAD.
 * Estimate the deviation of the input vector using MAD which 
 * is known to be robust.
 * \param[in] values Data vector.
 * \sa _robust_sigma(std::vector<T> &values)
 */
template<typename T>
T robust_sigma(const std::vector<T> &values);

template<typename T>
T robust_sigma(const std::vector<T> &values, T &median);

/**
 * \brief Estimate deviation using MAD.
 * \param[in] values Data vector.
 * \sa robust_sigma(const std::vector<T> &values)
 * \warning The input vector gets altered.
 */
template<typename T>
T _robust_sigma(std::vector<T> &values);

template<typename T>
T _robust_sigma(std::vector<T> &values, T &median);

template <typename T>
T std_dev(const std::vector<T> &values);

/**
 * \brief Calculate weight of Tukey M-estimator.
 * \param[in] x Data value (residual)
 * \param[in] sigma Deviation of the set \arg x is belonging to.
 * \return Weight
 */
template<typename T>
T tukey_weight(const T x, const T sigma = 1.0);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
template <typename T>
T deg2rad(const T d) {
	return (d*pi) / 180;
}

template <typename T>
inline int in_range(const T &x, const T &low, const T &high) {
	return x >= low && x <= high;
}

template <typename T>
int intersection(const T plane_point[3],
	const T plane_normal_vector[3],
	const T line_point[3],
	const T line_direction[3],
	T intersection[3]) {

	T distance = line_direction[0]*plane_normal_vector[0]
		+ line_direction[1]*plane_normal_vector[1]
		+ line_direction[2]*plane_normal_vector[2];

	if(distance == 0) { // denominator is zero
		distance = (plane_point[0]-line_point[0])*plane_normal_vector[0]
			+ (plane_point[1]-line_point[1])*plane_normal_vector[1]
			+ (plane_point[2]-line_point[2])*plane_normal_vector[2];

		if(distance != 0) {// numerator is zero
			//line is is outsite and parallel to plane
			return -1;
		}

		// line lies in plane
		intersection[0] = line_point[0];
		intersection[1] = line_point[1];
		intersection[2] = line_point[2];

		return 0;
	}
	
	distance = ( (plane_point[0]-line_point[0])*plane_normal_vector[0]
		+ (plane_point[1]-line_point[1])*plane_normal_vector[1]
		+ (plane_point[2]-line_point[2])*plane_normal_vector[2] )
		/ distance;

	intersection[0] = line_point[0] + distance*line_direction[0];
	intersection[1] = line_point[1] + distance*line_direction[1];
	intersection[2] = line_point[2] + distance*line_direction[2];
	
	return 0;
}

template <typename T>
T mad(std::vector<T> values) {
	return _mad(values);
}

template <typename T>
T mad(std::vector<T> values, T &median) {
	return _mad(values, median);
}

template <typename T>
T _mad(std::vector<T> &values) {
	T med = _median(values);
	for(typename std::vector<T>::iterator iter = values.begin(); iter != values.end(); ++iter) {
		*iter = std::abs(*iter - med);
	}
	return _median(values);
}

template <typename T>
T _mad(std::vector<T> &values, T &median) {
	median = _median(values);
	for(typename std::vector<T>::iterator iter = values.begin(); iter != values.end(); ++iter) {
		*iter = std::abs(*iter - median);
	}
	return _median(values);
}

template <typename T>
T mean(const std::vector<T> &values) {
	if(values.size() == 0) return 0;

	T sum = 0;
	for(typename std::vector<T>::const_iterator i=values.begin(); i != values.end(); ++i) {
		sum += *i;
	}
	return sum / values.size();
}

template <typename T>
T median(std::vector<T> values) {
	return _median(values);
}

template <typename T>
T _median(std::vector<T> &values) {
	typename std::vector<T>::iterator first = values.begin();
	typename std::vector<T>::iterator last = values.end();
	typename std::vector<T>::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last);
	return *middle;
}

template <typename T>
inline T rad2deg(const T r) {
	return (r*180) / pi;
}

template<typename T>
inline T robust_sigma(const std::vector<T> &values) {
	const T sigma = 1.4862 * mad(values);
	return sigma;
}

template<typename T>
inline T robust_sigma(const std::vector<T> &values, T &median) {
	const T sigma = 1.4862 * mad(values, median);
	return sigma;
}

template<typename T>
inline T _robust_sigma(std::vector<T> &values) {
        T sigma = 1.4862 * _mad(values);
        return sigma;
}

template<typename T>
inline T _robust_sigma(std::vector<T> &values, T &median) {
        T sigma = 1.4862 * _mad(values, median);
        return sigma;
}

template <typename T>
T std_dev(const std::vector<T> &values) {
	if(values.size() <= 1) return 0;

	T m = mean(values);
	T sum = 0;
	for(typename std::vector<T>::const_iterator i=values.begin(); i != values.end(); ++i) {
		sum += std::pow(*i-m, 2);
	}
	return sqrt( sum/(values.size()-1) );
}

template<typename T>
inline T tukey_weight(const T x, const T sigma) {
        const double c = sigma*4.6851;

        if(std::abs(x) > c)
                return  0.0;

        const T tmp = 1 - (x*x)/(c*c);
        return tmp*tmp;
}

} // namespace hub

#endif // _HUB_MATH_H_
