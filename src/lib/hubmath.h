#ifndef _HUB_MATH_H_
#define _HUB_MATH_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <cstdlib>	//abs
#include <algorithm>	//nth_element

namespace hub {

/**
 * Calculate MAD (Median Absolute Deviation)
 */
template <typename T>
T mad(std::vector<T> values);

/**
 * Calculate MAD (Median Absolute Deviation) without copy constructor.
 * \warning The input vector gets altered.
 */
template <typename T>
T cl_mad(std::vector<T> values);

/**
 * Find median in linear time using copy constructor.
 */
template <typename T>
T median(std::vector<T> values);

/**
 * Find median in linear time without copy constructor.
 * \warning The input vector gets altered.
 */
template <typename T>
T cl_median(std::vector<T> &values);

/**
 * Estimate deviation using MAD.
 */
template<typename T>
T robust_sigma(const std::vector<T> &values);

/**
 * Estimate deviation using MAD.
 * \warning The input vector gets altered.
 */
template<typename T>
T cl_robust_sigma(std::vector<T> &values);

/**
 * Calculate weight of Tukey M-estimator.
 */
template<typename T>
T tukey_weight(const T x, const T sigma = 1.0);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
template <typename T>
T mad(std::vector<T> values) {
	return cl_mad(values);
}

template <typename T>
T cl_mad(std::vector<T> values) {
	T med = cl_median(values);
	for(typename std::vector<T>::iterator iter = values.begin(); iter != values.end(); ++iter) {
		*iter = std::abs(*iter - med);
	}
	return cl_median(values);
}

template <typename T>
T median(std::vector<T> values) {
	return cl_median(values);
}

template <typename T>
T cl_median(std::vector<T> &values) {
	typename std::vector<T>::iterator first = values.begin();
	typename std::vector<T>::iterator last = values.end();
	typename std::vector<T>::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last);
	return *middle;
}

template<typename T>
inline T robust_sigma(const std::vector<T> &values) {
	const T sigma = 1.4862 * mad(values);
	return sigma;
}

template<typename T>
inline T cl_robust_sigma(std::vector<T> &values) {
        T sigma = 1.4862 * cl_mad(values);
        return sigma;
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
