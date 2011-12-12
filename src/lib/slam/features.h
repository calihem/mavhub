#ifndef _HUB_FEATURES_H_
#define _HUB_FEATURES_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_OPENCV2

#include <opencv/cv.h>	//cv::Mat

#if (CV_MINOR_VERSION > 1)

/**
 * \brief Calculate the Shi-Tomasi score.
 *
 * Computes the minimal eigenvalue of the Hessian matrix. A
 * larger minimal eigenvalue means a better feature. For
 * further informations have a look at Harris corner
 * detector. 
 * \param image The image to analyse
 * \param x x-component of image point
 * \param y y-component of image point
 * \param box_radius The radius of the box building the subimage for
 * which the derivatives are calculated. A value of 1 should be fine.
 * \return Minimal eigenvalue. A good threshold for detecting good
 * features should be in the range from 40 to 150.
 */
template <typename T>
T shi_tomasi_score(const cv::Mat &image, const int x, const int y, const int box_radius);

#endif // CV_MINOR_VERSION > 1
#endif // HAVE_OPENCV2
#endif // HUB_FEATURES
