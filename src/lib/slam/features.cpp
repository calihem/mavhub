#include "features.h"

#include <iostream>     // cout
#include <iomanip>	//setprecision

#define min_eigenval(dxx, dxy, dyy) 0.5 * (dxx + dyy - sqrt( (dxx + dyy) * (dxx + dyy) - 4 * (dxx * dyy - dxy * dxy) ))

#ifdef HAVE_OPENCV_CV_H

template <typename T>
T shi_tomasi_score(const cv::Mat &image, const int x, const int y, const int box_radius) {
	//TODO: optional range check
	//check coordinate range
	if(x <= box_radius || x >= image.cols-box_radius-1
	|| y <= box_radius || y >= image.rows-box_radius-1)
		return 0.0;

	int32_t dx, dy;
	int32_t dxx = 0, dxy = 0, dyy = 0;
	//iterate through box (window)
	for(int i=y-box_radius; i <= y+box_radius; i++) {
		for(int j=x-box_radius; j <= x+box_radius; j++) {
			dx = image.at<uint8_t>(i, j+1) - image.at<uint8_t>(i, j-1);
			dy = image.at<uint8_t>(i+1, j) - image.at<uint8_t>(i-1, j);
			dxx += dx*dx; dxy += dx*dy; dyy += dy*dy;
		}
	}

	//calculate minimal eigenvalue
	T t_dxx = static_cast<T>(dxx);
	T t_dxy = static_cast<T>(dxy);
	T t_dyy = static_cast<T>(dyy);
	T min_lambda = min_eigenval(t_dxx, t_dxy, t_dyy);

	//thus we have used unscaled version to calculate derivatives, we have to do it now
	int num_pixels = (2*box_radius+1)*(2*box_radius+1);
	return min_lambda / (4.0*num_pixels);
}

cv::Point2f transform_affine(const cv::Point2f &point, const cv::Mat &transform_matrix) {
	if(transform_matrix.rows != 2 || transform_matrix.cols != 3)
		return point;

	cv::Point2f transformed;
	
	const double *matrix_data = reinterpret_cast<const double*>(transform_matrix.data);
	transformed.x = matrix_data[0]*point.x + matrix_data[1]*point.y + matrix_data[2];
	transformed.y = matrix_data[3]*point.x + matrix_data[4]*point.y + matrix_data[5];
	
	return transformed;
}

#endif // HAVE_OPENCV_CV_H