#ifndef _OPTICALFLOW_H_
#define _OPTICALFLOW_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <stdint.h>
#include <list>

#ifdef HAVE_OPENCV2
#include "opencv/cv.h"
//#include "opencv2/opencv.hpp"

#if CV_MINOR_VERSION >= 2
#include "oftypes.h"

enum of_algorithm {
	UNKNOWN,
	FIRST_ORDER, // 1
	HORN_SCHUNCK,
	CENSUS_TRANSFORM,
	LINE_SUM,
	LINE_CENSUS,
	LK, //  6
	LK_PYR,
};

struct UnwrapSettings {
	int cx;		// x-coordinate of center point
	int cy;		// y-coordinate of center point
	int ri;		// inner radius
	int ro;		// outer radius
	int im;		// interpolation mode
	double sx;	// scaling in x-direction
	double sy;	// scaling in y-direction
	int fw;		// fixed width 
	int fh;		// fixed height 
	UnwrapSettings(int cx, int cy, int ri, int ro, int im, double sx, double sy, int fw, int fh);
};

/// Enumeration of camera types
enum cam_type_t {
	CAM_TYPE_PLANAR, /// planar camera image
	CAM_TYPE_OMNI	/// Omnidirectional camera image
};

/// type representing sparse motion 
// typedef std::list<Displacement> motion_list;

/// abstract base class representing optical flow
class OpticalFlow {
	public:
		virtual void setVelocity(int x, int y, int dx = 0, int dy = 0) = 0;
		virtual void clear() = 0;
		virtual int getMeanVelX(int x0, int x1, int y0, int y1) const = 0;
		virtual int getMeanVelY(int x0, int x1, int y0, int y1) const = 0;
		/* virtual CvMat* getVelXf(); */
		/* virtual CvMat* getVelYf(); */
		virtual void visualize(cv::Mat &image) const = 0;
		virtual void visualizeMean(int sectors, cv::Mat &image) const; 
		virtual void visualizeMeanXY(int sectorsx, int sectorsy, cv::Mat &image) const; 

	protected:
		of_algorithm algo;

};

/* inline CvMat* getVelXf() { */
/* 	return NULL; */
/* } */
/* inline CvMat* getVelYf() { */
/* 	return NULL; */
/* } */

/// class representing dense optical flow
class DenseOpticalFlow : public OpticalFlow {
	public:
		DenseOpticalFlow(int rows, int cols);
		virtual ~DenseOpticalFlow();
		virtual void setVelocity(int x, int y, int dx = 0, int dy = 0);
		virtual void clear();
		virtual int getMeanVelX(int x0, int x1, int y0, int y1) const;
		virtual int getMeanVelY(int x0, int x1, int y0, int y1) const;
		virtual CvMat* getVelXf();
		virtual CvMat* getVelYf();
		virtual float getMeanVelXf(int x0, int x1, int y0, int y1) const;
		virtual float getMeanVelYf(int x0, int x1, int y0, int y1) const;
		virtual void visualizeMeanXYf(int sectorsx, int sectorsy, cv::Mat &image) const;
		virtual void visualize(cv::Mat &image) const;
		virtual int getDirection(cv::Mat &image) const;

	protected:
		CvMat *velX, *velY;
		CvMat *velXf, *velYf;
};

inline DenseOpticalFlow::DenseOpticalFlow(int rows, int cols) {
	velX = cvCreateMat(rows, cols, CV_8SC1);
	velY = cvCreateMat(rows, cols, CV_8SC1);
	velXf = cvCreateMat(rows, cols, CV_32F);
	velYf = cvCreateMat(rows, cols, CV_32F);
}
inline void DenseOpticalFlow::setVelocity(int x, int y, int dx, int dy) {
	*( (signed char*)CV_MAT_ELEM_PTR(*(velX), y, x) ) = dx;
	*( (signed char*)CV_MAT_ELEM_PTR(*(velY), y, x) ) = dy;
}
inline void DenseOpticalFlow::clear() {
// 	TODO
}
inline CvMat* DenseOpticalFlow::getVelXf() {
	return(velXf);
}
inline CvMat* DenseOpticalFlow::getVelYf() {
	return (velYf);
}

/// class representing sparse optical flow
class SparseOpticalFlow : public OpticalFlow {
	public:
		SparseOpticalFlow();
		virtual ~SparseOpticalFlow();
		virtual void setVelocity(int x, int y, int dx = 0, int dy = 0);
		virtual void clear();
		virtual int getMeanVelX(int x0, int x1, int y0, int y1) const;
		virtual int getMeanVelY(int x0, int x1, int y0, int y1) const;
		virtual void visualize(cv::Mat &image) const;

	protected:
		std::list<Displacement> velocity;

};
inline SparseOpticalFlow::SparseOpticalFlow() {
}
inline void SparseOpticalFlow::setVelocity(int x, int y, int dx, int dy) {
	velocity.push_back( Displacement(x, y, dx, dy) );
}
inline void SparseOpticalFlow::clear() {
	velocity.clear();
}

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif
