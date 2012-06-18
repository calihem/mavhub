#ifndef _OFTYPES_H_
#define _OFTYPES_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <stdint.h>
#include <list>

#ifdef HAVE_OPENCV2
/* #include <opencv2/core/core.hpp> */
/* #include <opencv2/highgui/highgui.hpp> */
#include <opencv/cv.h>
//#include "OpenCV.h"

//FIXME: image_t -> Image
struct image_t {
	IplImage *data;			//image data
	char *name;			//filename
	timeval time;			//time when image was captured
	CvPoint poi;			//coordinates of point of interest
};

struct image_of {
    IplImage *data;
    float alt;
    float rot;
    float x;
    float y;
};

/// class representing the displacement of a point
class Displacement : public CvPoint {
	public:
		Displacement(int x, int y, int dx=0, int dy=0);
		int dx;
		int dy;
		double length() const;

	private:
		mutable double len;
};
inline Displacement::Displacement(int x, int y, int dx, int dy) : dx(dx), dy(dy), len(-1) {
	Displacement::x = x; Displacement::y = y;
}
inline double Displacement::length() const {
	if(len<0)
		len = sqrt( (dx*dx)+(dy*dy) );
	return len;
}

/*struct oflow_t {
	CvMat *uMat, *vMat;		//x,y components of optical flow
	int8_t u[10], v[10];		//optical flow for every nonant
					//of image (lower resolution)
					//u[0], v[0] is of for whole image
	of_algorithm algo;		//used optical flow algorithm
};*/

struct motion_t {
	int16_t dx, dy, dz;	//movement in x,y and z direction
	uint16_t angle;		//yaw angle
};

#endif // HAVE_OPENCV2
#endif
