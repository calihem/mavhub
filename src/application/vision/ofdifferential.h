#ifndef _OFDIFFERENTIAL_H_
#define _OFDIFFERENTIAL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>
#if CV_MINOR_VERSION >= 2
#include "ofmodel.h"

class FirstOrder : public OFModel {
	public:
		FirstOrder(int height, int width);
		virtual ~FirstOrder();
		virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);

	private:
		int height;
		int width;
		IplImage *lastImage;
};

class HornSchunck : public OFModel {
	public:
		HornSchunck(int height, int width);
		virtual ~HornSchunck();
		virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);

	private:
		int height;
		int width;
		IplImage *lastImage;
};

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif
