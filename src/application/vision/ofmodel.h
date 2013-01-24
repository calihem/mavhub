#ifndef _OFMODEL_H_
#define _OFMODEL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <stdint.h>

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>

#include "OpticalFlow.h"

class OpticalFlow;

class OFModel {
	public:
		OFModel();
		virtual ~OFModel();

		virtual const OpticalFlow &calcOpticalFlow(const IplImage &image) = 0;
		/// return last determined optical flow
		virtual const OpticalFlow &getOpticalFlow() const { return *oFlow; };

	protected:
		OpticalFlow *oFlow;
};

#endif // HAVE_OPENCV2
#endif
