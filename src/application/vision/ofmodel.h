#ifndef _OFMODEL_H_
#define _OFMODEL_H_

#include <stdint.h>
//#include <cv.h>

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

#endif
