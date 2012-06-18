#include "ofmodel.h"

#ifdef HAVE_OPENCV2

#include "OpticalFlow.h"

OFModel::OFModel() : oFlow(0) {
}

OFModel::~OFModel() {
}
#endif // HAVE_OPENCV2
