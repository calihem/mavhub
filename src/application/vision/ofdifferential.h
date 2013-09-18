#ifndef _OFDIFFERENTIAL_H_
#define _OFDIFFERENTIAL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#if CV_MINOR_VERSION >= 2
#include "ofmodel.h"

#include "toolbox.h"

class FirstOrder : public OFModel {
 public:
  FirstOrder(int height, int width);
  virtual ~FirstOrder();
  virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);
  const OpticalFlow &calcOpticalFlow2(cv::Mat &image);

 private:
  int height;
  int width;
  IplImage *lastImage;
  cv::Mat lastImageM;
};

class HornSchunck : public OFModel {
 public:
  HornSchunck(int height, int width);
  virtual ~HornSchunck();
  virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);
  const OpticalFlow &calcOpticalFlow2(cv::Mat &image);

 private:
  int height;
  int width;
  IplImage *lastImage;
};

class HornSchunckCV : public OFModel {
 public:
  HornSchunckCV(int height, int width);
  virtual ~HornSchunckCV();
  virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);
  const OpticalFlow &calcOpticalFlow2(cv::Mat &image);

 private:
  int height;
  int width;
  IplImage *lastImage;
};

class BlockMatchingCV : public OFModel {
 public:
  BlockMatchingCV(int height, int width);
  virtual ~BlockMatchingCV();
  virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);
  const OpticalFlow &calcOpticalFlow2(cv::Mat &image);

 private:
  int height;
  int width;
  IplImage *lastImage;
};

class LucasKanade : public OFModel {
 public:
  LucasKanade(int height, int width);
  virtual ~LucasKanade();
  virtual const OpticalFlow &calcOpticalFlow(const IplImage &image);
  const OpticalFlow &calcOpticalFlow2(cv::Mat &image);

 private:
  int height;
  int width;
  IplImage *lastImage;
};

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif
