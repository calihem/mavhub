#include "ofdifferential.h"

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)

#include "toolbox.h"
//#include "Settings.h"
#include <iostream>
// #include <stdio.h>
using namespace std;
using namespace cv;

FirstOrder::FirstOrder(int height, int width) : height(height), width(width), lastImage(0) {
  oFlow = new MHDenseOpticalFlow(height, width);
}

FirstOrder::~FirstOrder() {
  delete oFlow;
  if(lastImage)
    cvReleaseImage(&lastImage);
}

/// first order optical flow
const OpticalFlow &FirstOrder::calcOpticalFlow(const IplImage &image) {
  // const bool unwrapped = true;

  if(lastImage) {
    int step = image.widthStep;
    const uchar *last = (const uchar *)lastImage->imageData;
    const uchar *cur = (const uchar *)image.imageData;
		
    int8_t sgn_u, sgn_v;
    int16_t diff;
    float u, v;
    // uint32_t abs_grad_x, abs_grad_y, abs_grad_t, grad_magn;
    double abs_grad_x, abs_grad_y, abs_grad_t, grad_magn;
    register uint index, indexyp1, indexym1;
    index = indexyp1 = indexym1 = 0;

    u = v = 0.;

    for(int y=1;y<height-1;y++) {
      for(int x=1;x<width-1;x++) {

        index = y*step+x;
        indexyp1 = (y+1)*step+x;
        indexym1 = (y-1)*step+x;
        // if(unwrapped || (_ro*_ro*9/4>(x-_cx)*(x-_cx)+(y-_cy)*(y-_cy))
        // 	 ){ // Edit Lóa: Only calculate OF inside circle
        // I_t
        diff = cur[index] - last[index];
        abs_grad_t = ABS(diff);

        if(abs_grad_t) {
          // consider signum form I_t 
          sgn_u = SGN(diff); sgn_v = SGN(diff);
          //I_x
          diff = (cur[index+1] - cur[index-1]
                  + last[index+1] - last[index-1]);
          abs_grad_x = ABS(diff);
          sgn_u *= SGN(diff);
          //I_y
          diff = (cur[indexyp1] - cur[indexym1]
                  + last[indexyp1] - last[indexym1]);
          abs_grad_y = ABS(diff);
          sgn_v *= SGN(diff);

          if(abs_grad_x || abs_grad_y) {
            grad_magn = sqrt(abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y);
            // grad_magn = (abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y);
            // cout << "grad_magn = " << grad_magn << endl;
            // better would be || \nabla I || >= THRESHOLD
            if( grad_magn > 0) { // 6*32
              u = -sgn_u * ((abs_grad_x * abs_grad_t) / grad_magn);
              v = -sgn_v * ((abs_grad_y * abs_grad_t) / grad_magn);
              // oFlow->setVelocityf(x, y, u, v);
            }
            // else {//set zero
            //   oFlow->setVelocityf(x, y, 0, 0);
            // }
          }
          // else {//set zero
          //   oFlow->setVelocity(x, y, 0, 0);
          // }
        }
        // else {//set zero
        //   oFlow->setVelocity(x, y, 0, 0);
        // }
        oFlow->setVelocityf(x, y, u, v);

      }
    }
    // replace last image with current image
    cvCopy(&image, lastImage, NULL);
  } else {
    lastImage = cvCloneImage(&image);
  }
  return *oFlow;
}

/// first order optical flow
const OpticalFlow &FirstOrder::calcOpticalFlow2(Mat &image) {
  return calcOpticalFlow(image);
}

// /// first order optical flow
// const OpticalFlow &FirstOrder::calcOpticalFlow(const IplImage &image) {
//   // const bool unwrapped = true;

//   if(lastImage) {
//     int step = image.widthStep;
//     const uchar *last = (const uchar *)lastImage->imageData;
//     const uchar *cur = (const uchar *)image.imageData;
		
//     int8_t sgn_u, sgn_v;
//     int16_t diff;
//     uint32_t abs_grad_x, abs_grad_y, abs_grad_t, grad_magn;
//     register uint index, indexyp1, indexym1;
//     index = indexyp1 = indexym1 = 0;

//     for(int y=1;y<height-1;y++) {
//       for(int x=1;x<width-1;x++) {

//         index = y*step+x;
//         indexyp1 = (y+1)*step+x;
//         indexym1 = (y-1)*step+x;
//         // if(unwrapped || (_ro*_ro*9/4>(x-_cx)*(x-_cx)+(y-_cy)*(y-_cy))
//         // 	 ){ // Edit Lóa: Only calculate OF inside circle
//         // I_t
//         diff = cur[index] - last[index];
//         abs_grad_t = ABS(diff);

//         if(abs_grad_t) {
//           // consider signum form I_t 
//           sgn_u = SGN(diff); sgn_v = SGN(diff);
//           //I_x
//           diff = (cur[index+1] - cur[index-1]
//                   + last[index+1] - last[index-1]);
//           abs_grad_x = ABS(diff);
//           sgn_u *= SGN(diff);
//           //I_y
//           diff = (cur[indexyp1] - cur[indexym1]
//                   + last[indexyp1] - last[indexym1]);
//           abs_grad_y = ABS(diff);
//           sgn_v *= SGN(diff);

//           if(abs_grad_x || abs_grad_y) {
//             grad_magn = isqrt( (abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y) << 10);
//             // grad_magn = (abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y);
//             // cout << "grad_magn = " << grad_magn << endl;
//             // better would be || \nabla I || >= THRESHOLD
//             if( grad_magn > 0) { // 6*32
//               oFlow->setVelocity(x,
//                                  y,
//                                  -sgn_u * (((abs_grad_x * abs_grad_t) << 5) / grad_magn),
//                                  -sgn_v * (((abs_grad_y * abs_grad_t) << 5) / grad_magn)
//                                  );
//               // oFlow->setVelocity(x,
//               // 									 y,
//               // 									 -sgn_u * (((abs_grad_x * abs_grad_t) << 10) / grad_magn),
//               // 									 -sgn_v * (((abs_grad_y * abs_grad_t) << 10) / grad_magn)
//               // 									 );


//             } else {//set zero
//               oFlow->setVelocity(x, y, 0, 0);
//             }
//           } else {//set zero
//             oFlow->setVelocity(x, y, 0, 0);
//           }
//         } else {//set zero
//           oFlow->setVelocity(x, y, 0, 0);
//         }

//         /*				// better would be || \nabla I || >= THRESHOLD
//                                         if( grad_magn >= 1*256) {
//                                         cout << "dividend_u: " << ((abs_grad_x * abs_grad_t) << 8)
//                                         << ", dividend_v: " << ((abs_grad_y * abs_grad_t) << 8)
//                                         << ", grad_magn: " << grad_magn << endl;
//                                         oFlow->setVelocity(x,
//                                         y,
//                                         -sgn_u * (((abs_grad_x * abs_grad_t) << 8) / grad_magn),
//                                         -sgn_v * (((abs_grad_y * abs_grad_t) << 8) / grad_magn) );
//                                         } else {
//                                         oFlow->setVelocity(x, y, 0, 0);
//                                         }*/
//         //}
//       }
//     }
//     // cout << oFlow->getMeanVelX(0, 320, 0, 240) << endl;
//     // cout << oFlow->getMeanVelY(0, 320, 0, 240) << endl;
		
//     // 		oFlow->algo = FIRST_ORDER;

//     // replace last image with current image
//     cvCopy(&image, lastImage, NULL);

//   } else {
//     lastImage = cvCloneImage(&image);
//   }

//   return *oFlow;
// }

HornSchunck::HornSchunck(int height, int width) : height(height), width(width), lastImage(0) {
  oFlow = new MHDenseOpticalFlow(height, width);
}

HornSchunck::~HornSchunck() {
  delete oFlow;
  if(lastImage)
    cvReleaseImage(&lastImage);
}

const OpticalFlow &HornSchunck::calcOpticalFlow(const IplImage &image) {

  if(lastImage) {
    int step = image.widthStep;
    const uchar *last = (const uchar *)lastImage->imageData;
    const uchar *cur = (const uchar *)image.imageData;

    const uint8_t alphasquare = 1;
    int8_t sgn_u, sgn_v;
    int8_t u, v;
    int16_t diff;
    uint32_t abs_grad_x, abs_grad_y, abs_grad_t, divisor;

    for(int y=1;y<height-1;y++) {
      for(int x=1;x<width-1;x++) {
        diff = (cur[y*step+x+1] - cur[y*step+x-1]
                + last[y*step+x+1] - last[y*step+x-1]);
        abs_grad_x = ABS(diff) >> 2;
        sgn_u = SGN(diff);
				
        diff = (cur[(y+1)*step+x] - cur[(y-1)*step+x]
                + last[(y+1)*step+x] - last[(y-1)*step+x]);
        abs_grad_y = ABS(diff) >> 2;
        sgn_v = SGN(diff);
				
        diff = cur[y*step+x] - last[y*step+x];
        abs_grad_t = ABS(diff);
        sgn_u *= SGN(diff); sgn_v *= SGN(diff);

        divisor = (alphasquare + abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y) << 19;

        u = -sgn_u * (((abs_grad_x * abs_grad_t) << 19) / divisor);
        v = -sgn_v * (((abs_grad_y * abs_grad_t) << 19) / divisor);

        // cout << "u: " << (int)u << ", v: " << (int)v << endl;
        // oFlow->setVelocity(x, y, u, v);
        oFlow->setVelocityf(x, y, (float)u, (float)v);
      }
    }
		

    //replace last image with current image
    cvCopy(&image, lastImage, NULL);

  } else {
    lastImage = cvCloneImage(&image);
  }

  return *oFlow;
}

/// first order optical flow
const OpticalFlow &HornSchunck::calcOpticalFlow2(Mat &image) {
  return calcOpticalFlow(image);
}

////////////////////////////////////////////////////////////
// Horn-Schunck CV / OpenCV version
HornSchunckCV::HornSchunckCV(int height, int width) : height(height), width(width), lastImage(0) {
  oFlow = new MHDenseOpticalFlow(height, width);
}

HornSchunckCV::~HornSchunckCV() {
  delete oFlow;
  if(lastImage)
    cvReleaseImage(&lastImage);
}

// legacy
const OpticalFlow &HornSchunckCV::calcOpticalFlow(const IplImage &image) {
  // /* Calculates Optical flow for 2 images using Horn & Schunck algorithm */
  // CVAPI(void)  cvCalcOpticalFlowHS( const CvArr* prev, const CvArr* curr,
  //                                   int use_previous, CvArr* velx, CvArr* vely,
  //                                   double lambda, CvTermCriteria criteria );
  // CvMat *velx, *vely;
  CvMat *velx, *vely;
  CvTermCriteria termcrit = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 5, 1);

  velx = ((MHDenseOpticalFlow*)oFlow)->getVelXf();
  vely = ((MHDenseOpticalFlow*)oFlow)->getVelYf();

  if(lastImage) {
    // calc flow
    cvCalcOpticalFlowHS(lastImage, &image, 1, velx, vely, 10.0, termcrit);
    // replace last image with current image
    cvCopy(&image, lastImage, NULL);
  } else {
    lastImage = cvCloneImage(&image);
  }
  return *oFlow;
  return *oFlow;
}

/// first order optical flow
const OpticalFlow &HornSchunckCV::calcOpticalFlow2(Mat &image) {
  return calcOpticalFlow(image);
}
////////////////////////////////////////////////////////////
// BlockMatching CV / OpenCV version
BlockMatchingCV::BlockMatchingCV(int height, int width) : height(height), width(width), lastImage(0) {
  oFlow = new MHDenseOpticalFlow(height/10, width/10);
}

BlockMatchingCV::~BlockMatchingCV() {
  delete oFlow;
  if(lastImage)
    cvReleaseImage(&lastImage);
}

// legacy
const OpticalFlow &BlockMatchingCV::calcOpticalFlow(const IplImage &image) {
  // /* Calculates optical flow for 2 images using block matching algorithm */
  // CVAPI(void)  cvCalcOpticalFlowBM( const CvArr* prev, const CvArr* curr,
  //                                CvSize block_size, CvSize shift_size,
  //                                CvSize max_range, int use_previous,
  //                                CvArr* velx, CvArr* vely );
  CvMat *velx, *vely;

  velx = ((MHDenseOpticalFlow*)oFlow)->getVelXf();
  vely = ((MHDenseOpticalFlow*)oFlow)->getVelYf();

  cout << "velx sz: " << velx->cols << ", " << velx->rows << endl;
  cout << "vely sz: " << vely->cols << ", " << vely->rows << endl;

  if(lastImage) {
    cout << "last: " << lastImage->width << ", " << lastImage->height << endl;
    cout << "curr: " << image.width << ", " << image.height << endl;
    // calc flow
    cvCalcOpticalFlowBM(lastImage, &image, cvSize(10,10), cvSize(10,10), cvSize(100,100), 1, velx, vely);
    // replace last image with current image
    cvCopy(&image, lastImage, NULL);
  } else {
    lastImage = cvCloneImage(&image);
  }
  return *oFlow;
}

/// first order optical flow
const OpticalFlow &BlockMatchingCV::calcOpticalFlow2(Mat &image) {
  return calcOpticalFlow(image);
}

////////////////////////////////////////////////////////////
// LucasKanade
LucasKanade::LucasKanade(int height, int width) : height(height), width(width), lastImage(0) {
  oFlow = new MHDenseOpticalFlow(height, width);
}

LucasKanade::~LucasKanade() {
  delete oFlow;
  if(lastImage)
    cvReleaseImage(&lastImage);
}

const OpticalFlow &LucasKanade::calcOpticalFlow(const IplImage &image) {

  // IplImage* velx = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F, 1);
  // IplImage* vely = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F, 1);
  // cv::Mat velx = cv::Mat::zeros(cv::Size(width, height), CV_32F);
  // cv::Mat vely = cv::Mat::zeros(cv::Size(width, height), CV_32F);
  CvMat *velx, *vely;

  // double velocity_y = 0;
  // double velocity_x = 0;

  // double _x = 0;
  // double _y = 0;

  velx = ((MHDenseOpticalFlow*)oFlow)->getVelXf();
  vely = ((MHDenseOpticalFlow*)oFlow)->getVelYf();

  // cvSet(velx, cvScalarAll(0));
  // cvSet(vely, cvScalarAll(0));

  // cv::Mat m(velx);
  // // m.setTo(0.);
  // // cout << "velx:" << velx << endl;
  // cout << "m: " << m << endl;
  // // velx = (CvMat*) &m;

  if(lastImage) {
    cvCalcOpticalFlowLK(lastImage, &image, cvSize(5,5),
                        velx,
                        vely);

    // for(int i = 0; i < width; i++) {
    //   for(int j = 1; j < height - 1; j++) {
    //     double u = cvGetReal2D(velx,j,i);
    //     double v = cvGetReal2D(vely,j,i);
    //     double n = sqrt((double)(u*u + v*v));

    //     // cout << u << ", " << v << endl;
    //     if(n == 0) continue;
              
    //     double x = (double)(u) / n;
    //     double y = (double)(v) / n;
              
    //     _x += x;
    //     _y += y;
              
    //     velocity_x += u;
    //     velocity_y += v;
    //   }
    // }
          
    // double n = sqrt((double)(_x*_x + _y*_y));
    // if(n != 0) {
    //   _x /= n;
    //   _y /= n;
    // }
          
    // cout << "velX: " << velocity_x << endl;
    // cout << "velY: " << velocity_y << endl;
    // cout << "_x: " << _x << endl;
    // cout << "_y: " << _y << endl;

    //replace last image with current image
    cvCopy(&image, lastImage, NULL);

  } else {
    lastImage = cvCloneImage(&image);
  }

  // cvReleaseImage(&velx);
  // cvReleaseImage(&vely);
  // cv::ReleaseMat(&velx);
  // cv::ReleaseMat(&vely);
  return *oFlow;
}

/// first order optical flow
const OpticalFlow &LucasKanade::calcOpticalFlow2(Mat &image) {
  return *oFlow;
}

////////////////////////////////////////////////////////////
// Farneback OpenCV
Farneback::Farneback(int height, int width) : height(height), width(width), lastImage(0) {
  oFlow = new MHDenseOpticalFlow(height, width);
}

Farneback::~Farneback() {
  delete oFlow;
  if(lastImage)
    cvReleaseImage(&lastImage);
}

const OpticalFlow &Farneback::calcOpticalFlow(const IplImage &image) {
  CvMat *velx, *vely;
  Mat flow;
  Mat limg(lastImage);
  Mat img(&image);
  int i,j;
  Point p;

  velx = ((MHDenseOpticalFlow*)oFlow)->getVelXf();
  vely = ((MHDenseOpticalFlow*)oFlow)->getVelYf();


  if(lastImage) {
    calcOpticalFlowFarneback(limg, img, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
    for(i = 0; i < flow.rows; i++) {
      for(j = 0; j < flow.cols; j++) {
        p = flow.at<Point2f>(i, j);
        CV_MAT_ELEM(*velx, float, i, j) = p.x;
        CV_MAT_ELEM(*vely, float, i, j) = p.y;
        // cout << "p.x: " << p.x << ", p.y: " << p.y << endl;
        // cout << "velx: " << CV_MAT_ELEM(*velx, float, i,j) << ", vely: " << CV_MAT_ELEM(*vely, float, i,j) << endl;
      }
    }

    //replace last image with current image
    cvCopy(&image, lastImage, NULL);

  } else {
    lastImage = cvCloneImage(&image);
  }

  // cvReleaseImage(&velx);
  // cvReleaseImage(&vely);
  // cv::ReleaseMat(&velx);
  // cv::ReleaseMat(&vely);
  return *oFlow;
}

/// first order optical flow
const OpticalFlow &Farneback::calcOpticalFlow2(Mat &image) {
  return *oFlow;
}

#endif // defined HAVE_OPENCV2 and CV_MINOR_VERSION >= 2
