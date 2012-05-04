#include "ofdifferential.h"

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)

#include "toolbox.h"
//#include "Settings.h"
#include <iostream>
using namespace std;

FirstOrder::FirstOrder(int height, int width) : height(height), width(width), lastImage(0) {
	oFlow = new DenseOpticalFlow(height, width);
}

FirstOrder::~FirstOrder() {
	delete oFlow;
	if(lastImage)
		cvReleaseImage(&lastImage);
}

///	first order optical flow

const OpticalFlow &FirstOrder::calcOpticalFlow(const IplImage &image) {
	const bool unwrapped = true;

	if(lastImage) {

		int step = image.widthStep;
		const uchar *last = (const uchar *)lastImage->imageData;
		const uchar *cur = (const uchar *)image.imageData;
		
		int8_t sgn_u, sgn_v;
		int16_t diff;
		uint32_t abs_grad_x, abs_grad_y, abs_grad_t, grad_magn;

		for(int y=1;y<height-1;y++) {
			for(int x=1;x<width-1;x++) {

				// if(unwrapped || (_ro*_ro*9/4>(x-_cx)*(x-_cx)+(y-_cy)*(y-_cy))
				// 	 ){ // Edit Lóa: Only calculate OF inside circle
					// I_t
					diff = cur[y*step+x] - last[y*step+x];
					abs_grad_t = ABS(diff);

					if(abs_grad_t) {
						// consider signum form I_t 
						sgn_u = SGN(diff); sgn_v = SGN(diff);
						//I_x
						diff = (cur[y*step+x+1] - cur[y*step+x-1]
										+ last[y*step+x+1] - last[y*step+x-1]);
						abs_grad_x = ABS(diff);
						sgn_u *= SGN(diff);
						//I_y
						diff = (cur[(y+1)*step+x] - cur[(y-1)*step+x]
										+ last[(y+1)*step+x] - last[(y-1)*step+x]);
						abs_grad_y = ABS(diff);
						sgn_v *= SGN(diff);

						if(abs_grad_x || abs_grad_y) {
							grad_magn = isqrt( (abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y) << 10);
							// grad_magn = (abs_grad_x*abs_grad_x + abs_grad_y*abs_grad_y);
							// cout << "grad_magn = " << grad_magn << endl;
							// better would be || \nabla I || >= THRESHOLD
							if( grad_magn > 0) { // 6*32
								oFlow->setVelocity(x,
																	 y,
																	 -sgn_u * (((abs_grad_x * abs_grad_t) << 5) / grad_magn),
																	 -sgn_v * (((abs_grad_y * abs_grad_t) << 5) / grad_magn)
								);
								// oFlow->setVelocity(x,
								// 									 y,
								// 									 -sgn_u * (((abs_grad_x * abs_grad_t) << 10) / grad_magn),
								// 									 -sgn_v * (((abs_grad_y * abs_grad_t) << 10) / grad_magn)
								// 									 );


							} else {//set zero
								oFlow->setVelocity(x, y, 0, 0);
							}
						} else {//set zero
							oFlow->setVelocity(x, y, 0, 0);
						}
					} else {//set zero
						oFlow->setVelocity(x, y, 0, 0);
					}

					/*				// better would be || \nabla I || >= THRESHOLD
										if( grad_magn >= 1*256) {
										cout << "dividend_u: " << ((abs_grad_x * abs_grad_t) << 8)
										<< ", dividend_v: " << ((abs_grad_y * abs_grad_t) << 8)
										<< ", grad_magn: " << grad_magn << endl;
										oFlow->setVelocity(x,
										y,
										-sgn_u * (((abs_grad_x * abs_grad_t) << 8) / grad_magn),
										-sgn_v * (((abs_grad_y * abs_grad_t) << 8) / grad_magn) );
										} else {
										oFlow->setVelocity(x, y, 0, 0);
										}*/
					//}
			}
		}
		// cout << oFlow->getMeanVelX(0, 320, 0, 240) << endl;
		// cout << oFlow->getMeanVelY(0, 320, 0, 240) << endl;
		
		// 		oFlow->algo = FIRST_ORDER;

		//replace last image with current image
		cvCopy(&image, lastImage, NULL);

	} else {
		lastImage = cvCloneImage(&image);
	}

	return *oFlow;
}

HornSchunck::HornSchunck(int height, int width) : height(height), width(width), lastImage(0) {
	oFlow = new DenseOpticalFlow(height, width);
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

				oFlow->setVelocity(x, y, u, v);
			}
		}
		

		//replace last image with current image
		cvCopy(&image, lastImage, NULL);

	} else {
		lastImage = cvCloneImage(&image);
	}

	return *oFlow;
}

#endif // defined HAVE_OPENCV2 and CV_MINOR_VERSION >= 2
