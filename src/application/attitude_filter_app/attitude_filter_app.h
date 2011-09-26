#ifndef _ATTITUDE_FILTER_APP_H_
#define _ATTITUDE_FILTER_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV_CV_H
#include <opencv/cv.h>

#include "protocol/protocollayer.h"
#include "core/thread.h"

namespace mavhub {

	class AttitudeFilterApp : public AppLayer<mavlink_message_t> {
		
		public:
			AttitudeFilterApp(const Logger::log_level_t loglevel, uint32_t usMeasurementTimeout, int numberOfMeasurementsForGyroBiasMean);
			virtual ~AttitudeFilterApp();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();
			
			void getIMURawADCData();
			void calibrateGyroBiases();
			void initializeAttitudeFilter();
			void runAttitudeFilter();
			void computeJacobianMatrices();
			void computeCurrentDt();
			
			// States
			enum State {
				doNothing,
				measure
			};
			State mCurrentState;
			
			// Measurement variables
			pthread_mutex_t mMeasurementMutex;
			pthread_cond_t mMeasurementCond;
			uint32_t mUSMeasurementTimeout;

			mavlink_huch_imu_raw_adc_t mCurrentIMURawADCData;
			
			// Calibration variables
			int mNumberOfMeasurementsForGyroBiasMean;
			CvPoint3D32f mGyroBiases;
			
			// Attitude filter variables
			struct timeval mLastTime;
			float mDt;
			CvMat* mCurrentStateEstimate;
			CvMat* mJacobyA;
			CvMat* mJacobyW;
			CvMat* mJacobyH;
			//CvMat* mJacobyV;	// Not needed, because V = Id
	};

}

#endif // HAVE_OPENCV_CV_H

#endif // _ATTITUDE_FILTER_APP_H_

