#ifndef _ACC_CALIBRATION_APP_H_
#define _ACC_CALIBRATION_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_OPENCV_CV_H
#include <opencv/cv.h>

#include <vector>

#include "protocol/protocollayer.h"
#include "core/thread.h"

namespace mavhub {

	class AccCalibrationAppTestThread;

	/*
	* @class AccCalibrationApp
	* 
	* @brief App for calibrating the acceleration sensors, testing and saving the results.
	* 
	* With this app the biases and scale factors of the acceleration sensors can
	* be calibrated. A simple calibration routine is used. You just have to position
	* the IMU in the min and max position of all three axes and press a button, respectively.
	* From this data the biases and scale factors are computed.
	* 
	* After this, the results of the calibration can be tested by rotating the IMU
	* and the App displays the pitch and roll angles computed with the new calibration data.
	* 
	* If the results are satisfying, you can save them.
	*/
	class AccCalibrationApp : public AppLayer<mavlink_message_t> {
		
		friend class AccCalibrationAppTestThread;
		
		public:
			AccCalibrationApp(const Logger::log_level_t loglevel, int numberOfMeasurementsForDetermineMinMax, uint32_t usMeasurementTimeout);
			virtual ~AccCalibrationApp();
			virtual void handle_input(const mavlink_message_t &msg);
			static CvPoint3D32f readInSavedCalibrationBiases();
			static CvPoint3D32f readInSavedCalibrationScaleFactors();

		protected:
			virtual void run();
			bool doCalibration();
			bool collectCalibrationData();
			void measure();
			void computeCalibrationResults();
			void testCalibration();
			void saveCalibrationResults();

			// States
			enum State {
				doNothing,
				displayValues,
				startMeasurement,
				measurementRunning,
				measureAndDisplayValuesForTesting
			};
			State mCurrentState;

			// Measurement variables
			int mMeasurementCounter;
			pthread_mutex_t mMeasurementMutex;
			pthread_cond_t mMeasurementCond;
			uint32_t mUSMeasurementTimeout;
			int mNumberOfMeasurementsForDetermineMinMax;

			enum CurrentMeasurementVariable {
				xMin = 0,
				xMax = 1,
				yMin = 2,
				yMax = 3,
				zMin = 4,
				zMax = 5
      		};

			CurrentMeasurementVariable mCurrentMeasurementVariable;
			float mMeasuredValue;
			CvPoint3D32f mMeasurementMinimums;
			CvPoint3D32f mMeasurementMaximums;

			// Calibration const
			static const float CONST_G;

			// Calibration solution
			CvPoint3D32f mCalibrationBiases;
			CvPoint3D32f mCalibrationScaleFactors;
			
			// Calibration test variables
			AccCalibrationAppTestThread* mAccCalibrationAppTestThread;
			CvPoint3D32f mMeasuredValuesForTesting;
			
			// Paths for saving calibration results
			static const std::string mCalibrationBiasesPath;
			static const std::string mCalibrationScaleFactorsPath;
	};
		
	/*
	* @class AccCalibrationAppTestThread
	* 
	* @brief A helper class for the AccCalibrationApp.
	* 
	* This class realises a seperate thread of the AccCalibrationApp for roll and pitch angle computation and display.
	* A seperate thread is used, so that the handle_input function of the AccCalibrationApp cannot be blocked.
	*/
	class AccCalibrationAppTestThread : public cpp_pthread::PThread {
		public:
			AccCalibrationAppTestThread(AccCalibrationApp* accCalibrationApp);
		protected:
			virtual void run();
			AccCalibrationApp* mAccCalibrationApp;
	};

} // namespace mavhub

#endif // HAVE_OPENCV_CV_H

#endif // HAVE_MAVLINK_H

#endif // _ACC_CALIBRATION_APP_H_
