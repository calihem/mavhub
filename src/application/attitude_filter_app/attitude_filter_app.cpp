#include "attitude_filter_app.h"

#include "core/datacenter.h"
#include "application/acc_calibration_app/acc_calibration_app.h"

#include <iostream>
#include <opencv/cv.h>
#include <cmath>

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

AttitudeFilterApp::AttitudeFilterApp(const Logger::log_level_t loglevel, uint32_t usMeasurementTimeout, int numberOfMeasurementsForGyroBiasMean) :
		AppLayer("attitude_filter_app", loglevel) {

	// Transfer initialization parameters to class members
	mUSMeasurementTimeout = usMeasurementTimeout;
	mNumberOfMeasurementsForGyroBiasMean = numberOfMeasurementsForGyroBiasMean;

	// Initialize state	
	mCurrentState = doNothing;
			
	// Initialize measurement variables
	pthread_mutex_init(&mMeasurementMutex, NULL);
	pthread_cond_init(&mMeasurementCond, NULL);
	
	// Initialize attitude filter variables
	mCurrentStateEstimate = cvCreateMat(6, 1, CV_32FC1);
	mJacobyA = cvCreateMat(6, 6, CV_32FC1);
	mJacobyW = cvCreateMat(6, 6, CV_32FC1);
	mJacobyH = cvCreateMat(3, 6, CV_32FC1);
}

AttitudeFilterApp::~AttitudeFilterApp() {
	pthread_cond_destroy(&mMeasurementCond);
	pthread_mutex_destroy(&mMeasurementMutex);
	
	// Release attitude filter variables
	cvReleaseMat(&mCurrentStateEstimate);
	cvReleaseMat(&mJacobyA);
	cvReleaseMat(&mJacobyW);
	cvReleaseMat(&mJacobyH);
}

void AttitudeFilterApp::handle_input(const mavlink_message_t &msg) {
	int rc;
	
	log("AttitudeFilerApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
	
	// Get IMU messages
	if(msg.msgid == MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC) {
		log("AttitudeFilerApp got HUCH_IMU_RAW_ADC message", Logger::LOGLEVEL_DEBUG);
		
		{//begin of measurementMutex Lock
			Lock measurementLock(mMeasurementMutex);
			
			switch(mCurrentState) {
				case measure:
					// Save IMU data
					mavlink_msg_huch_imu_raw_adc_decode(&msg, &mCurrentIMURawADCData);
					// Switch state back
					mCurrentState = doNothing;
					// Wake up waiting thread
					rc = pthread_cond_signal(&mMeasurementCond);
					if(rc != 0) {
						throw "AttitudeFilerApp::handle_input: pthread_cond_signal failed!";
					}
					break;
				default:
					break;
			}
		}//end of measurementMutex Lock
	}
}

void AttitudeFilterApp::run() {
	log("AttitudeFilterApp::run() started!", Logger::LOGLEVEL_ALL);

	try {
		// Read in acc calibration data
		cout << "Read in acc calibration data ... ";
		AccCalibrationApp::readInSavedCalibrationBiases();
		AccCalibrationApp::readInSavedCalibrationScaleFactors();
		cout << "finished!" << endl;
	
		// Start gyro calibration (biases)
		calibrateGyroBiases();

		// Run attitude filter
		runAttitudeFilter();
	} catch(string smess) {
		log(smess, Logger::LOGLEVEL_ERROR);
		return;
	}

}

void AttitudeFilterApp::initializeAttitudeFilter() {

	// Initialize Time
	gettimeofday(&mLastTime, NULL);
}

void AttitudeFilterApp::runAttitudeFilter() {
	// Get current state estimate variables
	float curPhi	= cvmGet(mCurrentStateEstimate, 0, 0);
	float curTheta	= cvmGet(mCurrentStateEstimate, 1, 0);
	float curPsi	= cvmGet(mCurrentStateEstimate, 2, 0);
	float curBOmX	= cvmGet(mCurrentStateEstimate, 3, 0);
	float curBOmY	= cvmGet(mCurrentStateEstimate, 4, 0);
	float curBOmZ	= cvmGet(mCurrentStateEstimate, 5, 0);
	
	// Precompute values needed in computations below
		// Compute calibrated gyro values
	//	float curCalOmegaX = cur
	
	float curSinPhi = sin(curPhi);
	float curCosPhi = cos(curPhi);
	float curSinTheta = sin(curTheta);
	float curCosTheta = cos(curTheta);
	float curTanTheta = tan(curTheta);
	float curSinPsi = sin(curPsi);
	float curCosPsi = cos(curPsi);
	
	float curSinPhi_M_TanTheta = curSinPhi * curTanTheta;
	float curCosPhi_M_TanTheta = curCosPhi * curTanTheta;
	
	float curSinPhi_D_CosTheta = curSinPhi / curCosTheta;
	float curCosPhi_D_CosTheta = curCosPhi / curCosTheta;

	// Compute Jacobian matrices
}

void AttitudeFilterApp::computeCurrentDt() {
	// Get current time
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	
	// Compute dt with lastTime
	struct timeval diff;
	timediff(diff, mLastTime, currentTime);
	
	mDt = diff.tv_sec + diff.tv_usec / 1000000.f;
}

void AttitudeFilterApp::computeJacobianMatrices() {
	// Get current state estimate variables
	float curPhi	= cvmGet(mCurrentStateEstimate, 0, 0);
	float curTheta	= cvmGet(mCurrentStateEstimate, 1, 0);
	float curPsi	= cvmGet(mCurrentStateEstimate, 2, 0);
	float curBOmX	= cvmGet(mCurrentStateEstimate, 3, 0);
	float curBOmY	= cvmGet(mCurrentStateEstimate, 4, 0);
	float curBOmZ	= cvmGet(mCurrentStateEstimate, 5, 0);
	
	// Precompute values needed in computations below
		// Compute calibrated gyro values
//		float curCalOmegaX
	
	float curSinPhi = sin(curPhi);
	float curCosPhi = cos(curPhi);
	float curSinTheta = sin(curTheta);
	float curCosTheta = cos(curTheta);
	float curTanTheta = tan(curTheta);
	float curSinPsi = sin(curPsi);
	float curCosPsi = cos(curPsi);
	
	float curSinPhi_M_TanTheta = curSinPhi * curTanTheta;
	float curCosPhi_M_TanTheta = curCosPhi * curTanTheta;
	
	float curSinPhi_D_CosTheta = curSinPhi / curCosTheta;
	float curCosPhi_D_CosTheta = curCosPhi / curCosTheta;
	
	// A
//	cvmSet(mJacobyA, 0, 0, 
}

void AttitudeFilterApp::calibrateGyroBiases() {
	// User output
	cout << "Gyro calibration:" << endl;
	cout << "Please bring the IMU into a stable position and then press [Enter]." << endl;

	// Waiting for [Enter] from user
	string str;
	getline(cin, str);
	
	// Get gyro mean
	mGyroBiases.x = 0;
	mGyroBiases.y = 0;
	mGyroBiases.z = 0;
	for(int i = 0; i < mNumberOfMeasurementsForGyroBiasMean; i++) {
		getIMURawADCData();
		mGyroBiases.x += mCurrentIMURawADCData.xgyro;
		mGyroBiases.y += mCurrentIMURawADCData.ygyro;
		mGyroBiases.z += mCurrentIMURawADCData.zgyro;
	}
	mGyroBiases.x /= mNumberOfMeasurementsForGyroBiasMean;
	mGyroBiases.y /= mNumberOfMeasurementsForGyroBiasMean;
	mGyroBiases.z /= mNumberOfMeasurementsForGyroBiasMean;
	
	cout << "Gyro calibration results:" << endl;
	cout << "BiasX = " << mGyroBiases.x << endl;
	cout << "BiasY = " << mGyroBiases.y << endl;
	cout << "BiasZ = " << mGyroBiases.z << endl;
}

void AttitudeFilterApp::getIMURawADCData() {
	int rc;
	
	{//begin of measurementMutex Lock
		Lock measurementLock(mMeasurementMutex);

		// Set state to measurement
		mCurrentState = measure;
		
		// Get current time to set timeout for measurement
		struct timeval now;
		struct timespec timeout;
			
		gettimeofday(&now, NULL);
		timeout = timeval_to_timespec(add_delta_us_to_timeval(now, mUSMeasurementTimeout));
		
		// Start waiting for measurement data
		rc = pthread_cond_timedwait(&mMeasurementCond, &mMeasurementMutex, &timeout);
		mCurrentState = doNothing;	// On error
		
		// Evaluate result of measurement
		if(rc == ETIMEDOUT) { // measurement timeout
			throw string("AttitudeFilterApp::getIMURawADCData: Measurement timed out!");
		} else if(rc != 0) { // pthread_cond_timewait error
			throw string("AttitudeFilterApp::getIMURawADCData: pthread_cond_timewait failed!");
		}
	}//end of measurementMutex Lock
}

} // namespace mavhub
