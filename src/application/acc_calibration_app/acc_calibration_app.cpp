#include "acc_calibration_app.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include "lib/hub/time.h"
using namespace std;
using namespace cpp_pthread;
using namespace hub;

namespace mavhub {

const float AccCalibrationApp::CONST_G = 9.80665f;
const string AccCalibrationApp::mCalibrationBiasesPath = "app/acc_calibration_app/calibration_data/acc_calibration_biases.xml";
const string AccCalibrationApp::mCalibrationScaleFactorsPath = "app/acc_calibration_app/calibration_data/acc_calibration_scale_factors.xml";

AccCalibrationApp::AccCalibrationApp(const Logger::log_level_t loglevel, int numberOfMeasurementsForDetermineMinMax, uint32_t usMeasurementTimeout) :
		AppInterface("acc_calibration_app", loglevel),
		AppLayer<mavlink_message_t>("acc_calibration_app", loglevel) {
	// Transfer initialization parameters to class members
	mUSMeasurementTimeout = usMeasurementTimeout;
	mNumberOfMeasurementsForDetermineMinMax = numberOfMeasurementsForDetermineMinMax;
	
	// Initialize measurement variables
	mCurrentState = doNothing;
	mCurrentMeasurementVariable = xMin;
	pthread_mutex_init(&mMeasurementMutex, NULL);
	pthread_cond_init(&mMeasurementCond, NULL);
	
	// Initialize calibration test variables
	mAccCalibrationAppTestThread = new AccCalibrationAppTestThread(this);
}

AccCalibrationApp::~AccCalibrationApp() {
	pthread_cond_destroy(&mMeasurementCond);
	pthread_mutex_destroy(&mMeasurementMutex);
	
	delete mAccCalibrationAppTestThread;
}

void AccCalibrationApp::run() {
	log("AccCalibrationApp::run() started!", Logger::LOGLEVEL_ALL);

	// Start calibration routine
	if(doCalibration()) {	// Calibration was successful?
		// Test calibration
		testCalibration();
		// Save calibration results
		saveCalibrationResults();
	}
}

void AccCalibrationApp::handle_input(const mavlink_message_t &msg) {
	int rc;
		
	log("AccCalibrationApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);

	// Get IMU messages
	if(msg.msgid == MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC) {
		log("AccCalibrationApp got HUCH_IMU_RAW_ADC message", Logger::LOGLEVEL_DEBUG);
		
		{//begin of measurementMutex Lock
			Lock measurementLock(mMeasurementMutex);

			switch(mCurrentState) {
				// Display current acc values
				case displayValues:
					switch(mCurrentMeasurementVariable) {
						case xMax:
						case xMin:
							cout << "\rX = " << setfill(' ') << setw(5) << (float)mavlink_msg_huch_imu_raw_adc_get_xacc(&msg);
							break;
						case yMax:
						case yMin:
							cout << "\rY = " << setfill(' ') << setw(5) << (float)mavlink_msg_huch_imu_raw_adc_get_yacc(&msg);
							break;
						case zMax:
						case zMin:
							cout << "\rZ = " << setfill(' ') << setw(5) << (float)mavlink_msg_huch_imu_raw_adc_get_zacc(&msg);
							break;
					}
					break;
				// Start a measurement
				case startMeasurement:
					mCurrentState = measurementRunning;
					mMeasurementCounter = 0;
					// No break!!!
				// Measurement running
				case measurementRunning:
					// Special case first run
					if(mMeasurementCounter == 0) {
						switch(mCurrentMeasurementVariable) {
							case xMax:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_xacc(&msg);
								mMeasurementMaximums.x = mMeasuredValue;
								break;
							case xMin:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_xacc(&msg);
								mMeasurementMinimums.x = mMeasuredValue;
								break;
							case yMax:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_yacc(&msg);
								mMeasurementMaximums.y = mMeasuredValue;
								break;
							case yMin:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_yacc(&msg);
								mMeasurementMinimums.y = mMeasuredValue;
								break;
							case zMax:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_zacc(&msg);
								mMeasurementMaximums.z = mMeasuredValue;
								break;
							case zMin:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_zacc(&msg);
								mMeasurementMinimums.z = mMeasuredValue;
								break;
						}
					} else {	// If not first run, compare current value to min/max
						switch(mCurrentMeasurementVariable) {
							case xMax:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_xacc(&msg);
								if(mMeasuredValue > mMeasurementMaximums.x) {
									mMeasurementMaximums.x = mMeasuredValue;
								}
								break;
							case xMin:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_xacc(&msg);
								if(mMeasuredValue < mMeasurementMinimums.x) {
									mMeasurementMinimums.x = mMeasuredValue;
								}
								break;
							case yMax:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_yacc(&msg);
								if(mMeasuredValue > mMeasurementMaximums.y) {
									mMeasurementMaximums.y = mMeasuredValue;
								}
								break;
							case yMin:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_yacc(&msg);
								if(mMeasuredValue < mMeasurementMinimums.y) {
									mMeasurementMinimums.y = mMeasuredValue;
								}
								break;
							case zMax:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_zacc(&msg);
								if(mMeasuredValue > mMeasurementMaximums.z) {
									mMeasurementMaximums.z = mMeasuredValue;
								}
								break;
							case zMin:
								mMeasuredValue = (float)mavlink_msg_huch_imu_raw_adc_get_zacc(&msg);
								if(mMeasuredValue < mMeasurementMinimums.z) {
									mMeasurementMinimums.z = mMeasuredValue;
								}
								break;
						}
					}
					// Increase measurement counter
					mMeasurementCounter++;
					// If measurement counter is full, stop measurement and wake up waiting thread
					if(mMeasurementCounter == mNumberOfMeasurementsForDetermineMinMax) {
						mCurrentState = doNothing;
						rc = pthread_cond_signal(&mMeasurementCond);
						if(rc != 0) {
							throw "AccCalibrationApp::handle_input: pthread_cond_signal failed!";
						}
					}
					break;
				// Values for testing should be measured and displayed
				case measureAndDisplayValuesForTesting:
					mMeasuredValuesForTesting.x = (float)mavlink_msg_huch_imu_raw_adc_get_xacc(&msg);
					mMeasuredValuesForTesting.y = (float)mavlink_msg_huch_imu_raw_adc_get_yacc(&msg);
					mMeasuredValuesForTesting.z = (float)mavlink_msg_huch_imu_raw_adc_get_zacc(&msg);
					// Start thread for calculating calibrated pitch and roll angles and display them
					mCurrentState = doNothing;	// Don't react on messages while thread below is running
					mAccCalibrationAppTestThread->join();
					mAccCalibrationAppTestThread->start();
					break;
				case doNothing:
				default:
					break;
			}
		}//end of measurementMutex Lock
	}
}

/**
 * Does the calibration.
 * 
 * @return true, if calibration was successful, otherwise false.
 */
bool AccCalibrationApp::doCalibration() {
	log("AccCalibrationApp::doCalibration() started!", Logger::LOGLEVEL_ALL);

	// User output
	cout << "Accelerometer calibration routine started. Please follow the instructions carefully!" << endl;
	cout << "..." << endl;
	
	// Collect calibration data
	bool continueCalibration;
	continueCalibration = collectCalibrationData();
	if(!continueCalibration) {
		return false;
	}
	
	// Compute calibration results from calibration data
	computeCalibrationResults();
	
	return true;
}

/**
 * Collects the calibration data for all axes.
 * 
 * @return false if the user aborts, otherwise true.
 */
bool AccCalibrationApp::collectCalibrationData() {
	// Start collecting calibration data
	for(int i = 0; i < 6; i++) {
		string str;	// user input string
		string currentMeasurementValueName;	// name of current measurement value
		bool continueCalibration = false;	// continue calibration?
		bool abortCalibration = false;	// abort calibration?

		{//begin of measurementMutex Lock
			Lock measurementLock(mMeasurementMutex);
			switch(i) {
				case 0:
					currentMeasurementValueName = "xMin";
					mCurrentMeasurementVariable = xMin;
					break;
				case 1:
					currentMeasurementValueName = "xMax";
					mCurrentMeasurementVariable = xMax;
					break;
				case 2:
					currentMeasurementValueName = "yMin";
					mCurrentMeasurementVariable = yMin;
					break;
				case 3:
					currentMeasurementValueName = "yMax";
					mCurrentMeasurementVariable = yMax;
					break;
				case 4:
					currentMeasurementValueName = "zMin";
					mCurrentMeasurementVariable = zMin;
					break;
				case 5:
					currentMeasurementValueName = "zMax";
					mCurrentMeasurementVariable = zMax;
					break;
			}		
			// User output
			cout << "Bring the IMU into the " << currentMeasurementValueName << " position (only gravitation force, no other acceleration!) and press [Enter] when ready. Press [x/X] to abort calibration." << endl;	
			mCurrentState = displayValues;
		}//end of measurementMutex Lock
		// Wait for user input
		do {
			getline(cin, str);
			if((str.compare("x") == 0) || (str.compare("X") == 0)) {
				abortCalibration = true;
			} else if(str.length() == 0) {
				continueCalibration = true;
			}
		} while(!(continueCalibration || abortCalibration));
		{//begin of measurementMutex Lock
			Lock measurementLock(mMeasurementMutex);
			mCurrentState = doNothing;
		}//end of measurementMutex Lock
		if(abortCalibration) {
			return false;
		}
		// Start measurement
		measure();
		// Display measured value
		cout << endl << currentMeasurementValueName << " = " << mMeasuredValue << endl;
	}
	return true;
}

/**
 * Makes one acceleration data measurement.
 * 
 */
void AccCalibrationApp::measure() {
	int rc;
	
	{//begin of measurementMutex Lock
		Lock measurementLock(mMeasurementMutex);
		// Signal to handle_input that measurement should be started
		mCurrentState = startMeasurement;
		
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
			throw "AccCalibrationApp::measure: Measurement timed out!";
		} else if(rc != 0) { // pthread_cond_timewait error
			throw "AccCalibrationApp::measure: pthread_cond_timewait failed!";
		}
	}//end of measurementMutex Lock
}

/**
 * Computes the calibration results from the collected calibration data.
 * 
 */
void AccCalibrationApp::computeCalibrationResults() {
	// Validate collected calibration data
	if(mMeasurementMinimums.x >= mMeasurementMaximums.x) {
		throw "AccCalibrationApp::computeCalibrationResults: xMin >= xMax";
	}
	if(mMeasurementMinimums.y >= mMeasurementMaximums.y) {
		throw "AccCalibrationApp::computeCalibrationResults: yMin >= yMax";
	}
	if(mMeasurementMinimums.z >= mMeasurementMaximums.z) {
		throw "AccCalibrationApp::computeCalibrationResults: zMin >= zMax";
	}
	// Compute biases and scale factors
	float diff;
	diff = mMeasurementMaximums.x - mMeasurementMinimums.x;
	mCalibrationBiases.x = mMeasurementMinimums.x + diff / 2.f;
	mCalibrationScaleFactors.x = 2.f * CONST_G / diff;
	diff = mMeasurementMaximums.y - mMeasurementMinimums.y;
	mCalibrationBiases.y = mMeasurementMinimums.y + diff / 2.f;
	mCalibrationScaleFactors.y = 2.f * CONST_G / diff;
	diff = mMeasurementMaximums.z - mMeasurementMinimums.z;
	mCalibrationBiases.z = mMeasurementMinimums.z + diff / 2.f;
	mCalibrationScaleFactors.z = 2.f * CONST_G / diff;

	// Print out Results
	cout << "Calibration results:" << endl;
	cout << "XBias = " << mCalibrationBiases.x << endl;
	cout << "XScaleFactor = " << mCalibrationScaleFactors.x << endl;
	cout << "YBias = " << mCalibrationBiases.y << endl;
	cout << "YScaleFactor = " << mCalibrationScaleFactors.y << endl;
	cout << "ZBias = " << mCalibrationBiases.z << endl;
	cout << "ZScaleFactor = " << mCalibrationScaleFactors.z << endl;
}

/**
 * Displays the current yaw roll and pitch angles computed with the calibration data.
 * 
 */
void AccCalibrationApp::testCalibration() {
	log("AccCalibrationApp::testCalibration() started!", Logger::LOGLEVEL_ALL);

	// User output
	cout << "Now computing pitch and roll angles with new calibration data for testing. Press [Enter] to continue." << endl; 
	
	// Start measure and display values
	{//begin of measurementMutex Lock
		Lock measurementLock(mMeasurementMutex);
		mCurrentState = measureAndDisplayValuesForTesting;
	}//end of measurementMutex Lock
	
	// Waiting for [Enter] from user
	string str;
	getline(cin, str);
	
	// Stop computing and displaying
	{//begin of measurementMutex Lock
		Lock measurementLock(mMeasurementMutex);
		mCurrentState = doNothing;
	}//end of measurementMutex Lock
		
}

AccCalibrationAppTestThread::AccCalibrationAppTestThread(AccCalibrationApp* accCalibrationApp) {
	mAccCalibrationApp = accCalibrationApp;
}

/**
 * A seperate thread for roll and pitch angle computation and display. A seperate thread is used, so that the handle_input function cannot be blocked.
 * 
 */
void AccCalibrationAppTestThread::run() {
	{//begin of measurementMutex Lock
		Lock measurementLock(mAccCalibrationApp->mMeasurementMutex);
		// Get measured values
		float uncalibratedAccX = mAccCalibrationApp->mMeasuredValuesForTesting.x;
		float uncalibratedAccY = mAccCalibrationApp->mMeasuredValuesForTesting.y;
		float uncalibratedAccZ = mAccCalibrationApp->mMeasuredValuesForTesting.z;
		// Compute calibrated values
		float accX = mAccCalibrationApp->mCalibrationScaleFactors.x * (uncalibratedAccX - mAccCalibrationApp->mCalibrationBiases.x);
		float accY = mAccCalibrationApp->mCalibrationScaleFactors.y * (uncalibratedAccY - mAccCalibrationApp->mCalibrationBiases.y);
		float accZ = mAccCalibrationApp->mCalibrationScaleFactors.z * (uncalibratedAccZ - mAccCalibrationApp->mCalibrationBiases.z);
		// Compute pitch angle phi
		float g = mAccCalibrationApp->CONST_G;
		float phi = 180.0 * asin(accY / g) / M_PI;
		// Compute roll angle theta
		float theta = 180.0 * atan(accX / accZ) / M_PI;
		// Display computed angles
		cout << "\rPitch angle phi = " << fixed << setfill(' ') << setw(5) << setprecision(1) << phi << ", Roll angle theta = "  << theta;
		// AccCalibrationApp can measure next value
		mAccCalibrationApp->mCurrentState = mAccCalibrationApp->measureAndDisplayValuesForTesting;
	}//end of measurementMutex Lock
}

/**
 * Saves the calibration results to a file.
 * 
 */
void AccCalibrationApp::saveCalibrationResults() {
	// Ask user for Saving
	string str;
	bool saveResults = false;
	bool abort = false;
	cout << endl << "Press [s/S] to save calibration results or [x/X] to abort." << endl;
	do {
		getline(cin, str);
		if((str.compare("x") == 0) || (str.compare("X") == 0)) {
			abort = true;
		} else if((str.compare("s") == 0) || (str.compare("S") == 0)) {
			saveResults = true;
		}
	} while(!(saveResults || abort));
	
	// If aborted, return
	if(abort) {
		cout << "Calibration results not saved!" << endl;
		return;
	}
	
	// Otherwise save calibration results
		// Convert Points to Matrices for Saving
		CvMat* biases = cvCreateMat(3, 1, CV_32FC1);
		cvmSet(biases, 0, 0, mCalibrationBiases.x);
		cvmSet(biases, 1, 0, mCalibrationBiases.y);
		cvmSet(biases, 2, 0, mCalibrationBiases.z);
		CvMat* scaleFactors = cvCreateMat(3, 1, CV_32FC1);
		cvmSet(scaleFactors, 0, 0, mCalibrationScaleFactors.x);
		cvmSet(scaleFactors, 1, 0, mCalibrationScaleFactors.y);
		cvmSet(scaleFactors, 2, 0, mCalibrationScaleFactors.z);
	
		// Save to file
		cvSave(mCalibrationBiasesPath.data(), biases, "CalibrationBiases");
		cvSave(mCalibrationScaleFactorsPath.data(), scaleFactors, "CalibrationScaleFactors");
	
		// User output
		cout << "Saved calibration results!" << endl;
	
		// Clean up
		cvReleaseMat(&biases);
		cvReleaseMat(&scaleFactors);
}

/**
 * Reads in the saved calibration biases from file.
 * 
 */
CvPoint3D32f AccCalibrationApp::readInSavedCalibrationBiases() {
	CvPoint3D32f ret;
	CvMat* calibrationBiasesMat = (CvMat*)cvLoad(mCalibrationBiasesPath.data());
	if(calibrationBiasesMat == NULL) {
		string err = "AccCalibrationApp::readInSavedCalibrationBiases: Read in failed! No calibration file at " + mCalibrationBiasesPath + "?";;
		throw err;
	}
	ret.x = cvmGet(calibrationBiasesMat, 0, 0);
	ret.y = cvmGet(calibrationBiasesMat, 1, 0);
	ret.z = cvmGet(calibrationBiasesMat, 2, 0);
	cvReleaseMat(&calibrationBiasesMat);
	return ret;
}

/**
 * Reads in the saved calibration scale factors from file.
 * 
 */
CvPoint3D32f AccCalibrationApp::readInSavedCalibrationScaleFactors() {
	CvPoint3D32f ret;
	CvMat* calibrationScaleFactorsMat = (CvMat*)cvLoad(mCalibrationScaleFactorsPath.data());
	if(calibrationScaleFactorsMat == NULL) {
		string err = "AccCalibrationApp::readInSavedCalibrationScaleFactors: Read in failed! No calibration file at " + mCalibrationScaleFactorsPath + "?";;
		throw err;
	}
	ret.x = cvmGet(calibrationScaleFactorsMat, 0, 0);
	ret.y = cvmGet(calibrationScaleFactorsMat, 1, 0);
	ret.z = cvmGet(calibrationScaleFactorsMat, 2, 0);
	cvReleaseMat(&calibrationScaleFactorsMat);
	return ret;
}

} // namespace mavhub

#endif // HAVE_OPENCV

#endif // HAVE_MAVLINK_H
