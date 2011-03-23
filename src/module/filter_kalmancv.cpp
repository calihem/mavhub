// kalman opencv
#include "filter_kalmancv.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <stdio.h>

#include "core/logger.h"
#include "utility.h"
#include "core/protocolstack.h"

using namespace std;

namespace mavhub {
	Kalman_CV::Kalman_CV() {
		cvkal = cvCreateKalman(3, 5, 0);
		init();
	}

	Kalman_CV::~Kalman_CV() {
		cvReleaseKalman(&cvkal);
	}

	void Kalman_CV::init() {
		Logger::log("kalman_cv init", Logger::LOGLEVEL_INFO);
		// state transition matrix
		const float F[] = {1.0, 0.02, 0.0, 0.0, 1.0, 0.004, 0.0, 0.0, 1.0};
		memcpy( cvkal->transition_matrix->data.fl, F, sizeof(F));
		// H - measurement transform
		const float H[] = {1.0, 0.0, 0.0, \
											 1.0, 0.0, 0.0, \
											 0.0, 0.0, 1.0, \
											 1.0, 0.0, 0.0, \
											 1.0, 0.0, 0.0
		};
		memcpy(cvkal->measurement_matrix->data.fl, H, sizeof(H));
		// Q - process noise covariance
		cvSetIdentity( cvkal->process_noise_cov, cvRealScalar(5e-3));
		// R - measurement covariance
		const float R[] = {0.05, 0.0, 0.0,  0.0, 0.0, \
											 0.0,     0.5, 0.0,  0.0, 0.0, \
											 0.0,     0.0, 0.01, 0.0, 0.0, \
											 0.0,     0.0, 0.0,  0.05, 0.0, \
											 0.0,     0.0, 0.0,  0.0, 0.05
		}; // 7.86737928e+04
		memcpy(cvkal->measurement_noise_cov->data.fl, R, sizeof(R));
		// P - a posteriori error estimate
		cvSetIdentity( cvkal->error_cov_post, cvRealScalar(1));

		// measurement vector
		meas = cvCreateMat(5,1, CV_32FC1);
	}

	void Kalman_CV::eval() {
		// Logger::log("kalman_cv eval", meas, Logger::LOGLEVEL_INFO);
		// predict
		cvKalmanPredict(cvkal, 0);
		// correct
		cvKalmanCorrect(cvkal, meas);
	}

	void Kalman_CV::update_F_dt(double dt) {
		cvmSet(cvkal->transition_matrix, 0, 1, dt);
		cvmSet(cvkal->transition_matrix, 1, 2, dt*dt);
	}

}
