// opencv based kalman filter

#ifndef _FILTER_KALMANCV_H_
#define _FILTER_KALMANCV_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV_CV_H
#include "opencv/cv.h" // opencv headers

#include <sstream>

#include "core/logger.h"


namespace mavhub {
	/// Kalman filter class: hover (altitude)
  class Kalman_CV {
  public:
		/// Constructor
		Kalman_CV();
		virtual ~Kalman_CV();

		/// initialize kalman
		virtual void init();
		/// evaluate kalman: predict + correct
		virtual void eval();
		/// update transition matrix
		virtual void update_F_dt(double dt);
		/// update transition matrix
		//virtual void update_R(int row, int col, double val);

		/// static cv matrix print function
		static inline void cvPrintMat(CvMat* M, int rows, int cols, char* name) {
			int i,j;
			std::ostringstream s;
			s << name << " = [\n";
			// printf("%s = [\n", name);
			for(i=0; i < rows; i++) {
				for(j=0; j< cols; j++) {
					s << cvmGet(M, i, j) << " ";
					//printf("%f ", cvmGet(M, i, j));
				}
				s << "\n";
				//printf("\n");
			}
			s << "]";
			//printf("]\n");
			Logger::log(s.str(), Logger::LOGLEVEL_INFO);
			// Logger::log("blub", Logger::LOGLEVEL_INFO);
		}

		/// kalman measurement getter
		inline CvMat* getMeas() {
			return meas;
		}

		/// kalman transition matrix getter
		inline CvMat* getTransMat() {
			return cvkal->transition_matrix;
		}

		/// kalman transition matrix getter
		inline CvMat* getMeasTransMat() {
			return cvkal->measurement_matrix;
		}

		/// kalman measurement noise covariance getter
		inline CvMat* getMeasNoiseCov() {
			return cvkal->measurement_noise_cov;
		}

		/// kalman state post getter
		inline CvMat* getStatePost() {
			return cvkal->state_post;
		}

		/// kalman measurement setter
		inline void setMeasAt(int row, int col, double val) {
			// FIXME: check bound violation
			cvmSet(meas, row, col, val);
		}

		/// kalman measurement matrix setter
		inline void setMeasTransAt(int row, int col, double val) {
			// FIXME: check bound violation
			cvmSet(cvkal->measurement_matrix, row, col, val);
		}

		/// kalman measurement noise covariance setter
		inline void setMeasNoiseCovAt(int row, int col, double val) {
			// FIXME: check bound violation
			cvmSet(cvkal->measurement_noise_cov, row, col, val);
		}

		/// kalman measurement noise covariance getter
		inline double getMeasNoiseCovAt(int row, int col) {
			// FIXME: check bound violation
			return cvmGet(cvkal->measurement_noise_cov, row, col);
		}

		// protected:
  private:
		CvKalman* cvkal;
		CvMat* meas;
  };
}

#endif // HAVE_OPENCV_CV_H

#endif
