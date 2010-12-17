// opencv based kalman filter

#ifndef _FILTER_KALMANCV_H_
#define _FILTER_KALMANCV_H_

#include <sstream>

#include "logger.h"
#include "protocollayer.h"

#include "opencv/cv.h" // opencv headers

using namespace std;

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

		/// static cv matrix print function
		static inline void cvPrintMat(CvMat* M, int rows, int cols, char* name) {
			int i,j;
			ostringstream s;
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

		/// kalman state post getter
		inline CvMat* getStatePost() {
			return cvkal->state_post;
		}

		/// kalman measurement setter
		inline void setMeasAt(int row, int col, double val) {
			// XXX: check bound violation
			cvmSet(meas, row, col, val);
		}

		/// kalman measurement matrix setter
		inline void setMeasTransAt(int row, int col, double val) {
			// XXX: check bound violation
			cvmSet(cvkal->measurement_matrix, row, col, val);
		}

		// protected:
  private:
		CvKalman* cvkal;
		CvMat* meas;
  };
}

#endif
