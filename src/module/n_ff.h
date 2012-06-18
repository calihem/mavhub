// feedforward network
#ifndef _FILTER_N_FF_H_
#define _FILTER_N_FF_H_

#ifdef _HAVE_OPENCV_2
#ifdef HAVE_LIBFANN
#include <sstream>

#include "core/logger.h"

#include "opencv2/opencv.hpp" // opencv headers
#include "fann.h"

namespace mavhub {
	/// Kalman filter class: hover (altitude)
  class N_FF {
  public:
		/// Constructor
		N_FF();
		virtual ~N_FF();

		/// initialize kalman
		virtual void init();
		/// evaluate kalman: predict + correct
		//virtual double eval(cv::Vec<double,5>& );
		virtual double eval(std::vector<double>& );
		/// update transition matrix
		// virtual void update_F_dt(double dt);
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

		// protected:
  private:
		//CvMat* W;
		cv::Mat* W;
		cv::Mat x;
		//CvMat W2;
		cv::Mat W2;
		cv::FileStorage fs;
  };
}

#endif // HAVE_LIBFANN
#endif // _HAVE_OPENCV_2
#endif
