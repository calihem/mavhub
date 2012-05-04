#ifdef _HAVE_OPENCV_2
#ifdef HAVE_LIBFANN
// kalman opencv
#include "n_ff.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <stdio.h>

#include "core/logger.h"
#include "utility.h"

using namespace std;
using namespace cv;

namespace mavhub {
	N_FF::N_FF() {
		init();
	}

	N_FF::~N_FF() {

	}

	void N_FF::init() {
		Logger::log("n_ff init", Logger::LOGLEVEL_INFO);
		//Mat W(6, 6, CV_32FC2);
		float m[6][12] = 		{
			{ 0.0078 , -0.0202,  0.0684,  0.0283, -0.0619, 0.0538,
				0.        ,  0.        ,  0.        ,  0.        ,  0.        , 0.        },
			{ 0.0370,  0.0441,  0.0684, -0.087, -0.0650 , 0.0380,
				0.        ,  0.        ,  0.        ,  0.        ,  0.        , 0.        },
			{ 0.0160, -0.0533, -0.0585, -0.0993,  0.0549,	0.0814,  0.        ,  0.        ,  0.        ,  0.        ,
				0.        ,  0.        },
			{-0.0068,  0.0911,  0.0804,  0.0898,  0.0947,
			 -0.0695 ,  0.        ,  0.        ,  0.        ,  0.        ,
			 0.        ,  0.        },
			{-0.0399, -0.0601,  0.0347  , -0.0731, -0.0772,
			 -0.0745 ,  0.        ,  0.        ,  0.        ,  0.        ,
			 0.        ,  0.        },
			{-0.0591,  0.        ,  0.        ,  0.        ,  0.        ,
			 0.        ,  0.0523  ,  0.0363,  0.0659, -0.0042,
			 -0.0678 ,  0.        }
		};

		//Mat W = Mat::zeros(6, 12, CV_32F, m);
		//W = new Mat(6, 12, CV_32FC1, m);
		W2.create(6, 12, CV_32FC1);
		// W2 = Mat(6, 12, CV_32FC1, m);
		// //cout << "Z = " << endl << " " << Z << endl << endl;
		x = Mat(2,2,CV_32FC1);
		cout << "x = " << x << endl;

		//cout << "W = "<< endl << " " << *W << endl << endl;
		cout << "W2 = "<< endl << " " << W2 << endl << endl;

		int i, j;
		for(i = 0; i < 6; i++) {
			for(j = 0; j < 12; j++) {
				W2.at<float>(i,j) = m[i][j];
				//cout << "W2" << W2.at<double>(i,j) << endl;
				//=1./(i+j+1);
			}
		}
		cout << "W2 = "<< endl << " " << W2 << endl << endl;
		// FileStorage fs("W.yml", FileStorage::WRITE);
		// fs << "W" << *W;
		// fs.release();
	}

	//double N_FF::eval(Vec<double, 5>& v) {
	double N_FF::eval(vector<double>& v) {
		//Mat x_(v, CV_32FC1);
		//Mat x_ = Mat::ones(12, 1, CV_32FC1);
		cout << "x = " << x << endl;
		//cout << "W = " << (*W) << endl;
		cout << "W2 = " << W2 << endl;
		//cvPrintMat(&W2, 6, 12, "W2");

		//cout << "W*v = " << (*W)*x_ << endl;
		//cout << "W.type = " << W->type() << ", " << x_.type() << endl;
		
		//Logger::log("n_ff eval", v, Logger::LOGLEVEL_INFO);
		return 0.0;
	}

	// void N_FF::update_F_dt(double dt) {
	// 	cvmSet(cvkal->transition_matrix, 0, 1, dt);
	// 	cvmSet(cvkal->transition_matrix, 1, 2, dt*dt);
	// }

}

#endif // HAVE_LIBFANN
#endif
