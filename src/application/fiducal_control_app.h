#ifndef _FIDUCAL_CONTROL_APP_H_
#define _FIDUCAL_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>

#include <vector>

#define FIDUCAL_LOG 1

namespace mavhub {

class FiducalControlApp : public MavlinkAppLayer{
  class PIDController{
    public:
      PIDController(double kp, double ki, double kp, double imax, double eps, double setpoint) Kp(kp), Ki(ki), Kp(kp), iMax(imax), epsilon(eps), setPoint(setpoint), prevError(0.0), integral(0.0) { }
      double operator(double dt) {
        float error;
        float derivative;
        float output;

        error = setpoint - actual_position;
        
        if(abs(error) > epsilon) {
          integral = integral + error*dt;
        }
        if(integral > iMax) {
          integral = iMax;
        }
        else if(integral < -iMax) {
          integral = -iMax;
        }
        derivative = (error - prevError)/dt;
        output = Kp*error + Ki*integral + Kd*derivative;

        //Update error
        prevError = error;

        return output;
      }
    private:
      double Kp;
      double Ki;
      double Kd;
      double iMax;
      double epsilon;
      double setPoint;
      double prevError;
      double integral;
  };
	public:
		FiducalControlApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~FiducalApp();

		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

	protected:
		virtual void run();

	private:
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat fvec;
    cv::Mat setpoint;
#ifdef _LOG
		std::ofstream log_file;
#endif
};

} // namespace mavhub

#endif // HAVE_OPENCV2
#endif // HAVE_MAVLINK_H
#endif // _FIDUCAL_APP_CONTROL_H_
