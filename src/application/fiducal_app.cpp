#include "fiducal_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 3)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

FiducalApp::FiducalApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	MavlinkAppLayer("slam_app", loglevel),
	hub::gstreamer::VideoClient(),
	cam_matrix(3, 3, CV_32FC1),
	dist_coeffs( cv::Mat::zeros(4, 1, CV_32FC1) ),
#ifdef FIDUCAL_LOG
	, log_file("fiducal_log.data")
#endif
	{

	pthread_mutex_init(&sync_mutex, NULL);

	// set sink name
	std::map<std::string,std::string>::const_iterator iter = args.find("sink");
	if( iter != args.end() ) {
		sink_name.assign(iter->second);
	} else {
		log(name(), ": sink argument missing", Logger::LOGLEVEL_DEBUG);
		sink_name.assign("sink0");
	}

	get_value_from_args("out_stream", with_out_stream);

	// get calibration data of camera
	//FIXME: use image dimensions for default camera matrix
	cam_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, 160.0, 0.0, 1.0, 120.0, 0.0, 0.0, 1.0);
	string calib_filename;
	get_value_from_args("calibration_data", calib_filename);
	if(!calib_filename.empty())
		load_calibration_data(calib_filename);

#ifdef FIDUCAL_LOG
	log_file << "# time [ms]"
		<< " | IMU roll angle [rad] | IMU pitch angle [rad] | IMU yaw angle [rad]"
		<< " | roll angular speed [rad/s] | pitch angular speed [rad/s] | yaw angular speed [rad/s]"
		<< " | altitude [cm]"
		<< " | Cam roll angle [rad] | Cam pitch angle [rad] | Cam yaw angle [rad]"
		<< " | Cam x position [cm] | Cam y positionn [cm] | Cam z position [cm]"
		<< std::endl;
	log_file << "#" << std::endl;
	log_file << setprecision(5) << fixed << setfill(' ');
#endif
}

FiducalApp::~FiducalApp() {}

void FiducalApp::handle_input(const mavlink_message_t &msg) {
}

void FiducalApp::load_calibration_data(const std::string &filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if( !fs.isOpened() ) {
		log("Can't open calibration data", filename, Logger::LOGLEVEL_DEBUG);
		return;
	}

	fs["camera_matrix"] >> cam_matrix;
	fs["distortion_coefficients"] >> dist_coeffs;
  
  fs.release();
}

void FiducalApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
	if(!data) return;

	Logger::log(name(), ": got new video data of size", width, "x", height, Logger::LOGLEVEL_DEBUG, _loglevel);
	if(bpp != 8) {
		log(name(), ": unsupported video data with bpp =", bpp, Logger::LOGLEVEL_WARN);
		return;
	}

	// make a matrix header for captured data
	unsigned char *image_data = const_cast<unsigned char*>(data);
	cv::Mat frame(height, width, CV_8UC1, image_data);

	Lock sync_lock(sync_mutex);
  frame.copyTo(video_data);
	new_video_data = true;
}

void FiducalApp::run()
{
	log(name(), ": running", Logger::LOGLEVEL_DEBUG);

	if(Core::video_server) {
		int rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), sink_name.c_str());
		Logger::log(name(), ": binded to", sink_name, rc, Logger::LOGLEVEL_DEBUG, _loglevel);
	} else {
		log(name(), ": video server not running", Logger::LOGLEVEL_WARN);
		return;
	}

	while( !interrupted() ) {
    if(new_video_data)
    {
	    Lock sync_lock(sync_mutex);
      std::vector<std::vector<cv::Point> > markers;
      findTwoRectangles(video_data, markers);

      std::vector<cv::Point2f> outerMarkers;
      std::vector<cv::Point2f> innerMarkers;
      if(markers.size() == 2)
      {
        refineMarkers(video_data, markers, outerMarkers, innerMarkers);
        orderMarkers(video_data, outerMarkers, innerMarkers);

        cv::Mat rvec;
        cv::Mat tvec;
        doGeometry(outerMarkers, innerMarkers, cameraMatrix, distCoeffs, rvec, tvec);
        
        cv::Mat rmat;
        cv::Rodrigues(-rvec, rmat);
        cv::Mat finalVec = rmat*tvec;
        std::cout << finalVec << std::endl;
      }
      
      new_video_data = false;
    }
    //FIXME: remove usleep
    usleep(5000);
  }
}

cv::Point calcCentroid(std::vector<cv::Point> points)
{
  cv::Point centroid;
  for(std::vector<cv::Point>::iterator it = points.begin(); it != points.end(); it++)
    centroid += *it;
  centroid *= 1.0 / double(points.size());
  return centroid;
}

void FiducalApp::findTwoRectangles(const cv::Mat &grayscale, std::vector< std::vector<cv::Point> > &markers)
{
  int xsize = grayscale.size().width;
  int ysize = grayscale.size().height;
  
  cv::Mat binary;
  cv::threshold(grayscale, binary, cv::mean(grayscale)[0], 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > largeContours;
  for(int i = 0; i < hierarchy.size(); i++)
    if(std::fabs(cv::contourArea(contours[i])) > 100)
      largeContours.push_back(contours[i]);
  
  std::vector<std::vector<cv::Point> > rectangles;
  for(int i = 0; i < largeContours.size(); i++)
  {
    cv::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(largeContours[i]), approx, cv::arcLength(cv::Mat(largeContours[i]), true)*0.02, true);
    if(approx.size() == 4 && cv::isContourConvex(approx))
      rectangles.push_back(approx);
  }
  
  for(int i = 0; i < rectangles.size(); i++)
  {
    for(int j = 0; j < i; j++)
    {
      std::vector<cv::Point> marker1;
      std::vector<cv::Point> marker2;
      std::vector<cv::Point> marker2new;

      marker1 = rectangles[i];
      marker2 = rectangles[j];
      for(int k = 0; k < 4; k++)
      {
        double minDist = 1e100;
        int minIndex = 0;
        for(int l = 0; l < 4; l++)
        {
          double distance = 0.0;
          distance += (marker1[k].x - marker2[l].x) * (marker1[k].x - marker2[l].x);
          distance += (marker1[k].y - marker2[l].y) * (marker1[k].y - marker2[l].y);
          distance = std::sqrt(distance);
          if(distance < minDist)
          {
            minDist = distance;
            minIndex = l;
          }
        }
        marker2new.push_back(marker2[minIndex]);
      }
      marker2 = marker2new;
      
      bool match1 = true;
      double maxDist = std::max(cv::arcLength(marker1, true), cv::arcLength(marker2, true)) / 8.0;
      for(int k = 0; k < 4; k++)
      {
        double distance = 0.0;
        distance += (marker1[k].x - marker2[k].x) * (marker1[k].x - marker2[k].x);
        distance += (marker1[k].y - marker2[k].y) * (marker1[k].y - marker2[k].y);
        distance = std::sqrt(distance);
        if(distance > maxDist)
          match1 = false;
      }
      bool match3 = true;
      cv::Point centroid1 = calcCentroid(marker1);
      cv::Point centroid2 = calcCentroid(marker2);
      if(cv::norm(centroid1-centroid2) > std::max(cv::arcLength(marker1, true), cv::arcLength(marker2, true))*0.05)
        match3 = false;
      bool match5 = true;
      for(int k = 0; k < 4; k++)
      {
        if(marker1[k].x < 10 || marker1[k].x > xsize-10)
          match5 = false;
        if(marker1[k].y < 10 || marker1[k].y > ysize-10)
          match5 = false;
        if(marker2[k].x < 10 || marker2[k].x > xsize-10)
          match5 = false;
        if(marker2[k].y < 10 || marker2[k].y > ysize-10)
          match5 = false;
      }
      
      if(match1 && match3 && match5)
      {
        markers.push_back(marker1);
        markers.push_back(marker2);
      }
    }
  }
}

void FiducalApp::refineMarkers(const cv::Mat &grayscale, std::vector<std::vector<cv::Point> > &markers, std::vector<cv::Point2f> &outerMarkers, std::vector<cv::Point2f> &innerMarkers)
{
  outerMarkers.resize(4, cv::Point2f(0.0, 0.0));
  innerMarkers.resize(4, cv::Point2f(0.0, 0.0));
  if(std::fabs(cv::arcLength(markers[0], true)) > std::fabs(cv::arcLength(markers[1], true)))
  {
    for(int i = 0; i < 4; i++)
    {
      outerMarkers[i].x = markers[0][i].x;
      outerMarkers[i].y = markers[0][i].y;
      innerMarkers[i].x = markers[1][i].x;
      innerMarkers[i].y = markers[1][i].y;
    }
  }
  else 
  {
    for(int i = 0; i < 4; i++)
    {
      outerMarkers[i].x = markers[1][i].x;
      outerMarkers[i].y = markers[1][i].y;
      innerMarkers[i].x = markers[0][i].x;
      innerMarkers[i].y = markers[0][i].y;
    }
  }
  
  cv::cornerSubPix(grayscale, outerMarkers, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 10, 1e-10));
  cv::cornerSubPix(grayscale, innerMarkers, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 10, 1e-10));
}

void FiducalApp::orderMarkers(const cv::Mat &grayscale, std::vector<cv::Point2f> &outerMarkers, std::vector<cv::Point2f> &innerMarkers)
{
  double resizeFactor = 0.2;
  cv::Mat grayscaleSmall;
  cv::resize(grayscale, grayscaleSmall, cv::Size(0, 0), resizeFactor, resizeFactor);

  cv::Mat mask1 = cv::Mat::zeros(grayscaleSmall.size(), CV_8UC1);
  cv::Mat mask2 = cv::Mat::zeros(grayscaleSmall.size(), CV_8UC1);
  cv::Mat mask3 = cv::Mat::zeros(grayscaleSmall.size(), CV_8UC1);
  cv::Mat mask4 = cv::Mat::zeros(grayscaleSmall.size(), CV_8UC1);
  
  cv::Point mask1p[4];
  cv::Point mask2p[4];
  cv::Point mask3p[4];
  cv::Point mask4p[4];
  
  mask1p[0] = innerMarkers[0] + (innerMarkers[2] - innerMarkers[0]) * 0.1;
  mask1p[1] = innerMarkers[0] + (innerMarkers[1] - innerMarkers[0]) * 0.5 + (innerMarkers[3] - innerMarkers[1]) * 0.1;
  mask1p[2] = innerMarkers[0] + (innerMarkers[2] - innerMarkers[0]) * 0.5 + (innerMarkers[0] - innerMarkers[2]) * 0.1;
  mask1p[3] = innerMarkers[0] + (innerMarkers[3] - innerMarkers[0]) * 0.5 + (innerMarkers[1] - innerMarkers[3]) * 0.1;
  
  mask2p[0] = innerMarkers[1] + (innerMarkers[3] - innerMarkers[1]) * 0.1;
  mask2p[1] = innerMarkers[1] + (innerMarkers[2] - innerMarkers[1]) * 0.5 + (innerMarkers[0] - innerMarkers[2]) * 0.1;
  mask2p[2] = innerMarkers[1] + (innerMarkers[3] - innerMarkers[1]) * 0.5 + (innerMarkers[1] - innerMarkers[3]) * 0.1;
  mask2p[3] = innerMarkers[1] + (innerMarkers[0] - innerMarkers[1]) * 0.5 + (innerMarkers[2] - innerMarkers[0]) * 0.1;
  
  mask3p[0] = innerMarkers[2] + (innerMarkers[0] - innerMarkers[2]) * 0.1;
  mask3p[1] = innerMarkers[2] + (innerMarkers[3] - innerMarkers[2]) * 0.5 + (innerMarkers[1] - innerMarkers[3]) * 0.1;
  mask3p[2] = innerMarkers[2] + (innerMarkers[0] - innerMarkers[2]) * 0.5 + (innerMarkers[2] - innerMarkers[0]) * 0.1;
  mask3p[3] = innerMarkers[2] + (innerMarkers[1] - innerMarkers[2]) * 0.5 + (innerMarkers[3] - innerMarkers[1]) * 0.1;
  
  mask4p[0] = innerMarkers[3] + (innerMarkers[1] - innerMarkers[3]) * 0.1;
  mask4p[1] = innerMarkers[3] + (innerMarkers[0] - innerMarkers[3]) * 0.5 + (innerMarkers[2] - innerMarkers[0]) * 0.1;
  mask4p[2] = innerMarkers[3] + (innerMarkers[1] - innerMarkers[3]) * 0.5 + (innerMarkers[3] - innerMarkers[1]) * 0.1;
  mask4p[3] = innerMarkers[3] + (innerMarkers[2] - innerMarkers[3]) * 0.5 + (innerMarkers[0] - innerMarkers[2]) * 0.1;

  for(int i = 0; i < 4; i++)
  {
    mask1p[i] *= resizeFactor;
    mask2p[i] *= resizeFactor;
    mask3p[i] *= resizeFactor;
    mask4p[i] *= resizeFactor;
  }
  
  cv::fillConvexPoly(mask1, mask1p, 4, 255);
  cv::fillConvexPoly(mask2, mask2p, 4, 255);
  cv::fillConvexPoly(mask3, mask3p, 4, 255);
  cv::fillConvexPoly(mask4, mask4p, 4, 255);

  double mean1 = cv::mean(grayscaleSmall, mask1)[0];
  double mean2 = cv::mean(grayscaleSmall, mask2)[0];
  double mean3 = cv::mean(grayscaleSmall, mask3)[0];
  double mean4 = cv::mean(grayscaleSmall, mask4)[0];


  std::vector<cv::Point2f> temp(4, cv::Point2f(0.0, 0.0));
  if(mean1 < mean2 && mean1 < mean3 && mean1 < mean4)
  {
    temp[0] = outerMarkers[0];
    temp[1] = outerMarkers[1];
    temp[2] = outerMarkers[2];
    temp[3] = outerMarkers[3];
    outerMarkers = temp;
    temp[0] = innerMarkers[0];
    temp[1] = innerMarkers[1];
    temp[2] = innerMarkers[2];
    temp[3] = innerMarkers[3];
  }
  else if(mean2 < mean1 && mean2 < mean3 && mean2 < mean4)
  {
    temp[0] = outerMarkers[1];
    temp[1] = outerMarkers[2];
    temp[2] = outerMarkers[3];
    temp[3] = outerMarkers[0];
    outerMarkers = temp;
    temp[0] = innerMarkers[1];
    temp[1] = innerMarkers[2];
    temp[2] = innerMarkers[3];
    temp[3] = innerMarkers[0];
    innerMarkers = temp;
  }
  else if(mean3 < mean1 && mean3 < mean2 && mean3 < mean4)
  {
    temp[0] = outerMarkers[2];
    temp[1] = outerMarkers[3];
    temp[2] = outerMarkers[0];
    temp[3] = outerMarkers[1];
    outerMarkers = temp;
    temp[0] = innerMarkers[2];
    temp[1] = innerMarkers[3];
    temp[2] = innerMarkers[0];
    temp[3] = innerMarkers[1];
    innerMarkers = temp;
  }
  else 
  {
    temp[0] = outerMarkers[3];
    temp[1] = outerMarkers[0];
    temp[2] = outerMarkers[1];
    temp[3] = outerMarkers[2];
    outerMarkers = temp;
    temp[0] = innerMarkers[3];
    temp[1] = innerMarkers[0];
    temp[2] = innerMarkers[1];
    temp[3] = innerMarkers[2];
    innerMarkers = temp;
  }
}

void FiducalApp::doGeometry(const std::vector<cv::Point2f> &outerMarkers, const std::vector<cv::Point2f> &innerMarkers, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Mat &rvec, cv::Mat &tvec)
{
  std::vector<cv::Point2f> imagePoints;
  for(int i = 0; i < 4; i++)
    imagePoints.push_back(outerMarkers[i]);
  for(int i = 0; i < 4; i++)
    imagePoints.push_back(innerMarkers[i]);
  std::vector<cv::Point3f> objectPoints;
  objectPoints.push_back(cv::Point3f(7.5, 7.5, 0.0));
  objectPoints.push_back(cv::Point3f(7.5, -7.5, 0.0));
  objectPoints.push_back(cv::Point3f(-7.5, -7.5, 0.0));
  objectPoints.push_back(cv::Point3f(-7.5, 7.5, 0.0));
  objectPoints.push_back(cv::Point3f(4.5, 4.5, 0.0));
  objectPoints.push_back(cv::Point3f(4.5, -4.5, 0.0));
  objectPoints.push_back(cv::Point3f(-4.5, -4.5, 0.0));
  objectPoints.push_back(cv::Point3f(-4.5, 4.5, 0.0));
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
}

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
