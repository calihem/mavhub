#include "OpticalFlow.h"

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)

//#include "Settings.h"
//#include "highgui.h"
#include <iostream>
#include <stdio.h>

using namespace std;
int alt=0, hor=0,vertical=0, rot=0;

FILE *logg=fopen("log.log","w") ;

UnwrapSettings::UnwrapSettings(int cx, int cy, int ri, int ro, int im, double sx = 1, double sy = 1, int fw = 0, int fh = 0) {
	this->cx = cx;
	this->cy = cy;
	this->ri = ri;
	this->ro = ro;
	this->im = im;
	this->sx = sx;
	this->sy = sy;
	this->fw = fw;
	this->fh = fh;
}

void OpticalFlow::visualizeMean(int sectors, cv::Mat &image) const {
	CvPoint p,q;
	int dx, dy;
	// int avgx=0, avgy=0;
	const CvScalar color = CV_RGB(255,0,0);
	const double pi = 3.14159265358979323846;
	double angle;

	int sectorHeight = image.rows / sectors;
	int sectorWidth = image.cols /sectors;
	const int8_t scaleFactor = max(3, image.cols/200);
	const int lineThickness = max(2, image.cols/300);

	for(int i=0;i<sectors;i++) {//for every sector in row
		for(int j=0;j<sectors;j++) {//for every sector in col
			dx = getMeanVelX(max(sectorWidth*i, 1),
											 sectorWidth*(i+1)-1,
											 max(sectorHeight*j, 1),
											 sectorHeight*(j+1)-1);
			dy = getMeanVelY(max(sectorWidth*i, 1),
											 sectorWidth*(i+1)-1,
											 max(sectorHeight*j, 1),
											 sectorHeight*(j+1)-1);

			// avgy+=dy;
			p.x = sectorWidth*i+(sectorWidth/2);
			p.y = sectorHeight*j+(sectorHeight/2);
			q.x = p.x - scaleFactor*dx;
			q.y = p.y - scaleFactor*dy;
			cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );

			// arrow head
			if(dx || dy) {
				angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
				p.x = (int) (q.x + 9 * cos(angle + pi / 4));
				p.y = (int) (q.y + 9 * sin(angle + pi / 4));
				cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
				p.x = (int) (q.x + 9 * cos(angle - pi / 4));
				p.y = (int) (q.y + 9 * sin(angle - pi / 4));
				cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
			}
		}
	}
}

void OpticalFlow::visualizeMeanXY(int sectorsx, int sectorsy, cv::Mat &image) const {
	CvPoint p,q;
	int dx, dy;
	// int avgx=0, avgy=0;
	const CvScalar color = CV_RGB(255,0,0);
	const double pi = 3.14159265358979323846;
	double angle;

	int sectorHeight = image.rows / sectorsy;
	int sectorWidth = image.cols / sectorsx;
	const int8_t scaleFactor = max(3, image.cols/50);
	const int lineThickness = max(2, image.cols/300);

	// cout << "visualizeMeanXY: log gehts: " << sectorHeight << ", " << sectorWidth << endl;
	// cout << "visualizeMeanXY: log gehts: " << image.rows << ", " << image.cols << endl;

	for(int i=0;i<sectorsx;i++) {//for every sector in row
		for(int j=0;j<sectorsy;j++) {//for every sector in col
			// cout << "hier 1: " << i << ", " << j << endl;
			dx = getMeanVelX(max(sectorWidth*i, 1),
											 sectorWidth*(i+1)-1,
											 max(sectorHeight*j, 1),
											 sectorHeight*(j+1)-1);
			dy = getMeanVelY(max(sectorWidth*i, 1),
											 sectorWidth*(i+1)-1,
											 max(sectorHeight*j, 1),
											 sectorHeight*(j+1)-1);
	// 		// cout << "hier 2" << endl;
			// avgy+=dy;
			p.x = sectorWidth*i+(sectorWidth/2);
			p.y = sectorHeight*j+(sectorHeight/2);
			q.x = p.x - scaleFactor*dx;
			q.y = p.y - scaleFactor*dy;
			//cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
			cv::line( image, p, q, color, lineThickness, CV_AA, 0 );

			//arrow head
			if(dx || dy) {
				angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
				p.x = (int) (q.x + 9 * cos(angle + pi / 4));
				p.y = (int) (q.y + 9 * sin(angle + pi / 4));
				// cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
				cv::line(image, p, q, color, lineThickness, CV_AA, 0 );
				p.x = (int) (q.x + 9 * cos(angle - pi / 4));
				p.y = (int) (q.y + 9 * sin(angle - pi / 4));
				// cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
				cv::line(image, p, q, color, lineThickness, CV_AA, 0 );
			}
		}
	}
}

DenseOpticalFlow::~DenseOpticalFlow() {
	cvReleaseMat(&velX);
	cvReleaseMat(&velY);
	cvReleaseMat(&velXf);
	cvReleaseMat(&velYf);
}

int DenseOpticalFlow::getMeanVelX(int x0, int x1, int y0, int y1) const {
	int32_t sum_x = 0;	//sum of vectors
	int16_t num_x = 0;	//number of vectors which are not null

	signed char vel;

	for(int y=y0;y<y1;y++) {//for every row in range
		for(int x=x0;x<x1;x++) {//for every col in range
			vel = CV_MAT_ELEM(*velX, char, y, x);
			if(vel) {
				num_x++;
				sum_x += vel;
			}
		}
	}

	if(num_x) {
		return sum_x / num_x;
	}

	return 0;
}

int DenseOpticalFlow::getMeanVelY(int x0, int x1, int y0, int y1) const {
	int32_t sum_y = 0;	//sum of vectors
	int16_t num_y = 0;	//number of vectors which are not null

	signed char vel;

	for(int y=y0;y<y1;y++) {//for every row in range
		for(int x=x0;x<x1;x++) {//for every col in range
			vel = CV_MAT_ELEM(*velY, char, y, x);
			if(vel) {
				num_y++;
				sum_y += vel;
			}
		}
	}

	if(num_y) {
		return sum_y / num_y;
	}

	return 0;
}

float DenseOpticalFlow::getMeanVelXf(int x0, int x1, int y0, int y1) const {
	float sum_x = 0;	//sum of vectors
	int16_t num_x = 0;	//number of vectors which are not null

	// signed char vel;
	float vel;

        // printf("x0: %d, x1: %d, y0: %d, y1: %d\n", x0, x1, y0, y1);
	for(int y=y0;y<y1;y++) {//for every row in range
		for(int x=x0;x<x1;x++) {//for every col in range
                  // printf("y, rows: %d, %d, x, cols: %d, %d\n", (unsigned)(y),
                  //        (unsigned)(*velXf).rows, (unsigned)(x), (unsigned)(*velXf).cols);
			vel = CV_MAT_ELEM(*velXf, float, y, x);
			if(vel) {
				num_x++;
				sum_x += vel;
			}
		}
	}

	if(num_x) {
		return sum_x / num_x;
	}

	return 0;
}

float DenseOpticalFlow::getMeanVelYf(int x0, int x1, int y0, int y1) const {
	float sum_y = 0;	//sum of vectors
	int16_t num_y = 0;	//number of vectors which are not null

	float vel;

	for(int y=y0;y<y1;y++) {//for every row in range
		for(int x=x0;x<x1;x++) {//for every col in range
			vel = CV_MAT_ELEM(*velYf, float, y, x);
			if(vel) {
				num_y++;
				sum_y += vel;
			}
		}
	}

	if(num_y) {
		return sum_y / num_y;
	}

	return 0;
}

void DenseOpticalFlow::visualizeMeanXYf(int sectorsx, int sectorsy, cv::Mat &image) const {
  CvPoint p,q;
  float dx, dy;
  // int avgx = 0, avgy = 0;
  const CvScalar color = CV_RGB(255,0,0);
  const double pi = 3.14159265358979323846;
  double angle;

  int sectorHeight = image.rows / sectorsy;
  int sectorWidth = image.cols / sectorsx;
  const int8_t scaleFactor = 10; // max(5, image.cols/50);
  const int lineThickness = 2; // max(2, image.cols/300);


  // cout << "visualizeMeanXY: log gehts: " << sectorHeight << ", " << sectorWidth << endl;
  // cout << "visualizeMeanXY: log gehts: " << image.rows << ", " << image.cols << endl;
  // image = cvCreateMat(image.rows, image.cols, CV_32F); // cv::Mat();
  // cv::Mat velXf_t = cv::Mat(velXf, true);
  // velXf_t.copyTo(image);

  for(int i=0;i<sectorsx;i++) {//for every sector in row
    for(int j=0;j<sectorsy;j++) {//for every sector in col
      // cout << "hier 1: " << i << ", " << j << endl;
      dx = getMeanVelXf(max(sectorWidth*i, 1),
                        sectorWidth*(i+1)-1,
                        max(sectorHeight*j, 1),
                        sectorHeight*(j+1)-1);
      dy = getMeanVelYf(max(sectorWidth*i, 1),
                        sectorWidth*(i+1)-1,
                        max(sectorHeight*j, 1),
                        sectorHeight*(j+1)-1);
      // 		// cout << "hier 2" << endl;
      // avgy+=dy;
      p.x = sectorWidth*i+(sectorWidth/2);
      p.y = sectorHeight*j+(sectorHeight/2);
      q.x = p.x - scaleFactor*dx;
      q.y = p.y - scaleFactor*dy;
      //cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
      // printf("%f, %f\n", dx, dy);
      // if(fabs(dx) > 0.6 || fabs(dy) > 0.6) {
        cv::line( image, p, q, color, lineThickness, CV_AA, 0 );

        // arrow head
        // if(dx || dy) {
          angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
          p.x = (int) (q.x + 2 * cos(angle + pi / 4));
          p.y = (int) (q.y + 2 * sin(angle + pi / 4));
          // cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
          cv::line(image, p, q, color, lineThickness, CV_AA, 0 );
          p.x = (int) (q.x + 2 * cos(angle - pi / 4));
          p.y = (int) (q.y + 2 * sin(angle - pi / 4));
          // cvLine( &image, p, q, color, lineThickness, CV_AA, 0 );
          cv::line(image, p, q, color, lineThickness, CV_AA, 0 );
          // }
        // }
    }
  }
  // image = velXf;
  // cv::add(velXf, velYf, image);
}

void DenseOpticalFlow::visualize(cv::Mat &image) const {

//        for(int y=1; y<image.rows-1; y+=3) {
//		for(int x=1; x<image.width-1; x+=3) {
//                    if(_ro*_ro*9/4>(x-_cx)*(x-_cx)+(y-_cy)*(y-_cy)){ // Edit Lóa: Only calculate OF inside circle
//			cvLine(&image,
//			       cvPoint(x, y),
//			       cvPoint(x + CV_MAT_ELEM(*velX, signed char, y, x), y + CV_MAT_ELEM(*velY, signed char, y, x)),
//			       CV_RGB(255,0,0),
//			       1,
//			       8,
//			       0);
//                    }

//		}
//	}

        getDirection(image);

}


int DenseOpticalFlow::getDirection(cv::Mat &image) const{


//    cv::Mat* dirImage = cvCreateImage(cvSize(200,200),8,1);

//    memset(dirImage->imageData,255,dirImage->imageSize);
//    cvLine(dirImage,cvPoint(dirImage->width/2,dirImage->height),cvPoint(dirImage->width/2,1),cvScalar(0));
//    cvLine(dirImage,cvPoint(1,dirImage->height/2),cvPoint(dirImage->width,dirImage->height/2),cvScalar(0));
//    cvLine(dirImage,cvPoint(10,10),cvPoint(10, 180),cvScalar(0));

//    cvNamedWindow("Direction", CV_WINDOW_AUTOSIZE);
//   // looking at 4 different sectors of image

    int y2= (int) image.rows/2;
    int x2 = (int) image.cols/2;


    signed char velx;
    signed char vely;

    double sum_x1 =0, sum_x2 =0, sum_x3 =0,sum_x4 =0;
    double sum_y1 =0, sum_y2=0, sum_y3=0, sum_y4=0;
    int16_t num_x1=0, num_x2 =0,num_x3 =0, num_x4 =0;
    int16_t num_y1 =0, num_y2 =0, num_y3 =0,num_y4 =0;


    // sectors (x<x2,  y<y2), (x>x2, y<y2), (x<x2, y>y2), (x>x2, y>y2)
    for(int y=1; y<y2; y+=3){

        for (int x=1; x<image.cols;x+=3){
            velx =CV_MAT_ELEM(*velX,char, y,x);
            vely =CV_MAT_ELEM(*velY,char, y,x);

            if(velx || vely) {
                    num_x1++;
                    num_y1++;
                    sum_y1 += vely;
                    sum_x1 += velx;
            }

        }
    }

    for (int y=y2; y<image.rows-1;y+=3){
        for (int x=1; x<image.cols-1;x+=3){
            velx =CV_MAT_ELEM(*velX,char, y,x);
            vely =CV_MAT_ELEM(*velY,char, y,x);

            if(velx || vely) {
                    num_x3++;
                    num_y3++;
                    sum_y3 += vely;
                    sum_x3 += velx;


            }
        }

    }

    for(int x=1; x<x2; x+=3){

        for (int y=1; y<image.rows-1;y+=3){
            velx =CV_MAT_ELEM(*velX,char, y,x);
            vely =CV_MAT_ELEM(*velY,char, y,x);

            if(velx || vely) {
                    num_x4++;
                    num_y4++;
                    sum_y4 += vely;
                    sum_x4 += velx;

            }

        }
}


        for (int x=x2; x<image.cols-1;x+=3){
            for (int y=1; y<image.rows-1;y+=3){
            velx =CV_MAT_ELEM(*velX,char, y,x);
            vely =CV_MAT_ELEM(*velY,char, y,x);

            if(velx || vely) {
                    num_x2++;
                    num_y2++;
                    sum_y2 += vely;
                    sum_x2 += velx;


            }

        }
    }



       bool lat1=false, lat2=false, lat3=false, lat4=false;

       if(sum_x1<0 && sum_x2<0 && sum_x3<0 && sum_x4<0)lat1=true; // && sum_x1<sum_y1 && sum_x2<sum_y2 && sum_x3<sum_y3 && sum_x4 < sum_y4)lat1=true;

       if(sum_x1>0 && sum_x2>0 && sum_x3>0 && sum_x4>0)lat2=true;// && sum_x1>sum_y1 && sum_x3>sum_y3 && sum_x2>sum_y2 && sum_x4>sum_y4)lat2=true;


       if(sum_y1<0 && sum_y2<0 && sum_y3<0 && sum_y4<0)lat3=true;// && sum_x1>sum_y1 && sum_x3>sum_y3 && sum_x2>sum_y2 && sum_x4>sum_y4 )lat3=true;


       if(sum_y1>0 && sum_y2>0 && sum_y3>0 && sum_y4>0)lat4=true;// && sum_x1<sum_y1 && sum_x2<sum_y2 && sum_x3<sum_y3 && sum_x4 < sum_y4)lat4=true;

       int sensitivity=100;
       if(num_x1>sensitivity && num_y1>sensitivity && num_x2>sensitivity && num_y2>sensitivity && num_x3>sensitivity && num_y3>sensitivity && num_x4>sensitivity && num_y4>sensitivity){


       if(lat1 && !lat2 && !lat3 && !lat4){
           hor--;
       }
       else if(lat2 && !lat3 && !lat4){
           hor++;
       }
       else if (lat3 && !lat4){
           vertical++;
       }
       else if(lat4){
           vertical--;
       }


}
sensitivity=200;
if(num_x1>sensitivity && num_y1>sensitivity && num_x2>sensitivity && num_y2>sensitivity && num_x3>sensitivity && num_y3>sensitivity && num_x4>sensitivity && num_y4>sensitivity){

    cout<<num_x1 <<", " << num_y1 <<"," <<num_x2 <<", "<< num_y2 <<","<< num_x3 << "," << num_y3 << "," << num_x4 << "," << num_y4<< endl;
        if(sum_y1<0 && sum_x2>0 && sum_y3>0 && sum_x4<0){
//         cout<< "UP"<< endl;
          alt++;
          }

if(sum_y1>0 && sum_x2<0 &&sum_y3<0 && sum_x4>0 ){
//    cout<< "DOWN"<< endl;
    alt--;
    }

if(sum_x1<0 && sum_x3 > 0 && sum_y2<0  &&  sum_y4 >0  ){
//    cout<<"Rotate counterclockwise"<<endl;
                  rot--;
}
if(sum_x1>0 && sum_x3 < 0 && sum_y2>0 && sum_y4 <0 ){
//    cout<<"Rotate clockwise"<< endl;
        rot++;
}
}

fprintf(logg, "alt: %i, hor: %i, vertical: %i rot: %i \n ",alt, hor, vertical,rot);
//printf("alt: %i, hor: %i, vertical: %i rot: %i \n ",alt, hor, vertical,rot);


//cvLine(dirImage,cvPoint(100,100),cvPoint(90*cos(PI*rot/180)+100,90*sin(PI*rot/180)+100),cvScalar(0),1);
//cvLine(dirImage,cvPoint(100,100),cvPoint(-90*cos(PI*rot/180)+100,-90*sin(PI*rot/180)+100),cvScalar(0),1);
//cvLine(dirImage,cvPoint(100,100),cvPoint(90*cos(PI*rot/180+PI/2)+100,90*sin(PI*rot/180+PI/2)+100),cvScalar(0),1);
//cvLine(dirImage,cvPoint(100,100),cvPoint(-90*cos(PI*rot/180+PI/2)+100,-90*sin(PI*rot/180+PI/2)+100),cvScalar(0),1);

//cvCircle(dirImage,cvPoint(10,100-alt),3,cvScalar(0),3);
//cvCircle(dirImage,cvPoint(hor+100,vertical+100),3,cvScalar(0),3);
//cvShowImage("Direction",dirImage);


 return 0;
}

SparseOpticalFlow::~SparseOpticalFlow() {
}

int SparseOpticalFlow::getMeanVelX(int x0, int x1, int y0, int y1) const {
	int32_t sum_x = 0;	//sum of vectors
	int16_t num_x = 0;	//number of vectors which are not null

	for(std::list<Displacement>::const_iterator i = velocity.begin(); i!=velocity.end(); ++i) {
		if( (i->x >= x0) && (i->x < x1) && (i->y >= y0) && (i->y < y1) ) {
			if(i->dx) {
				num_x++;
				sum_x += i->dx;
			}
		}
	}

	if(num_x) {
		return sum_x / num_x;
	}

	return 0;
}

int SparseOpticalFlow::getMeanVelY(int x0, int x1, int y0, int y1) const {
	int32_t sum_y = 0;	//sum of vectors
	int16_t num_y = 0;	//number of vectors which are not null

	for(std::list<Displacement>::const_iterator i = velocity.begin(); i!=velocity.end(); ++i) {
		if( (i->x >= x0) && (i->x < x1) && (i->y >= y0) && (i->y < y1) ) {
			if(i->dy) {
				num_y++;
				sum_y += i->dy;
			}
		}
	}

	if(num_y) {
		return sum_y / num_y;
	}

	return 0;
}

void SparseOpticalFlow::visualize(cv::Mat &image) const {
	int red, green, blue;

	for(std::list<Displacement>::const_iterator i = velocity.begin(); i!=velocity.end(); ++i) {
		CvScalar color;
// 		color = CV_RGB(255,0,0);

		//calculate color with respect to length
		//FIXME: factor should be 255/MAX_VALUE
		red = (255/25)*cvRound( i->length() );
		if(red > 255) red = 255;
		blue = 255 - red;
		if(red < 128)
			green = red;
		else
			green = 255 - red;
		green *= 2;	
		color = CV_RGB(red, green, blue);

		cvLine(&image,
			*i,
			cvPoint(i->x + i->dx, i->y + i->dy),
			color,
			1,
			8,
			0);
	}
}

#endif // (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)
