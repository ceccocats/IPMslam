/*
 * Project:  Inverse Perspective Mapping
 *
 * File:     main.cpp
 *
 * Contents: Creation, initialisation and usage of IPM object
 *           for the generation of Inverse Perspective Mappings of images or videos
 *
 * Author:   Marcos Nieto <marcos.nieto.doncel@gmail.com>
 * Date:	 22/02/2014
 * Homepage: http://marcosnietoblog.wordpress.com/
 */

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <ctime>
#include <fstream>
using namespace cv;

#include "IPM.h"

using namespace cv;
using namespace std;

const double y_shift = 0;
const double x_shift = 0;

struct odom_t {
	float x, y, yaw, speed, yawrate, accel; 
};

struct particle_t {
	float x, y, yaw, score;
};
const int N_PART = 1000;

void undistortImage(Mat & image, Mat & outputImage, 
    Mat & cameraMatrix, Mat & distCoeffs) {

    // setup enlargement and offset for new image
    Size imageSize = image.size();
    imageSize.height += 2*y_shift;
    imageSize.width += 2*x_shift;

    // create a new camera matrix with the principal point 
    // offest according to the offset above
    Mat newCameraMatrix = cameraMatrix.clone();
    newCameraMatrix.at<double>(0, 2) += x_shift; //adjust c_x by x_shift
    newCameraMatrix.at<double>(1, 2) += y_shift; //adjust c_y by y_shift

    // create undistortion maps
    Mat map1;
    Mat map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), 
        newCameraMatrix, imageSize, CV_16SC2, map1, map2);

    //remap
    remap(image, outputImage, map1, map2, INTER_LINEAR);
}

int main( int _argc, char** _argv )
{
	// Images
	cv::Mat frame;
	Mat inputImg, inputImgGray, calibImg, showImg0;
	Mat top, outputImg, occgrid;
	
	if( _argc != 4 )
	{
		cout << "Usage: ipm.exe <videofile> <calib.yml> <data.poses>" << endl;
		return 1;
	}	
	string videoFileName  = _argv[1];	
	string yml_filename   = _argv[2];	
	string poses_filename = _argv[3];	

	// Video
	cv::VideoCapture video;
	if( !video.open(videoFileName) )
		return 1;

	// Show video information
	int ratio = 1;
	int width = 0, height = 0, fps = 0, fourcc = 0;	
	width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH) + x_shift*2) / ratio;
	height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT) + y_shift*2) / ratio;
	fps = static_cast<int>(video.get(CV_CAP_PROP_FPS));
	fourcc = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));

	cout << "Input video: (" << width << "x" << height << ") at " << fps << ", fourcc = " << fourcc << endl;

	int slider[5];
	slider[0] = 955;
	slider[1] = 472;
	slider[2] = 178;
	slider[3] = 74;
	slider[4] = 1;//166;
    /*
	slider[0] = 1500;
	slider[1] = 580;
	slider[2] = 1040;
	slider[3] = 817;
	slider[4] = 120;
    */
	/// Create Windows
	namedWindow("roadslam", 1);
	createTrackbar( "s0", "roadslam", &slider[0], width*4, NULL );
	createTrackbar( "s1", "roadslam", &slider[1], height*2, NULL );
	createTrackbar( "s2", "roadslam", &slider[2], width*2, NULL );
	createTrackbar( "s3", "roadslam", &slider[3], width*2, NULL );
	createTrackbar( "s4", "roadslam", &slider[4], 255, NULL );

	// calib
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
    FileStorage fs;
    fs.open(yml_filename, FileStorage::READ);
    if( !fs.isOpened() ){
        cerr << " Fail to open " << yml_filename << endl;
        exit(EXIT_FAILURE);
    }
    // Get camera parameters
    fs["camera_matrix"] >> cameraMatrix;
    fs["dist_coefs"] >> distCoeffs; 
    // Print out the camera parameters
    cout << "\n -- Camera parameters -- " << endl;
    cout << "\n CameraMatrix = " << endl << " " << cameraMatrix << endl << endl;
    cout << " Distortion coefficients = " << endl << " " << distCoeffs << endl << endl;
	
	IPM *ipm = NULL;

	// poses
	std::vector<odom_t> odoms;

	std::ifstream fposes;
	fposes.open(poses_filename.c_str());
	if(!fposes.is_open()) {
		std::cout<<"cant open file: "<<poses_filename<<"\n";
		return 1;
	}
	std::string trash;	
	while(fposes>>trash) {
		
		int n_frame;
		fposes>>n_frame;
		std::cout<<trash<<n_frame<<"\n";

		odom_t o;
		fposes>>o.x
		      >>o.y
			  >>o.yaw
			  >>o.speed
			  >>o.yawrate
			  >>o.accel;
		odoms.push_back(o);
	}

	// global map
	Mat gOG = cv::Mat::zeros(cv::Size(4000,4000), CV_8U);

	Point2f pose(gOG.cols*0.7, gOG.rows*0.7);
	float yaw = -M_PI/2;

	particle_t particles[N_PART];
	for(int i=0; i<N_PART; i++) {
		particles[i].x = pose.x;
		particles[i].y = pose.y;
		particles[i].yaw = yaw;
		particles[i].score = 0;
	}

	// Main loop
	int frameNum = 0;
	for( ; ; )
	{
		printf("FRAME #%6d %4.3f", frameNum, odoms[frameNum].speed);
		fflush(stdout);
		frameNum++;


		// The 4-points at the input image	
		vector<Point2f> origPoints;
		origPoints.push_back( Point2f(width/2 - slider[0],  slider[2]) );
		origPoints.push_back( Point2f(width/2 + slider[0],  slider[2]) );
		origPoints.push_back( Point2f(width/2 + slider[1],  slider[3]) );
		origPoints.push_back( Point2f(width/2 - slider[1],  slider[3]) );

		// The 4-points correspondences in the destination image
		vector<Point2f> dstPoints;
		dstPoints.push_back( Point2f(0, height) );
		dstPoints.push_back( Point2f(width, height) );
		dstPoints.push_back( Point2f(width, 0) );
		dstPoints.push_back( Point2f(0, 0) );
			
		// IPM object
		if(ipm == NULL)
			ipm = new IPM( Size(width, height), Size(width, height), origPoints, dstPoints );


		// Get current image		
		video >> frame;
		if( frame.empty() )
			break;
		//frame = imread(img_filename);

		if(frameNum < 100)
			continue;

		undistort(frame, inputImg, cameraMatrix, distCoeffs);
		

		// Color Conversion
		cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);				 		 	
	

		 // Process
		clock_t begin = clock();
		ipm->applyHomography( inputImgGray, top );		 
		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		printf("%.2f (ms)\r", 1000*elapsed_secs);
		ipm->drawPoints(origPoints, inputImg );
		

		resize(top, outputImg, Size(), 0.05, 0.05);
		blur(outputImg, occgrid, Size(3, 3));
		/*
		//Canny(outputImg, top, 40, 100, 3);
		//threshold(top, outputImg, slider[4], 255, CV_THRESH_BINARY  CV_THRESH_OTSU);
		*/
		//threshold(occgrid, occgrid, slider[4], 255, CV_THRESH_BINARY);
		adaptiveThreshold(occgrid, occgrid, 255, CV_ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,-slider[4]);
		
		// View		
		resize(inputImg, showImg0, Size(), 0.3, 0.3);
		imshow("roadslam", showImg0);

/*
		Mat occgrid_big;
	    copyMakeBorder(occgrid, occgrid_big, occgrid.rows, occgrid.rows, occgrid.cols/2, occgrid.cols/2,BORDER_CONSTANT, 0);

		Point2f src_center(occgrid_big.cols/2.0F, occgrid_big.rows/2.0F + occgrid.rows/2.0F);
		Mat rot_mat = getRotationMatrix2D(src_center, 0, 1.0);
		Mat occgrid_r;
		warpAffine(occgrid_big, occgrid_r, rot_mat, occgrid_r.size());
		imshow("output", occgrid_r);
*/

		const float dt = 0.033;
		const float mt2grid = 90.0/8.0;

		float score_sum = 0;
		float max_score = 0;
		int best_part = 0;

		#pragma omp parallel for
		for(int p=0; p<N_PART; p++) {
			particle_t *part = &particles[p];
			part->score = 0;

			double rx   = (((double) rand() / (RAND_MAX)) - 0.5f)*0.2f*mt2grid;
			double ry   = (((double) rand() / (RAND_MAX)) - 0.5f)*0.2f*mt2grid;
			double ryaw = (((double) rand() / (RAND_MAX)) - 0.5f)*0.005f;

			part->x += cos(yaw)*odoms[frameNum].speed*dt*mt2grid + rx;
			part->y += sin(yaw)*odoms[frameNum].speed*dt*mt2grid + ry;
			part->yaw = -odoms[frameNum].yaw + ryaw;

			int pi = part->y;
			int pj = part->x;
			double yaw_sin = sin(yaw);
			double yaw_cos = cos(yaw);
			for(int i=0; i<occgrid.rows; i++) {
				for(int j=0; j<occgrid.cols; j++) {
					uint8_t val = occgrid.data[i*occgrid.cols + j];
					double lx = j - occgrid.cols/2;
					double ly = i - occgrid.rows - 7.45*mt2grid;
					int li = lx*yaw_cos - ly*yaw_sin;
					int lj = lx*yaw_sin + ly*yaw_cos;
					int gi = pi + li;
					int gj = pj - lj;	
					uint8_t gval = gOG.data[ gi*gOG.cols + gj];

					part->score += (255 - abs(gval - val));
				}
			}

			#pragma omp critical
			{
			score_sum += part->score;
			if(part->score > max_score) {
				max_score = part->score;
				best_part = p;
			}
		}
		}
		std::cout<<"best score: "<<max_score<<", sum: "<<score_sum<<"\n";

		// update best pose 
		pose.x = particles[best_part].x;
		pose.y = particles[best_part].y;
		yaw    = particles[best_part].yaw;

		#pragma omp parallel sections
		{
			#pragma omp section
			{ 
				// write data on global map
				int pi = pose.y;
				int pj = pose.x;
				double yaw_sin = sin(yaw);
				double yaw_cos = cos(yaw);
				for(int i=0; i<occgrid.rows; i++) {
					for(int j=0; j<occgrid.cols; j++) {
						uint8_t val = occgrid.data[i*occgrid.cols + j];
						double lx = j - occgrid.cols/2;
						double ly = i - occgrid.rows - 7.5*mt2grid;
						int li = lx*yaw_cos - ly*yaw_sin;
						int lj = lx*yaw_sin + ly*yaw_cos;
						int gi = pi + li;
						int gj = pj - lj;	
						uint8_t gval = gOG.data[ gi*gOG.cols + gj];
						if(val > 0 && gval < (255-20))
							gOG.data[ gi*gOG.cols + gj] += 20;
						if(val <= 0 && gval > 5)
							gOG.data[ gi*gOG.cols + gj] -= 5;
					}
				}
			}

			#pragma omp section
			{ 
				// resampling 
				int resampled = 0;
				for(int p=0; p<N_PART; p++) {
					particle_t *part = &particles[p];
					if(part->score < score_sum/N_PART) {
						resampled++;
						particles[p] = particles[best_part];
					}
				}
				std::cout<<"resampled: "<<resampled<<"\n";
			}
		}
	

		//circle(gOG, Point(pj, pi), 10, Scalar(255, 0, 0));
		imshow("local map", occgrid);

				
		// View		
		resize(gOG, showImg0, Size(), 0.25, 0.25);
		imshow("global map", showImg0);

		int key = waitKey(1);
		if(key == 114)
			ipm = NULL;

	}
	imwrite("map.png", gOG);

	return 0;	
}		
