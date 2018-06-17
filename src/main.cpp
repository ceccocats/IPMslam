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
const int N_PART = 100 ;

const double CAM_DST_F = 7.45;
const double CAM_DST_B = 7.45;

int main( int _argc, char** _argv )
{

	
	if( _argc != 5 )
	{
		cout << "Usage: ipm.exe <videofile_front> <videofile_back> <calib.yml> <data.poses>" << endl;
		return 1;
	}	
	string videoFileFront  = _argv[1];	
	string videoFileBack   = _argv[2];
	string yml_filename    = _argv[3];	
	string poses_filename  = _argv[4];	

	// Video
	cv::VideoCapture video;
	if( !video.open(videoFileFront) )
		return 1;
	
	cv::VideoCapture video_b;
	if( !video_b.open(videoFileBack) )
		return 1;

	// Show video information
	int ratio = 1;
	int width = 0, height = 0, fps = 0, fourcc = 0;	
	width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH) + x_shift*2) / ratio;
	height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT) + y_shift*2) / ratio;
	fps = static_cast<int>(video.get(CV_CAP_PROP_FPS));
	fourcc = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));

	cout << "Input video: (" << width << "x" << height << ") at " << fps << ", fourcc = " << fourcc << endl;

	int thresh_slider = 1;
	/// Create Windows
	namedWindow("roadslam", 1);
	createTrackbar( "thresh", "roadslam", &thresh_slider, 100, NULL );
	
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
	
	// Get IPM params
	cv::Mat ipm_fc, ipm_bc;
	fs["ipm_front"] >> ipm_fc;
	fs["ipm_back"] >> ipm_bc;
	cout<<ipm_fc<<"\n";
	cout<<ipm_bc<<"\n";
	
	// The 4-points at the input front image	
	vector<Point2f> origPoints_f;
	origPoints_f.push_back( Point2f(width/2 - ipm_fc.at<double>(0),  ipm_fc.at<double>(2) ));
	origPoints_f.push_back( Point2f(width/2 + ipm_fc.at<double>(0),  ipm_fc.at<double>(2) ));
	origPoints_f.push_back( Point2f(width/2 + ipm_fc.at<double>(1),  ipm_fc.at<double>(3) ));
	origPoints_f.push_back( Point2f(width/2 - ipm_fc.at<double>(1),  ipm_fc.at<double>(3) ));
	
	// The 4-points at the input back image	
	vector<Point2f> origPoints_b;
	origPoints_b.push_back( Point2f(width/2 - ipm_bc.at<double>(0),  ipm_bc.at<double>(2) ));
	origPoints_b.push_back( Point2f(width/2 + ipm_bc.at<double>(0),  ipm_bc.at<double>(2) ));
	origPoints_b.push_back( Point2f(width/2 + ipm_bc.at<double>(1),  ipm_bc.at<double>(3) ));
	origPoints_b.push_back( Point2f(width/2 - ipm_bc.at<double>(1),  ipm_bc.at<double>(3) ));
	
	// The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back( Point2f(0, height) );
	dstPoints.push_back( Point2f(width, height) );
	dstPoints.push_back( Point2f(width, 0) );
	dstPoints.push_back( Point2f(0, 0) );

	IPM *ipm_f = new IPM( Size(width, height), Size(width, height), origPoints_f, dstPoints );
	IPM *ipm_b = new IPM( Size(width, height), Size(width, height), origPoints_b, dstPoints );



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


	// Images
	Mat frame_f, frame_b;
	Mat input_f, input_b;
	Mat grey_f,  grey_b;
	Mat top_f,   top_b;
	Mat topr_f,  topr_b;
	Mat og_f,    og_b;
	Mat cv_map;

	// Main loop
	int frameNum = 0;
	for( ; ; )
	{
		printf("FRAME #%6d %4.3f", frameNum, odoms[frameNum].speed);
		fflush(stdout);
		frameNum++;
			
	
		// Get current image		
		video   >> frame_f;
		video_b >> frame_b; 
		if( frame_f.empty() || frame_b.empty() )
			break;
			

		if(frameNum < 2000)
			continue;

		undistort(frame_f, input_f, cameraMatrix, distCoeffs);
		
		// Color Conversion
		cvtColor(input_f, grey_f, CV_BGR2GRAY);				 		 	

		 // Process IPM
		ipm_f->applyHomography( grey_f, top_f );		 
		
		// thresh
		resize(top_f, topr_f, Size(), 0.05, 0.05);
		blur(topr_f, og_f, Size(3, 3));
		adaptiveThreshold(og_f, og_f, 255, CV_ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,-thresh_slider);
		
		// process back
		undistort(frame_b, input_b, cameraMatrix, distCoeffs);
		cvtColor(input_b, grey_b, CV_BGR2GRAY);
		ipm_b->applyHomography( grey_b, top_b );
		resize(top_b, topr_b, Size(), 0.05, 0.05);
		blur(topr_b, og_b, Size(3, 3));
		adaptiveThreshold(og_b, og_b, 255, CV_ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,-thresh_slider);
		

		// View		
		imshow("roadslam", input_f);
		imshow("roadslamb", input_b);

		imshow("local map front", og_f);
		imshow("local map back", og_b);


		const float dt = 0.033;
		const float mt2grid = 90.0/8.0;

		float score_sum = 0;
		float max_score = 0;
		int best_part = 0;

		#pragma omp parallel for
		for(int p=0; p<N_PART; p++) {
			particle_t *part = &particles[p];
			part->score = 0;

			double rx   = (((double) rand() / (RAND_MAX)) - 0.5f)*0.1f*mt2grid;
			double ry   = (((double) rand() / (RAND_MAX)) - 0.5f)*0.1f*mt2grid;
			double ryaw = (((double) rand() / (RAND_MAX)) - 0.5f)*0.01f;

			part->yaw = -odoms[frameNum].yaw + ryaw;
			double yaw_sin = sin(part->yaw);
			double yaw_cos = cos(part->yaw);
			double yawb_sin = sin(part->yaw - M_PI);
			double yawb_cos = cos(part->yaw - M_PI);

			part->x += yaw_cos*odoms[frameNum].speed*dt*mt2grid + rx;
			part->y += yaw_sin*odoms[frameNum].speed*dt*mt2grid + ry;

			int pi = part->y;
			int pj = part->x;
			for(int i=0; i<og_f.rows; i++) {
				for(int j=0; j<og_f.cols; j++) {

					// front
					{
						uint8_t val = og_f.data[i*og_f.cols + j];
						double lx = j - og_f.cols/2;
						double ly = i - og_f.rows - CAM_DST_F*mt2grid;
						int li = lx*yaw_cos - ly*yaw_sin;
						int lj = lx*yaw_sin + ly*yaw_cos;
						int gi = pi + li;
						int gj = pj - lj;	
						uint8_t gval = gOG.data[ gi*gOG.cols + gj];

						part->score += (255 - abs(gval - val));
					}
					// back
					{
						uint8_t val = og_b.data[i*og_f.cols + j];
						double lx = j - og_b.cols/2;
						double ly = i - og_b.rows - CAM_DST_B*mt2grid;
						int li = lx*yawb_cos - ly*yawb_sin;
						int lj = lx*yawb_sin + ly*yawb_cos;
						int gi = pi + li;
						int gj = pj - lj;	
						uint8_t gval = gOG.data[ gi*gOG.cols + gj];

						part->score += (255 - abs(gval - val));
					}
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
				double yawb_sin = sin(yaw - M_PI);
				double yawb_cos = cos(yaw - M_PI);
				for(int i=0; i<og_f.rows; i++) {
					for(int j=0; j<og_f.cols; j++) {

						// front 
						{
							uint8_t val = og_f.data[i*og_f.cols + j];
							double lx = j - og_f.cols/2;
							double ly = i - og_f.rows - CAM_DST_F*mt2grid;
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

						// back 
						{
							uint8_t val = og_b.data[i*og_b.cols + j];
							double lx = j - og_b.cols/2;
							double ly = i - og_b.rows - CAM_DST_B*mt2grid;
							int li = lx*yawb_cos - ly*yawb_sin;
							int lj = lx*yawb_sin + ly*yawb_cos;
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
	
				
		// View		
		resize(gOG, cv_map, Size(), 0.25, 0.25);
		imshow("global map", cv_map);

		int key = waitKey(1);
	}
	imwrite("map.png", gOG);

	return 0;	
}		
