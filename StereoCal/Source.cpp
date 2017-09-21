#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>
#include <Vec3.h>


#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iomanip>

/*Created for the Ocean Perception Laboratory 
	By Phuc Phung Te 21/09/2017

*/
struct imageVar{
	cv::Mat leftImage;
	cv::Mat rightImage;
	double focalLength;
	double baseLine;
};
using namespace std;

float** laserLineDetector(cv::Mat);
float** laserLineCombined(float**, float**, int);
imageVar StereoCalib(const char*, int, int, bool, cv::Mat, cv::Mat);
void help(char*);
void saveXYZ(const char*, const cv::Mat&);
std::vector<Vec3<double>> XYZSpace(const char*, float**, double, double, int);
double max3(double, double, double);
cv::Vec4d planeFromPoints(std::vector<Vec3<double>>);
std::vector<Vec3<double>> stereoLaserLineImageToPoints(string, string);

//using namespace cv;
//22/02/2017 !




/*int main(int argc, char** argv){
	help(argv);
	int board_w = 7, board_h = 7;
	const char* board_list = "ch12_list.txt";
	if (argc == 4) {
		board_list = argv[1];
		board_w = atoi(argv[2]);
		board_h = atoi(argv[3]);
	}
	StereoCalib(board_list, board_w, board_h, false);

	return 0;
	

}
*/

int main(int argc, char** argv){

	//ofstream outFile("C:\\Users\\phucte\\Desktop\\testing.csv");
	//outFile << "Wind Farm Data, Wind Speed, Power Generated" << endl;
	//outFile.close();

	//0001691
	//0001853
	//0002014
	//0001908
	//0001300
	//0001305
	//0001589
	//0001607
	//0001852
	//0001612


	cout << "\n Created for the Ocean Perception Laboratory \n	By Phuc Phuc Te 21/09/2017 \n \n ";

	string leftImage = "C:\\Users\\phucte\\Desktop\\jpg\\Cam51707923\\0001853.jpg";
	string rightImage = "C:\\Users\\phucte\\Desktop\\jpg\\Cam51707925\\0001853.jpg";

	std::vector<Vec3<double>> points = stereoLaserLineImageToPoints(leftImage, rightImage);

	cv::Vec4d plane = planeFromPoints(points);


	/*
	const char* board_list = "C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\ch12_list.txt";
	FILE*		f = fopen(board_list, "rt");



	if (!f){
		cout << "Cannot open file " << board_list << endl;

		cin.get();

		return 0;
	}
	arrayData.close();
	*/
	
	
	
	

	//cv::waitKey(0);
	return 0;
}

std::vector<Vec3<double>> stereoLaserLineImageToPoints(string leftImage, string rightImage){

	cv::Mat imageLeft = cv::imread(leftImage, 1);
	cv::Mat imageRight = cv::imread(rightImage, 1);

	//cv::imshow("Left Image", imageLeft);




	int imageLeftColumns = imageLeft.cols;
	int imageRightColums = imageRight.cols;

	if (imageLeft.cols != imageRight.cols){
		cout << "Error: Image sizes do not match";
		//cv::waitKey(0);
		//return 0;
	}

	int board_w = 7, board_h = 7;
	const char* board_list = "C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\ch12_list.txt";


	imageVar laserLineImages = StereoCalib(board_list, board_w, board_h, false, imageLeft, imageRight);



	float** laserArrayLeft = laserLineDetector(laserLineImages.leftImage);


	float** laserArrayRight = laserLineDetector(laserLineImages.rightImage);


	float** laserArrayCombined = laserLineCombined(laserArrayLeft, laserArrayRight, imageLeftColumns);

	ofstream arrayData;
	arrayData.open("C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\array.csv");


	for (int i = 0; i < imageLeft.cols; i++){

		//cout << "Laser Line at " << setw(4) << i << " Laser pixel: " << setw(5) << laserArrayLeft[i][1] << " laser Line Sub: " << setw(6) << laserArrayLeft[i][2] << "\n";
		//cout << "Laser Line Left   @ " << setw(4) << i << "  Laser Pixel Left : " << setw(5) << laserArrayCombined[i][1] << " Laser Left  sub : " << setw(6) << laserArrayCombined[i][2] << "\n";
		//cout << "Laser Line Right  @ " << setw(4) << i << "  Laser pixel Right: " << setw(5) << laserArrayCombined[i][3] << " Laser Right sub: " << setw(6) << laserArrayCombined[i][4] << "\n";

		if (laserArrayCombined[i][1] != 0 && laserArrayCombined[i][2] != 0 && laserArrayCombined[i][3] != 0 && laserArrayCombined[i][4] != 0){
			arrayData << i << ", " << laserArrayCombined[i][1] << ", " << laserArrayCombined[i][2] << ", " << laserArrayCombined[i][3] << ", " << laserArrayCombined[i][4] << ", " << laserArrayCombined[i][5] << "\n";
		}
	}

	std::string pointCloudFilename = "C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\pointCloud2.txt";

	std::vector<Vec3<double>> points = XYZSpace(pointCloudFilename.c_str(), laserArrayCombined, laserLineImages.focalLength, laserLineImages.baseLine, imageLeftColumns);

	return points;
}






void help(char* argv[]){

}

 imageVar StereoCalib(
	const char* imageList,
	int			nx,
	int			ny,
	bool		useUncalibrated,
	cv::Mat		leftImage,
	cv::Mat		rightImage

	){
	bool		displayCorners = true;
	bool		showUndistorted = true;
	bool		isVerticalStereo = false; //horiz or vert cams
	const int	maxScale = 1;
	const float squareSize = (float)(0.1); //actual square size (100mm)
	FILE*		f = fopen(imageList, "rt");
	int			i, j, lr;
	int			N = nx*ny;
	vector<string>	imageNames[2];
	vector< cv::Point3f > boardModel;
	vector< vector<cv::Point3f> >	objectPoints;
	vector< vector<cv::Point2f> >	points[2];
	vector< cv::Point2f>			corners[2];
	bool							found[2] = { false, false };
	cv::Size						imageSize;
	
	std::string point_cloud_filename = "C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\pointCloud.txt";


	// READ IN LIST OF CIRCLE GRIDS


	//Read file and see you can open

	
	if (!f){
		cout << "Cannot open file " << imageList << endl;
		//cin.get();
		
	}
	


	//This is to create the boardmodel (10cm) using two for loops
	for (i = 0; i < ny; i++)
	{
		for (j = 0; j < nx; j++)
		{
			boardModel.push_back(cv::Point3f((float)(i*squareSize), (float)(j*squareSize), 0.f));

			//cout << boardModel;

		}
	}

	i = 0;

	for (;;){

		//read each line of the file
		char buf[1024];

		//check to see if on odd or even image
		lr = i % 2;
		if (lr == 0) found[0] = found[1] = false;

		if (!fgets(buf, sizeof(buf) - 3, f)) break;

		size_t len = strlen(buf);
		while (len > 0 && isspace(buf[len - 1])) buf[--len] = '\0';
		if (buf[0] == '#') continue;

		cv::Mat img = cv::imread(buf, 0);
		if (img.empty()) break;


		//Convert to inverted black and white image
		cv::Mat im_gray = img;
		//cv::cvtColor(img, im_gray, CV_RGB2GRAY);
		//cv::cvtColor(img, im_gray, CV_RGB2GRAY);
		cv::Mat img_bw = im_gray > 25;
		cv::Mat outputImgBW;
		cv::bitwise_not(img_bw, outputImgBW);
		cv::imshow("Inverted Black and white image", outputImgBW);
		//cv::waitKey(0);


		imageSize = img.size();
		imageNames
			[lr].push_back(buf);

		/*
		if (lr == 1){
		cout << "@" << imageNames[0][i/2] << "\n";
		cout << "@" << imageNames[1][i/2] << "\n";

		}
		*/


		i++;

		// if we did not find board on the left image
		// it does not make sense to find it on the right
		if (lr == 1 && !found[0])
			continue;

		// Find circle grids and centers therein:

		//Check if the scaling factor is less than 1
		for (int s = 1; s <= maxScale; s++){

			cv::Mat timg = outputImgBW;
			if (s > 1) resize(img, timg, cv::Size(), s, s, cv::INTER_CUBIC);


			//cv::SimpleBlobDetector::Params params;
			//params.maxArea = 100000;
			//cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);
			//params.minArea = 1;
			//params.minDistBetweenBlobs = 0.5;
			//params.minCircularity = 1;
			//params.minThreshold = 0;



			//bool patternfound = cv::findCirclesGrid(outputImgBW, patternsize, centers, CALIB_CB_SYMMETRIC_GRID || CALIB_CB_CLUSTERING, blobDetector);

			//find the image circles on the grid
			found[lr] = cv::findCirclesGrid(timg, cv::Size(nx, ny), corners[lr], cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
			if (found[lr] || s == maxScale){
				cv::Mat mcorners(corners[lr]);
				mcorners *= (1. / s);

				//The pixels points of the corners
				//cout << "Corners: "<< corners[lr];
				//cout << "mCorners: " << mcorners;
			}

			//if you find the image circle then break out the for loop
			if (found[lr]) break;

		}

		//This part of the code displays the grid centres
		if (displayCorners){

			cout << buf << endl;
			cv::Mat cimg;
			cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);

			//draw the circle corners for the circle grid
			cv::drawChessboardCorners(cimg, cv::Size(nx, ny), corners[lr], found[lr]);

			cv::imshow("Conrners", cimg);
			cv::waitKey(5);
			//if (cv::waitKey(0 & 255) == 1){
			//	exit(-1);
			//}

		}
		else{
			cout << '.';
		}

		if (lr == 1 && found[0] && found[1]){

			//create the points for the stereoCalibration routine

			objectPoints.push_back(boardModel);
			points[0].push_back(corners[0]);
			points[1].push_back(corners[1]);

		}
	}
	fclose(f);

	// Calibrate the stereo cameras

	cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat D1, D2, R, T, E, F;
	cout << "\n Running stereo calibration ... \n";

	//cout << points[0][0] << "\n";
	//cout << points[0][1] << "\n";

	cv::stereoCalibrate(objectPoints, points[0], points[1], M1, D1, M2, D2, imageSize, R, T, E, F, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5),
		cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH);

	cout << "Done \n\n";
	cout << "Writing XML files \n";

	double focalLength = M1.at<double>(0, 0);
	double baseLine = T.at<double>(0, 1);

	double totalBaseLine = sqrt(T.at<double>(0, 0)*T.at<double>(0, 0) + T.at<double>(0, 1)*T.at<double>(0, 1) + T.at<double>(0, 2)*T.at<double>(0, 2));


	//writing the parameters to the test.xml file
	cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
	fs << "cameraMatrix" << M1;
	fs << "distCoeffs1" << D1;
	fs << "cameraMatrix2" << M2;
	fs << "distCoeffs2" << D2;
	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;




	cout << " focal lenght: \n " << M1.at<double>(0, 0);
	cout << " \n Baseline: \n " << T.at<double>(0, 1);
	cout << " \n Total Baseline: \n " << totalBaseLine;



	cout << "\nThe First Camera Matrix: \n" << M1;
	cout << "\n\nThe Second Camera Matrix: \n" << M2;
	cout << "\n\nThe First Camera Distortion Vector: \n" << D1;
	cout << "\n\nThe Second Camera Distortion Vector: \n" << D2;
	cout << "\n\nThe Rotation Matrix: \n" << R;
	cout << "\n\nThe Translation Vector: \n" << T;
	cout << "\n\nThe Essential Matrix: \n" << E;
	cout << "\n\nThe Fundamental Matrix \n" << F;

	fs.release();
	cout << "\n\nStereo Calibration Matrix Print Done! \n\n";


	//CALIBRATION QUALITY CHECK
	//..because the output fundamental matrix implicitly
	//includes all the output information,
	//we can check the quality of calibration using the
	//epipolar geometry constraint: m2^t*F*m1=0

	vector< cv::Point3f > lines[2];
	double avgErr = 0;
	int nframes = (int)objectPoints.size();

	for (i = 0; i < nframes; i++){

		vector < cv::Point2f >& pt0 = points[0][1];
		vector < cv::Point2f >& pt1 = points[1][i];

		cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);
		cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);

		cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
		cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);

		//cout << "epilines1: " << lines[0] << "\n";
		//cout << "epilines2: " << lines[1] << "\n";

		for (j = 0; j < N; j++){

			//fabs returns the absolute value
			double err = fabs(pt0[j].x*lines[1][j].x + pt0[j].y*lines[1][j].y + lines[1][j].z) + fabs(pt1[j].x*lines[0][j].x + pt1[j].y*lines[0][j].y + lines[0][j].z);
			avgErr += err;

		}
	}

	//cout << "avg error = " << avgErr / (nframes*N) << endl;

	//Compute and Display Rectification
	
	cv::Mat img1r, img2r, disp, vdisp;

	if (showUndistorted){

		cv::Mat R1, R2, P1, P2, Q,  map11, map12, map21, map22;



		//If By Calibrated (Bouguet's Method)

		if (!useUncalibrated){
			stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, 0);
			//isVerticalStereo = false;
			isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

			//precompute maps for cvRemap()

			initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
			initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);

		}

		// Using Hartley's Method instead
		else{

			//use intrinsic parameters of each camera, but compute the recitfication transformation directly from the fundamental matrix

			//!!This code needs testing :) !

			vector < cv::Point2f > allpoints[2];
			for (i = 0; i < nframes; i++){
				copy(points[0][i].begin(), points[0][i].end(), back_inserter(allpoints[0]));
				copy(points[1][i].begin(), points[1][i].end(), back_inserter(allpoints[1]));
			}

			cv::Mat F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);
			cv::Mat H1, H2;
			cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, imageSize, H1, H2, 3);

			R1 = M1.inv()*H1*M1;
			R2 = M2.inv()*H2*M2;

			// Precomupte map for cvRemap()

			cv::initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
			cv::initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);


		}


		//Rectify the images and find disparity maps



		



		//Setup for finding stereo corrrespondences

		//cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();


		cv::StereoSGBM sgbm;

		sgbm.SADWindowSize = 41;
		sgbm.preFilterCap = 31;
		sgbm.minDisparity = 0;
		sgbm.numberOfDisparities = 16;
		sgbm.speckleRange = 32;
		sgbm.speckleWindowSize = 100;
		sgbm.disp12MaxDiff = 1;
		sgbm.fullDP = false;
		sgbm.P1 = 8 * 1 * 41 * 41;
		sgbm.P2 = 32 * 1 * 41 * 41;
		sgbm.uniquenessRatio = 15;

		//Showing one depth map for one board

		cv::Mat imgDepth1 = leftImage;
		cv::Mat imgDepth2 = rightImage;
		cv::Mat dispDepth, vdispDepth;

		//cv::imshow("disparity", imgDepth2);
		//cv::waitKey(0);

		cout << "\nLoading\n";
		//sgbm(imgDepth1, imgDepth2, dispDepth);

		//cout << "\nDone\n";


		normalize(dispDepth, vdispDepth, 0, 256, cv::NORM_MINMAX, CV_8U);
		//cv::imshow("Depth Map for one board", vdispDepth);


		//cv::waitKey(0);





	//	for (i = 0; i < nframes; i++){

		i = 0;
			cv::Mat img1 = imgDepth1;
			cv::Mat img2 = imgDepth2;

			//cv::Mat img1r, img2r, disp, vdisp;

			/*
			if (img1.empty() || img2.empty()){
				continue;
			}
			*/




			cv::remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
			cv::remap(img2, img2r, map21, map22, cv::INTER_LINEAR);

			cv::imshow("Remaped", img1r);
			cv::imshow("Remaped2", img2r);


			
			if (!isVerticalStereo || !useUncalibrated){
				//When the Stereo camera is oriented Vertically, Hartley method does not transpose the image, so the epipolar lines in
				//the recitified images are vertical. Stereo correspondence function does not support such a case.
				
				//sgbm(img1r, img2r, disp);
				//normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
				//cv::imshow("disparity working", vdisp);
				//cv::waitKey(0);

				//int ndisparities = 16 * 5;   /**< Range of disparity */
				//int SADWindowSize = 21;

				

				cv::StereoBM sbm;

				sbm.state->SADWindowSize = 9;
				sbm.state->numberOfDisparities = 112;
				sbm.state->preFilterSize = 5;
				sbm.state->preFilterCap = 61;
				sbm.state->minDisparity = -39;
				sbm.state->textureThreshold = 507;
				sbm.state->uniquenessRatio = 0;
				sbm.state->speckleWindowSize = 0;
				sbm.state->speckleRange = 8;
				sbm.state->disp12MaxDiff = 1;

				cv::Mat g1, g2;

				cvtColor(img1r, g1, CV_BGR2GRAY);
				cvtColor(img2r, g2, CV_BGR2GRAY);
				
				sbm(g1, g2, disp);

				
				
				//cv::waitKey(0);



				//cout << disp << " Disp\n";
				
				normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
				//cv::imshow("disparity working", vdisp);
				//cv::waitKey(0);

				cv::Mat xyz;

				reprojectImageTo3D(disp, xyz, Q, true);
				cout << "\n3d Working \n";
				//cin.get();

				saveXYZ(point_cloud_filename.c_str(), xyz);
				cout << "\nPoint Cloud Working \n";
				//cin.get();


			}


			

			//Rectify the images and find disparity maps

			cv::Mat pair;
			if (!isVerticalStereo){
				pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);
			}
			else{
				pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);
			}


			if (!isVerticalStereo){
				cv::Mat part = pair.colRange(0, imageSize.width);
				cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
				part = pair.colRange(imageSize.width, imageSize.width * 2);
				cvtColor(img2r, part, cv::COLOR_GRAY2BGR);

				for (j = 0; j < imageSize.height; j += 16){
					cv::line(pair, cv::Point(0, j), cv::Point(imageSize.width * 2, j), cv::Scalar(0, 255, 0));
				}

			}

			
			
			else {
				
				cv::Mat img1rb, img2rb;


				cvtColor(img1r, img1rb, CV_RGB2GRAY);
				cvtColor(img2r, img2rb, CV_RGB2GRAY);
				cv::Mat part = pair.rowRange(0, imageSize.height);
				
				cv::cvtColor(img1rb, part, cv::COLOR_GRAY2BGR);
				part = pair.rowRange(imageSize.height, imageSize.height * 2);
				cv::cvtColor(img2rb, part, cv::COLOR_GRAY2BGR);

				for (j = 0; j < imageSize.width; j += 32){
					cv::line(pair, cv::Point(j, 0), cv::Point(j, imageSize.height * 2), cv::Scalar(0, 255, 0));
				}

			}


			
			cv::Mat pairResized;
			cv::resize(pair, pairResized, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
			cv::imshow("rectified", pairResized);
			cv::waitKey(0);




			
		
		}

		
	//}

		return imageVar{ img1r, img2r, focalLength, totalBaseLine };

}







float** laserLineCombined(float** laserLineArrayLeft, float** laserLineArrayRight, int imagecols){

	float** tempArr = new float*[imagecols];
	for (int i = 0; i < imagecols; i++){
		tempArr[i] = new float[4];
	}

	for (int i = 0; i < imagecols; i++){

		tempArr[i][0] = float (i);
		tempArr[i][1] = laserLineArrayLeft[i][1];
		tempArr[i][2] = laserLineArrayLeft[i][2];
		tempArr[i][3] = laserLineArrayRight[i][1];
		tempArr[i][4] = laserLineArrayRight[i][2];

		if (tempArr[i][2] != 0 && tempArr[i][4] != 0){
			tempArr[i][5] = abs(tempArr[i][2] - tempArr[i][4]);
		}
		else {
			tempArr[i][5] = 0;
		}


	}

	return tempArr;
}




float** laserLineDetector(cv::Mat imgInput){

	//Opening the laser line image 
	cv::Mat img = imgInput;
	//cv::namedWindow("Example1", cv::WINDOW_AUTOSIZE);
	cv::imshow("Example1", img);
	//cv::waitKey(0);

	
	
	
		//Initializing the three layers and spliting the RGB colors into bgr
		cv::Mat bgr[3];
		cv::Mat bgrThreshold[3];

		split(img, bgr);


		//Using Threshold of 60 brightness for the green pixel
		cv::threshold(bgr[0], bgrThreshold[0], 30, 255, cv::THRESH_BINARY);
		cv::threshold(bgr[1], bgrThreshold[1], 60, 255, cv::THRESH_BINARY);
		cv::threshold(bgr[2], bgrThreshold[2], 30, 255, cv::THRESH_BINARY);

		//cv::imshow("Threshold blue", bgrThreshold[0]);
		//cv::imshow("Threshold green", bgrThreshold[1]);
		//cv::imshow("Threshold red", bgrThreshold[2]);

		//Creating images for binary blue, green and red
		cv::Mat b = bgrThreshold[0];
		cv::Mat g = bgrThreshold[1];
		cv::Mat r = bgrThreshold[2];
		cv::Mat bandg;
		cv::Mat bandgandr;

		//Using logical AND to sum the pixels that are the laser line
		cv::bitwise_and(b, g, bandg);
		cv::bitwise_and(bandg, r, bandgandr);

		cv::imshow("bandgandr", bandgandr);

		//char* testing = "Hello world";



		// Number of row and columns of the image 

		const int rows = img.rows;
		const int columns = img.cols;

		float** arr = new float*[columns];
		for (int i = 0; i < columns; i++){
		arr[i] = new float[4];
		}
		arr[1][2] = (float)(0);

		//cout << "working" << arr[1][2];

		// Using Vec3b to test the pixel point
		cv::Vec3b test = bandgandr.at<cv::Vec3b>(400, 400);
		cv::Vec3b blank = { 0, 0, 0 };
		cv::Vec3b full = { 255, 255, 255 };

		if (bandgandr.at<cv::Vec3b>(400, 400) == blank){
			cout << "working";
		}


		//Will print the number of rows and  columns of the image
		cout << test << "rows: " << rows << "columns: " << columns << "\n";

		//Shows the green channel of the image in grayscale
		//cv::imshow("green", bgr[1]);

		//Getting one green pixel value and finding a weighting value
		cv::Vec3b greenPixel = bgr[1].at<cv::Vec3b>(193, 3);
		int greenPixelValue = greenPixel.val[1];
		float weighting = (float)(greenPixelValue / 255);

		//Intitalizing the calculation points 
		float weightedMean = 0;
		float totalOfWeightedMean = 0;
		float totalOfWeighting = 0;
		float laserLineSub = 0;


		//float weightedMean = pixelrow * weighting;
		//float totalOfWeightedMean = totalOfWeightedMean + weightedMean;
		//float totalOfWeighting = totalOfWeighting + weighting;
		//float laserLineSub = totalOfWeightedMean / totalOfWeighting;
		//cout << "greenPixel: " << greenPixelValue << "\n";

		double avg = 0;
		int counter = 0;
		float diffrence = 0;
		float maxDiffrence = 0;

		

		/*
		ofstream myfile;
		myfile.open("C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\data.csv");
		ofstream errorResult;
		errorResult.open("C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\errorResult.csv");
		ofstream arrayData;
		arrayData.open("C:\\Users\\phucte\\Documents\\Visual Studio 2013\\Projects\\StereoCal5\\Release\\array.csv");
		*/

	
		//cv::Mat A = cv::Mat::zeros(img.size(), CV_16S);

		cv::Mat A(img.size(), img.type(), cv::Scalar(0, 0, 0));

		//A.setTo(cv::Scalar(0, 0, 0));



		for (int i = 0; i < img.cols; i++)
		//for (int i = 512; i < 513; i++)
		{
			for (int j = 0; j < img.rows; j++)
			{

				//A.at<cv::Vec3b>(img.rows - j, img.cols - i) = img.at<cv::Vec3b>(j, i);

				cv::Vec3b greenPixel = img.at<cv::Vec3b>(j, i);
				greenPixelValue = greenPixel.val[1];


				//cout << "J: " << j << "\n";
	
				if (greenPixelValue > 30){

					//cout << greenPixel << "J det : " << j << "\n";
					A.at<cv::Vec3b>(j, i) = { 255, 255, 255 };

					weighting = (float)greenPixelValue / (float)255;
					weightedMean = j * weighting;
					totalOfWeightedMean = totalOfWeightedMean + weightedMean;
					totalOfWeighting = totalOfWeighting + weighting;

					//cout << "Green Pixel: " << greenPixelValue << "\n";
					//cout << "Weighting!!! : " << weighting << "\n";
					//cout << "WeightMean : " << weightedMean << "\n";

					//cout << "j: " << j << "\n";
					//cout << "Laser Line @ :" << " Columns: " << i << " Row: " << j << " greenPixelIntensity: " << greenPixelValue << "\n";
					//cout << "Laser Line @ :" << " Columns: " << i << " Row: " << j << " Pixel Number: " <<  bandgandr.at<cv::Vec3b>(j, i) << "\n";
					counter++;
					avg = avg + j;
				}

			}

			
			

			arr[i][0] = (float)i;

			if (counter > 0){
				float laserLineSub = totalOfWeightedMean / totalOfWeighting;
				//cout << "Laser Line @ :" << i << "Laser Line Sub Pixel = " << laserLineSub << "\n";;
				float laserPixel = float(avg / counter);
				float laserlineDifference = laserPixel - laserLineSub;

				//Print out results 
				/*
				cout << "Laser Line @ :" << i << "    Avg: " << avg << " Counter: " << counter << " LaserPixel: " << laserPixel;
				cout << " SubPixel: " << laserLineSub << " Difference: " << laserlineDifference << "\n";
				*/


				//myfile << i << ", " << laserPixel << ", " << laserLineSub << "\n";
				//errorResult << i << ", " << laserlineDifference << endl;;

				arr[i][1] = laserPixel;
				arr[i][2] = laserLineSub;

				//reset counter and laserPixel

				if (abs(laserlineDifference) > maxDiffrence){
					maxDiffrence = abs(laserlineDifference);
					//cout << " Max Difference: " << maxDiffrence << "\n";
				}

				float weightMean = 0;


				weightMean = 0;
				totalOfWeightedMean = 0;
				totalOfWeighting = 0;
				laserLineSub = 0;

				avg = 0;
				counter = 0;


			}
			else {

				arr[i][1] = 0;
				arr[i][2] = 0;

				//myfile << i << ", " << "0" << ", " << laserLineSub << "\n";
				//errorResult << i << ", " << "0" << endl;;

			}

		}

		

		//cv::imshow("Test filp ", A);
		cout << "Max Difference: " << maxDiffrence << "\n";
		cout << "bandgandr.cols:  " << bandgandr.cols << "\n";
		cout << "bandgandr.rows:  " << bandgandr.rows << "\n";

		
	
		for (int i = 0; i < columns; i++){

			//cout << "Laser Line at " << i << " Laser pixel: " << arr[i][1] << " laser Line Sub: " << arr[i][2] << "\n";


			//File output for the laser array (inactive)

			/*
			if (arr[i][1] != 0 || arr[i][2] != 0){
				//arrayData << i << ", " << arr[i][1] << ", " << arr[i][2] << "\n";
			}
			*/
		}

		cv::imshow("LaserLine Deteched", A);
		cv::waitKey(0);

		

		//myfile.close();
		//errorResult.close();
		///arrayData.close();



		cv::imshow("Here is A", A);
		//cv::waitKey(0);

		return arr;

		

		
}

double max3(double x, double y, double z){
	double max = x;

	if (y > max){
		max = y;
	}
	
	if (z > max){
		max = z;
	}

	return max;
}

vector<Vec3<double>> XYZSpace(const char* filename, float** laserLineArray, double focalLength, double baseLine, int imagecols){


	double x, y, z;

	vector<Vec3<double>> testing;

	

	//testing.push_back({ 1, 2, 3 });
	//testing.push_back({ 3, 5, 7 });
	//testing.push_back({ 6, 9, 12 });

	//testing.push_back({ 1, 0, 2 });
	//testing.push_back({ -1, 1, 2 });
	//testing.push_back({ 5, 0, 3 });
	

	FILE* fp = fopen(filename, "wt");

	bool subpixel = false;
	double laserLineTop, laserLineBottom;

	for (int i = 0; i < imagecols; i++){

		if (subpixel){
			laserLineTop = laserLineArray[i][2];
			laserLineBottom = laserLineArray[i][4];

		}
		else{
			laserLineTop = laserLineArray[i][1];
			laserLineBottom = laserLineArray[i][3];

		}


		if (laserLineTop != 0 && laserLineBottom && (abs(laserLineTop - laserLineBottom)) != 0){

			z = (focalLength * baseLine) / abs(laserLineTop - laserLineBottom);

			x = (laserLineTop * z) / focalLength;

			y = (laserLineArray[i][0] * z) / focalLength;

			if (i % 200 == 0){
				cout << "\n Z: " << z;
				cout << "\n X: " << x;
				cout << "\n Y: " << y;
				cout << "\n";

				testing.push_back({ x, y, z });

			}

			

			fprintf(fp, "%f;%f;%f\n", x, y, z);
		}

		

	}


	cout << "\nLast\n";
	cout << "\n Z: " << z;
	cout << "\n X: " << x;
	cout << "\n Y: " << y;

	cin.get();

	fclose(fp);


	
	/*
	z = (focalLength * baseLine) / abs(laserLineArray[123][2] - laserLineArray[123][4]);

	x = (laserLineArray[123][2] * z) / focalLength;

	y = (laserLineArray[123][0] * z) / focalLength;


	cout << "\n Z: " << z;
	cout << "\n X: " << x;
	cout << "\n Y: " << y;

	*/

	//for (int i = 0; i < imagecols; i++){
	//}
		
		

	return testing;

}


cv::Vec4d planeFromPoints(vector<Vec3<double>> testing){



	Vec3<double> sum = { 0, 0, 0 };

	for (int i = 0; i < (double)(testing.size()); i++){
		sum = sum + testing.at(i);
	}


	Vec3<double> centroid = sum / (double)(testing.size());

	//cout << "\n This is centroid:  \n";
	//centroid.display();

	double xx = 0.0;
	double xy = 0.0;
	double xz = 0.0;
	double yy = 0.0;
	double yz = 0.0;
	double zz = 0.0;

	Vec3<double> r = testing.at(0) - centroid;

	//cout << "\n This is R:  \n";
	//r.display();

	for (int i = 0; i < (double)(testing.size()); i++){
		r = testing.at(i) - centroid;

		//cout << "\n Testing : Y \n" << r.getY();

		//r.display();
		xx += r.getX() * r.getX();
		xy += r.getX() * r.getY();
		xz += r.getX() * r.getZ();
		yy += r.getY() * r.getY();
		yz += r.getY() * r.getZ();
		zz += r.getZ() * r.getZ();

	}

	double det_x = (yy*zz) - (yz*yz);
	double det_y = (xx*zz) - (xz*xz);
	double det_z = (xx*yy) - (xy*xy);




	//cout << "\n Testing det x : \n " << det_x << "\n Testing det y : \n " << det_y << "\n Testing det z : \n " << det_z;


	double det_max = max3(det_x, det_y, det_z);

	Vec3<double> dir = { 0, 0, 0 };

	if (det_max == det_x){
		double a = ((xz*yz) - (xy*zz)) / det_x;
		double b = ((xy*yz) - (xz*yy)) / det_x;
		dir = { 1.0, a, b };

	}
	else if (det_max == det_y){
		double a = ((yz*xz) - (xy*zz)) / det_y;
		double b = ((xy*xz) - (yz*xx)) / det_y;
		dir = { a, 1.0, b };
	}
	else {
		double a = ((yz*xy) - (xz*yy)) / det_z;
		double b = ((xz*xy) - (yz*xx)) / det_z;
		dir = { a, b, 1.0 };
	}
	//cout << "\n This is dir:  \n";
	//dir.display();

	Vec3<double> normalisedVector;

	double distance = sqrt((dir.getX()*dir.getX())
	+ (dir.getY()*dir.getY()) + (dir.getZ()*dir.getZ()));

	//cout << "\n This is distance : \n " << distance;

	normalisedVector.addX(dir.getX() / distance);
	normalisedVector.addY(dir.getY() / distance);
	normalisedVector.addZ(dir.getZ() / distance);

	cout << "\n This is normalisedVector:  \n";
	normalisedVector.display();

	double d = -((normalisedVector.getX()*centroid.getX()) 
	+ (normalisedVector.getY()*centroid.getY()) + 
	(normalisedVector.getZ()*centroid.getZ()));

	cout << "\n This is d : \n " << d;
	double a = normalisedVector.getX();
	double b = normalisedVector.getY();
	double c = normalisedVector.getZ();
	cout << "\n This is a:  \n" << a;
	cout << "\n This is b:  \n" << b;
	cout << "\n This is c:  \n" << c;

	cv::Vec4d plane(a, b, c, d);

	//cout << "\n plane: \n " << plane;
	cout << "\nPlane : ";
	cout << a << "x + " << b << "y + " << c << "z + " << d << " = 0";
	//cout << "\n Max det: \n " << det_max;


	//cout << "\n This is xx:  \n" << xx;
	//cout << "\n This is xy:  \n" << xy;
	//cout << "\n This is xz:  \n" << xz;
	//cout << "\n This is yy:  \n" << yy;
	//cout << "\n This is yz:  \n" << yz;
	//cout << "\n This is zz:  \n" << zz;
	//centroid /= (double) (testing.size());

	//cv::Point3f result = cv::dot(testing.at(1), testing.at(2));

	//cout << "\n working: vector addition : \n" << testing.at(0).display + testing.at(1).display << "\n";
	cout << "\n working: vector: \n" << testing.at(0).getX() << "\n";
	//cout << "\n working: vector: \n" << testing.at(1).display << "\n";
	//cout << "\n working: vector: x  \n" << testing.at(1).getX << "\n";
	//cout << "\n working: vector: \n" << testing.at(2).display << "\n";
	cout << "\n working: size: \n" << testing.size() << "\n";
	//sum.display();
	//centroid.display();
	cin.get();

	return plane;


}






void saveXYZ(const char* filename, const cv::Mat& mat){
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");

	cout << "\n Image Rows: \n" << mat.rows;
	cout << "\n Image Cols \n" << mat.cols;
	//cin.get();

	cv::Vec3f point = mat.at<cv::Vec3f>(400, 400);

	cout << "\n Image point @ 400 \n" << point;
	//cin.get();

	for (int y = 0; y < mat.rows; y++){
		for (int x = 0; x < mat.cols; x++){
			
			cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);

		}
	}
	fclose(fp);
}


/*
using namespace cv;



// Try and get diving circles board recognised - Done :)
// implement the files selecting 
// Try and get adaptive thresholding working - Otsu
//  Stereo

int test(int argc, char** argv){
	cv::Mat img = cv::imread(argv[1], -1);
	cv::Mat out;
	cv::Mat threshold;
	if (img.empty()) return -1;
	//cv::namedWindow("Example1", cv::WINDOW_AUTOSIZE);
	cv::imshow("Example", img);
	std::cout << "Testing";
	//cv::GaussianBlur(img, out, cv::Size(5, 5), 3, 3);
	//cv::GaussianBlur(out, out, cv::Size(5, 5), 3, 3);
	//cv::imshow("Blured", out);
	
	Mat im_gray;
	cvtColor(img, im_gray, CV_RGB2GRAY);
	Mat img_bw = im_gray > 25;



	//cv::threshold(img, threshold, 45, 255, CV_THRESH_BINARY);

	//cv::adaptiveThreshold(img, threshold, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);
	//C++: void adaptiveThreshold(InputArray src, OutputArray dst, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double C)
	//adaptiveThreshold(img, threshold, 255, 1, 1, 5, 4);
	//cv::adaptiveThreshold(out, threshold, 255, 1, 1, 3, 5);
	//adaptiveThreshold(img, threshold, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 13, 1);


	cv::imshow("Threshold", img_bw);

	//	cv::Mat outputImg = img;
	//	Size patternsize(6, 7); //number of centers
	//	Mat gray = img; //source image
	//	vector<Point2f> centers; //this will be filled by the detected centers

	//	bool patternfound = findCirclesGrid(gray, patternsize, centers);

	//	drawChessboardCorners(outputImg, patternsize, Mat(centers), patternfound);


	SimpleBlobDetector::Params params;
	params.maxArea = 100000;
	Ptr<FeatureDetector> blobDetector = new SimpleBlobDetector(params);
	params.minArea = 1;
	params.minDistBetweenBlobs = 0.5;
	params.minCircularity = 1;
	params.minThreshold = 0;


	


	cv::Size patternsize(7, 7);
	cv::Mat checkImg = img;
	cv::Mat outputImg = img;
	cv::Mat outputImgBW = img;
	std::vector<cv::Point2f> centers;

	bitwise_not(img_bw, outputImgBW);
	//cv::subtract(cv::Scalar(255, 255, 255), img, outputImg);

	bool patternfound = cv::findCirclesGrid(outputImgBW, patternsize, centers, CALIB_CB_SYMMETRIC_GRID || CALIB_CB_CLUSTERING, blobDetector);
	cv::drawChessboardCorners(outputImg, patternsize, cv::Mat(centers), patternfound);

	if (patternfound){
		std::cout << "Yay! working!";
	}
	else{
		std::cout << "Board was not found";
	}

	
	cv::imshow("Invert", outputImgBW);
	cv::imshow("CirclesBoard", outputImg);


	cv::waitKey(0);
	cv::destroyWindow("Example1");


	return 0;

}

*/
