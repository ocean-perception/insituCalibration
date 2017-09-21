

/* Data visualisation of mapping displacement error
	By Phuc PHung Te 21/09/2017
	*/

#include "stdafx.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <math.h> 
#pragma warning(disable:4996)

using namespace std;

struct colour{
	int rr;
	int gg;
	int bb;
};

struct coordinate {

	double x;
	double y;
	double z;

};

struct intensity {

	double x;
	double y;
	double z;

};

struct stats{
	double xMean;
	double xStd;
	double yMean;
	double yStd;
	double zMean;
	double zStd;
};

const int maxReading = 100000;
coordinate points[maxReading];
coordinate points1[maxReading];
coordinate points2[maxReading];
coordinate pointsDifference[maxReading];
intensity pointsIntensity[maxReading];
colour pixelColour[maxReading];


colour hsvrgb(double input){

	int vv = 1;
	int ss = 1;
	

	if (input>= 360){
		input = 360;
	}
	if (input <= 0){
		input = 0;
	}

	double hh = input / 60;

	//cout << setprecision(10) << "\n input:  " << input;

	int ii = floor(hh);

	//cout << setprecision(10) << "\n hh:  " << hh;
	//cout << setprecision(10) << "\n ii:  " << ii;

	/*
	if (ii < 0){
		ii = 0;
	}

	if (ii < 360){
		ii = 360;
	}
	*/

	double ff = hh - ii;
	double pp = vv * (1 - ss);
	double qq = vv * (1 - ss * ff);
	double tt = vv * (1 - ss * (1 - ff));

	//cout << setprecision(10) << "\n ff:  " << ff;
	//cout << setprecision(10) << "\n pp:  " << pp;
	//cout << setprecision(10) << "\n qq:  " << qq;
	//cout << setprecision(10) << "\n tt:  " << tt;

	double rr, gg, bb;


	if (ii == 0) {
		rr = vv;
		gg = tt;
		bb = pp;
	}
	if (ii == 1){
		rr = qq;
		gg = vv;
		bb = pp;
	}
	if (ii == 2){
		rr = pp;
		gg = vv;
		bb = tt;
	}
	if (ii == 3){
		rr = pp;
		gg = qq;
		bb = vv;
	}
	if (ii == 4){
		rr = tt;
		gg = pp;
		bb = vv;
	}
	if (ii >= 5){
		rr = vv;
		gg = pp;
		bb = qq;
	}


	/*
	
	*/
	int  Crr, Cgg, Cbb;

	Crr = (int) floor(rr * 255.0);
	Cgg = (int) floor(gg * 255.0);
	Cbb = (int) floor(bb * 255.0);

	//cout << setprecision(10) << "\n rr:  " << rr * 255.0;
	//cout << setprecision(10) << "\n Crr:  " << Crr;
	//cout << setprecision(10) << "\n Cgg:  " << Cgg;
	//cout << setprecision(10) << "\n Cbb;  " << Cbb;

	colour rgb;

	rgb.rr = Crr;
	rgb.gg = Cgg;
	rgb.bb = Cbb;

	return rgb;

}




void readingPoints(string filename){

	

	string line, line2;

	double x, y, z;

	vector<double> xList;
	vector<double> yList;
	vector<double> zList;


	//D:\\material_for_tutorial\\processed\\nt15-e03_tutorial\\3d_reconstructions\\3dRec_160000to161100.ply
	//C:\\Users\\phucte\\Desktop\\string.txt


	ifstream file(filename);

	bool pastHeaderFlag = false;

	int i = 0;

	while (file.good() && i<maxReading){
		getline(file, line);
		if (pastHeaderFlag){


			//cout << "\n Working: " << line;
			stringstream work(line);
			string token;
			int j = 0;



			while (getline(work, token, ' ')){

				//cout << "\n Working token : " << token;

				if (j == 0){
					//cout << "\n Working X : " << token;
					x = stod(token);
				}
				if (j == 1){
					//cout << "\n Working Y : " << token;
					y = stod(token);
				}
				if (j == 2){
					//cout << "\n Working Z : " << token;
					z = stod(token);
				}
				j++;
				if (j > 4){
					j = 0;
				}

			}

			//cout << "\n x: " << x << " y: " << y << " z: " << z;
			points[i].x = x;
			points[i].y = y;
			points[i].z = z;
			//cout << "\n x: " << points[i].x << " y: " << points[i].y << " z: " << points[i].z;

			i++;

			//cout << "\n i = " << i;

		}
		if (line == "end_header"){
			pastHeaderFlag = true;
		}




	}//end of while



	
}


stats findMeanAndStd(){

	stats result;

	double xSum = 0.0, xMean, xStandardDeviation = 0.0;
	double ySum = 0.0, yMean, yStandardDeviation = 0.0;
	double zSum = 0.0, zMean, zStandardDeviation = 0.0;
	double xAbsSum = 0.0, xAbsMean;
	double yAbsSum = 0.0, yAbsMean;
	double zAbsSum = 0.0, zAbsMean;


	/*
	for (int a = 0; a < maxReading; a++){
		
		pointsDifference[a].x = a;
		pointsDifference[a].y = a;
		pointsDifference[a].z = a;
	}
	*/

	/*
	for (int a = 0; a < maxReading; a++){

		cout << setprecision(10) << "\n pixel no.:" << a << " Diff. x: " << pointsDifference[a].x << " y: " << pointsDifference[a].y << " z: " << pointsDifference[a].z << " Pixel intensity: " << pointsIntensity[a].y;

	}
	*/

	for (int a = 1; a < maxReading; a++){
		
		xSum += pointsDifference[a].x;
		ySum += pointsDifference[a].y;
		zSum += pointsDifference[a].z;
	}

	for (int a = 1; a < maxReading; a++){

		xAbsSum += abs(pointsDifference[a].x);
		yAbsSum += abs(pointsDifference[a].y);
		zAbsSum += abs(pointsDifference[a].z);
	}

	xAbsMean = xAbsSum / maxReading;
	yAbsMean = yAbsSum / maxReading;
	zAbsMean = zAbsSum / maxReading;





	//cout << "\n xSum " << xSum;
	//cout << "\n ySum " << ySum;
	//cout << "\n zSum " << zSum;

	xMean = xSum / maxReading;
	yMean = ySum / maxReading;
	zMean = zSum / maxReading;

	
	for (int a = 0; a < maxReading; a++){

		xStandardDeviation += pow(pointsDifference[a].x - xMean, 2);
		yStandardDeviation += pow(pointsDifference[a].y - yMean, 2);
		zStandardDeviation += pow(pointsDifference[a].z - zMean, 2);
	}

	//cout << "\n xStandardDeviation " << xStandardDeviation;
	//cout << "\n yStandardDeviation " << yStandardDeviation;
	//cout << "\n zStandardDeviation " << zStandardDeviation;

	result.xMean = xAbsMean;
	result.xStd = sqrt(xStandardDeviation/maxReading);
	result.yMean = yAbsMean;
	result.yStd = sqrt(yStandardDeviation/maxReading);
	result.zMean = zAbsMean;
	result.zStd = sqrt(zStandardDeviation/maxReading);

	return result;
}





int _tmain(int argc, _TCHAR* argv[])
{
	
	

	string filename1 = "D:\\material_for_tutorial\\processed\\nt15-e03_tutorial\\3d_reconstructions\\3dRec_160000to161100.ply";
	string filename2 = "D:\\material_for_tutorial\\processed\\nt15-e03_tutorial\\3d_reconstructions\\3dRec_160000to161100_1cmoffset.ply";
	string filenameOut = "C:\\Users\\phucte\\Desktop\\ply\\points1offset1cmScale00209X.ply";
	
	bool scale = true;
	//bool scale = false;

	//string filename2 = "C:\\Users\\phucte\\Desktop\\string.txt";

	//coordinate points[maxReading];
	//coordinate points1[maxReading];
	//coordinate points2[maxReading];

	readingPoints(filename1);

	for (int a = 0; a < maxReading; a++){
			// << "Working ";
			points1[a] = points[a];
			//cout << "Testing :" << points1[a].x;
		}

	readingPoints(filename2);

	for (int a = 0; a < maxReading; a++){
		// << "Working ";
		points2[a] = points[a];
		//cout << "Testing :" << points1[a].x;
	}

	for (int a = 0; a < maxReading; a++){

		pointsDifference[a].x = points1[a].x - points2[a].x;
		pointsDifference[a].y = points1[a].y - points2[a].y;
		pointsDifference[a].z = points1[a].z - points2[a].z;

	}

	

	/*

	double xMax ;
	double xMin ;

	xMax = pointsDifference[0].x;
	xMin = pointsDifference[0].x;

	for (int a = 0; a < maxReading; a++){

		


		if (pointsDifference[a].x > xMax){
			xMax = pointsDifference[a].x;
		}

		if (pointsDifference[a].x < xMin){
			xMin = pointsDifference[a].x;
		}

	}

	cout << "\n X Min : " << xMin;
	cout << "\n X Max : " << xMax;
	*/

	double zMax, xMax, yMax;
	double zMin, xMin, yMin;

	zMax = pointsDifference[0].z;
	zMin = pointsDifference[0].z;
	xMax = pointsDifference[0].x;
	xMin = pointsDifference[0].x;
	yMax = pointsDifference[0].y;
	yMin = pointsDifference[0].y;

	for (int a = 0; a < maxReading; a++){

		if (pointsDifference[a].z > zMax){
			zMax = pointsDifference[a].z;
		}

		if (pointsDifference[a].z < zMin){
			zMin = pointsDifference[a].z;
		}

		if (pointsDifference[a].x > xMax){
			xMax = pointsDifference[a].x;
		}

		if (pointsDifference[a].x < xMin){
			xMin = pointsDifference[a].x;
		}
		if (pointsDifference[a].y > yMax){
			yMax = pointsDifference[a].y;
		}

		if (pointsDifference[a].y < yMin){
			yMin = pointsDifference[a].y;
		}

	}


	cout << setprecision(10) << "\n xMax: " << xMax;
	cout << setprecision(10) << "\n xMin: " << xMin;

	cout << setprecision(10) << "\n yMax: " << yMax;
	cout << setprecision(10) << "\n yMin: " << yMin;

	cout << setprecision(10) << "\n zMax: " << zMax;
	cout << setprecision(10) << "\n zMin: " << zMin;



	double minScale, maxScale;

	minScale = -0.5;
	maxScale = 0.5;

	if (scale){
		xMin = minScale;
		xMax = maxScale;
		yMin = minScale;
		yMax = maxScale;
		zMin = minScale;
		zMax = maxScale;
		cout << "\n Scaling \n";
	}
	double intensityValue;

	double tempX, tempY, tempZ;



	for (int a = 0; a < maxReading; a++){

		tempX = (240 * (pointsDifference[a].x - xMin)) / abs((xMax)-(xMin));
		tempY = (240 * (pointsDifference[a].y - yMin)) / abs((yMax)-(yMin));
		tempZ = (240 * (pointsDifference[a].z - zMin)) / abs((zMax)-(zMin));


		if (tempX > 359){
			tempX = 359;
		}
		if (tempX < 0){
			tempX = 0;
		}
		if (tempY > 359){
			tempY = 359;
		}
		if (tempY < 0){
			tempY = 0;
		}
		if (tempZ > 359){
			tempZ = 359;
		}
		if (tempZ < 0){
			tempZ = 0;
		}
	
		pointsIntensity[a].x = tempX;
		pointsIntensity[a].y = tempY;
		pointsIntensity[a].z = tempZ;

	}

	/*
	for (int a = 0; a < maxReading; a++){

		cout << setprecision(10) << "\n Intensity x: " << pointsIntensity[a].x << " y : " << pointsIntensity[a].y << " TEST z: " << pointsIntensity[a].z;

	}
	*/


	//cout << setprecision(10) << "\n x intensity: " << (240 * (abs(pointsDifference[12].x - xMin))) / abs(abs(xMax) - abs(xMin));
	//cout << setprecision(10) << "\n x intensity div abs : " << abs(abs(xMax) - abs(xMin));
	//cout << setprecision(10) << "\n x intensity diff. value : " << abs(pointsDifference[12].x - xMin);
	
	
	for (int a = 0; a < maxReading; a++){

		intensityValue = pointsIntensity[a].x;
		
		/*
		if (intensityValue > 359){
			intensityValue = 359;
		}
		if (intensityValue <= 0){
			intensityValue = 0;
		}
		*/
		
		//cout << setprecision(10) << "\n intensityValue: " << intensityValue;
		//cout << setprecision(10) << "\n difference : " << abs(pointsDifference[2].z - zMin);
		//cout << setprecision(10) << "\n div : " << abs(pointsDifference[2].z - zMin);

		double input = (240 - intensityValue);

		colour rgb = hsvrgb(input);

		pixelColour[a] = rgb;

		//cout << setprecision(10) << "\n RGB rr : " << rgb.rr;
		//cout << setprecision(10) << "\n RGB gg : " << rgb.gg;
		//cout << setprecision(10) << "\n RGB bb : " << rgb.bb;

		//ii = input


	}



	//cs137intensity = round(240 * (cs137_log - cmin) / (cmax - cmin));











	//cout << "\n Z Min : " << zMin;
	//cout << "\n Z Max : " << zMax;














	
	
	//double cs137intensity;

	


	//points1[maxReading] = points[];

	//std::copy(std::begin(points), std::end(points), std::begin(points1));

	//printx = xList.back();

	//cout << "\n This is X at 2" << printx;

	//getline(file, line);
	//getline(file, line2);



	//cout << "Working: " << line;
	//cout << "\n Working2: " << line2;

	/*
	for (int a = 0; a < maxReading; a++){

		cout << setprecision(10) << "\n x: " << points1[a].x << " y: " << points1[a].y << " z: " << points1[a].z;

	}

	for (int a = 0; a < maxReading; a++){

		cout << setprecision(10) << "\n !x: " << points2[a].x << " y: " << points2[a].y << " z: " << points2[a].z;

	}
	
	


	for (int a = 0; a < maxReading; a++){

		cout << setprecision(10) << "\n pixel no.:" << a << " Diff. x: " << pointsDifference[a].x << " y: " << pointsDifference[a].y << " z: " << pointsDifference[a].z << " Pixel intensity: " << pointsIntensity[a].y;

	}

	
	
	for (int a = 0; a < maxReading; a++){


		cout << setprecision(10) << "\n PixelColour R : " << pixelColour[a].rr << " G : " << pixelColour[a].gg << " B : " << pixelColour[a].rr;
		
	}
	*/
	
	stats statsResult = findMeanAndStd();


	cout << "\n xMean: " << statsResult.xMean << "\n xStd: " << statsResult.xStd 
		<< "\n yMean: " << statsResult.yMean << "\n yStd: " << statsResult.yStd 
		<< "\n zMean: " << statsResult.zMean << "\n zStd: " << statsResult.zStd;








	const char* filenameOutChar = filenameOut.c_str();

	FILE* fp = fopen(filenameOutChar, "wt");

	fprintf(fp, "ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n", maxReading);


	for (int a = 0; a < maxReading; a++){

		//cout << setprecision(10) << "\n" << points1[a].x << " " << points1[a].y << " " << points1[a].z << " " << pixelColour[a].rr << " " << pixelColour[a].gg << " " << pixelColour[a].bb;
		
		fprintf(fp, "%f %f %f %d %d %d\n", points1[a].x, points1[a].y, points1[a].z, pixelColour[a].rr, pixelColour[a].gg, pixelColour[a].bb);
	}

	fclose(fp);

	cout << "\nDone";

	cin.get();




	cin.get();







	return 0;
}

