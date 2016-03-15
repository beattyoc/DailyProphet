#include "Utilities.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

#define NUM_MARKS 4

using namespace cv;
using namespace std;

int main(int argc, const char** argv)
{

	//
	// open webcam
	//
	//Mat colourSample = imread("Media/colourSamples.png");
	Mat test_src = imread("windows_square.jpg");
	Mat dotSample = imread("Media/dotSample.png");
	Mat cameraPic = imread("Media/cameraPic.jpg");
	Mat sciGal = imread("Media/ScienceGalleryNewsLight.png");
	Mat calibrationImage = imread("Media/calibrationImage.png");
	Mat black = imread("Media/black.png");
	Mat samples = imread("Media/samples_new.png");
	


	
	Mat webcam[7];

	webcam[0] = imread("Media/webcam.jpg");
	webcam[1] = imread("Media/webcam1.jpg");
	webcam[2] = imread("Media/webcam2.jpg");
	webcam[3] = imread("Media/webcam3.jpg");
	webcam[4] = imread("Media/webcam4.jpg");
	webcam[5] = imread("Media/webcam5.jpg");
	webcam[6] = imread("Media/webcam6.jpg");




	/*
	//findMarks(sciGal);
	for (int i = 0; i < 7; i++)
	{
		//findMarks(webcam[i]);
		//waitKey(0);
	}

	Mat cam[2];

	cam[0] = imread("Media/cam0.jpg");
	cam[1] = imread("Media/cam1.jpg");

	for (int i = 0; i < 2; i++)
	{
		findMarks(cam[i]);
		waitKey(0);
	}*/

	/*Mat colours[8];
	colours[0] = imread("Media/circle_bluecentre.png");
	colours[1] = imread("Media/circle_browncentre.png");
	colours[2] = imread("Media/circle_greencentre.png");
	colours[3] = imread("Media/circle_pinkcentre.png");
	colours[4] = imread("Media/circle_purplecentre.png");
	colours[5] = imread("Media/circle_redcentre.png");
	colours[6] = imread("Media/circle_tealcentre.png");
	colours[7] = imread("Media/circle_yellowcentre.png");

	for (int i=0; i < 8; i++)
	{
		findMarks(colours[i]);
	}*/

	//findMarks(sciGal);
	//findMarks(dotSample);
	//qr_code(dotSample, test_src);

	//circle(dotSample, Point(187,203.5), 16, (0,50,100), 2, 8, 0);
	//imshow("dotSample", dotSample);

	/*
	Mat gray, binary;
	cvtColor(dotSample, gray, COLOR_BGR2GRAY);
	threshold(gray, binary, 0, 255, THRESH_OTSU);
	//imshow("Original", colourSample);
	imshow("Gray", gray);
	imshow("Binary", binary);
	waitKey();
	*/

	//findColours(samples);
	






	// ------------- Start up cameras -------------------------
	
	VideoCapture capInput(0); // open the default camera
	if (!capInput.isOpened())  // check if we succeeded
		return -1;
		
	
	VideoCapture capOutput(1);
	if (!capOutput.isOpened())
		return -1;

	//namedWindow("Calibration Image", WINDOW_NORMAL);
	//setWindowProperty("Calibration Image", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	//imshow("Calibration Image", calibrationImage);
	

	bool cameraCalibrated = false, colourCalibrated = false;
	string answer = "";
	Mat input, output, gray, binary;
	float avgHue = 0;
	int iterations = 100;
	int total = iterations;


	// physically align camera and projector
	

	for (;;)
	{
		capInput >> input;

		cvtColor(input, gray, COLOR_BGR2GRAY);
		threshold(gray, binary, 0, 255, THRESH_OTSU);

		imshow("Input", input);
		imshow("Binary", binary);

		if (waitKey(30) >= 0) break;
	}
	

	int count = 0; // need to allow time for frames to set up

	//populatedCalibPt();

	for (;;)
	{
		capInput >> input; 
		capOutput >> output;

		//imshow("original", input);
		//test(input, output);
		//findMarks(input, output);
		
		//----------- Projection Calibration -----------------
		//while (!cameraCalibrated && count > 10)
		if (!cameraCalibrated && count > 1)
		{
			calibrateProjection(input);
			
			//cout << "\nWere the marks correct?: ";
			//string answer;
			//cin >> answer;
			//if (answer == "y")
				cameraCalibrated = true;
		}
		
		//if (count > 1)
			//cameraCalibrated = true;

		// ----------- Colour Calibration ----------------------
		if (cameraCalibrated && !colourCalibrated)
		{
			namedWindow("Black", WINDOW_NORMAL);
			setWindowProperty("Black", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
			imshow("Black", black);
			
			cout << "\nColour Codes:\n0 - Pink\n1 - Blue\n2 - Green\n3 - Gold\n\n";
				//"4 - Purple\n5 - Teal\n6 - Green/Yellow\n7 - Red\n\n";
			for (int j = 0; j < NUM_MARKS; j++)
			{
				imshow("Input", input);
				cout << "\nCalibrate " << j << endl;

				waitKey(0);
				capInput >> input;
				for (int i = 0; i < iterations; i++)
				{
					capInput >> input;
					imshow("Input", input);
					float temp = calibrateColours(input, j);
					//cout << "temp " << temp << endl;
					if (temp > 0)
						avgHue += temp;
					else
						total--;
				}
				avgHue = avgHue / total;
				cout << "avgHue: " << avgHue << endl;

				populateColourPts(j, avgHue - 13, avgHue + 13);

				total = iterations;
				avgHue = 0;
			}
			cout << "Calibration Complete.\n\nSet up for detection...\n\n";
			waitKey(0);
			capInput >> input >> input;
			colourCalibrated = true;
			//capInput >> input;
		}
		
		if (cameraCalibrated && colourCalibrated)
		{
			//test(input, output);
			findMarks(input, output);
			imshow("Input", input);
		}
		

		if (waitKey(30) >= 0) break;
		count++;
	}
	
	// the camera will be deinitialized automatically in VideoCapture destructor
	

	//waitKey(0);
	return 0;
}
