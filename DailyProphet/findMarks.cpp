#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "Utilities.h"
#include <vector>
#include <iostream>

#define CONT vector<Point>
#define NUM_MARKS 4

using namespace cv;
using namespace std;

Point2f gold(0, 0), pink(0, 0), blue(0, 0), green(0, 0), yg(0, 0), purple(0, 0), red(0, 0), teal(0, 0);
Point2f TL(0, 0), TR(0, 0), BL(0, 0), BR(0, 0);

bool identified[NUM_MARKS];

float goldMin = 0, goldMax = 0, 
	pinkMin = 0, pinkMax = 0,
	blueMin = 0, blueMax = 0,
	greenMin = 0, greenMax = 0,
	ygMin = 0, ygMax = 0,
	purpleMin = 0, purpleMax = 0,
	redMin = 0, redMax = 0,
	tealMin = 0, tealMax = 0;

// hard coded values for testing when a projector is not available
void populatedCalibPt()
{
	TL = Point2f(20, 20);
	TR = Point2f(360, 20);
	BR = Point2f(360, 290);
	BL = Point2f(20, 290);
	
}

// ---------------- functions for locating markers -------------------------------

// return true if 1 is larger than 2 (fabs returns the modulus)
bool compareContourAreas(vector<Point> contour1, vector<Point> contour2) {
	double i = fabs(contourArea(Mat(contour1)));
	double j = fabs(contourArea(Mat(contour2)));
	return (i > j);
}

Point getContourCentre(CONT& vec){
	double tempx = 0.0, tempy = 0.0;
	for (int i = 0; i<vec.size(); i++){
		tempx += vec[i].x;
		tempy += vec[i].y;
	}
	return Point(tempx / (double)vec.size(), tempy / (double)vec.size());
}

bool isContourInsideContour(CONT& in, CONT& out){
	//every point of in must be within the contour out
	for (int i = 0; i<in.size(); i++){
		// checks if point in[i] is inside contour out (returns positive if inside contour
		// returns negative if outside or returns zero if on edge
		// because the third parameter is set to false the function 
		// returns -1, +1 or 0
		if (pointPolygonTest(out, in[i], false) <= 0) return false; // if outside return false
	}
	//if no points are outside then the contour is inside
	return true;
}

vector<CONT > findLimitedConturs(Mat contour, float minPix, float maxPix){
	//cout << "min: " << minPix << "\tmax: " << maxPix << endl;
	vector<CONT > contours; //will be populated with all found contours by findContours each contour is a vector of points
	vector<Vec4i> hierarchy; //output vector containing info about contour topology, e.g. the number of contours
	//RETR_TREE, constructs a full hierarchy of nested contours
	//CHAIN_APPROX_SIMPLE, compresses horizontal, vertical, and diagonal segments and leaves only their end points. 
	//For example, an up-right rectangular contour is encoded with 4 points
	findContours(contour, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	//cout << "contours.size = " << contours.size() << endl;
	int m = 0;
	// for each contour eliminate too big and too small
	while (m < contours.size()){
		if (contourArea(contours[m]) <= minPix){
			contours.erase(contours.begin() + m);
		}
		else if (contourArea(contours[m]) > maxPix){
			contours.erase(contours.begin() + m);
		}
		else ++m;
	}
	//cout << "contours.size() = " << contours.size() << endl;
	return contours;
}

// if contour is inside another contour return
vector<vector<CONT > > getContourPair(vector<CONT > &contours){
	vector<vector<CONT > > vecpair;
	vector<bool> bflag(contours.size(), false);

	for (int i = 0; i<contours.size() - 1; i++){
		// if i is true skip to next i (like if j found it to be true)
		if (bflag[i]) continue;
		vector<CONT > temp;
		//adds element to temp
		temp.push_back(contours[i]);
		for (int j = i + 1; j < contours.size(); j++){ // all the larger contours
			if (isContourInsideContour(contours[j], contours[i])){
				// if j is in i, then add to temp
				temp.push_back(contours[j]);
				bflag[j] = true;
			}
		}
		// if it is a pair
		if (temp.size() > 1){
			vecpair.push_back(temp);
		}
	}
	//cout << "vecpair.size() " << vecpair.size() << endl;
	bflag.clear();
	for (int i = 0; i<vecpair.size(); i++){
		sort(vecpair[i].begin(), vecpair[i].end(), compareContourAreas);
	}
	//cout << "vecpair.size(): " << vecpair.size() << endl;
	return vecpair;
}

void eliminatePairs(vector<vector<CONT > >& vecpair, double minRatio, double maxRatio){
	//cout << "minRation: " << minRatio << "\tmaxRatio: " << maxRatio << endl;
	int m = 0;
	bool flag = false;
	while (m < vecpair.size()){
		flag = false;
		if (vecpair[m].size() < 3){
			vecpair.erase(vecpair.begin() + m);
			continue;
		}
		for (int i = 0; i<vecpair[m].size() - 1; i++){
			double area1 = contourArea(vecpair[m][i]);
			double area2 = contourArea(vecpair[m][i + 1]);
			//cout << "area1/area2: " << area1 / area2 << endl;
			if (area1 / area2 < minRatio || area1 / area2 > maxRatio){
				//cout << "erase\n";
				vecpair.erase(vecpair.begin() + m);
				flag = true;
				break;
			}
		}
		if (!flag){ // if not erased continue to next vecpair
			++m;
		}
	}
}

// --------- calibrate projection ------------------
void populateCalibrationPts(vector<Point2f> &center, Mat &input)
{
	vector<bool> isIdentified(center.size(), false);
	Point2f topLeft, topRight, bottomLeft, bottomRight;
	float topLeftSum, bottomRightSum;
	int topLeftPos = 0;

	topLeft.x = bottomRight.x = center[0].x;
	topLeft.y = bottomRight.y = center[0].y;
	topLeftSum = bottomRightSum = center[0].x + center[0].y;
	isIdentified[0] = true;

	for (int i = 1; i < center.size(); i++)
	{
		if ((center[i].x + center[i].y) < topLeftSum)
		{
			topLeftSum = center[i].x + center[i].y;
			topLeft = center[i];
			topLeftPos = i;
			for (int j = 0; j < center.size(); j++)
			{
				if (j != i)
					isIdentified[j] = false;
				else isIdentified[j] = true;
			}
		}
	}
	for (int i = 1; i < center.size(); i++)
	{
		if ((center[i].x + center[i].y) > bottomRightSum)
		{
			bottomRightSum = center[i].x + center[i].y;
			bottomRight = center[i];
			for (int j = 0; j < center.size(); j++)
			{
				if ((j != i) && (j != topLeftPos))
					isIdentified[j] = false;
				else isIdentified[j] = true;
			}
		}
	}

	for (int i = 0; i < center.size(); i++)
	{
		if (!isIdentified[i])
		{
			bottomLeft = center[i];
			for (int j = 0; j < center.size(); j++)
			{
				if ((!isIdentified[j]) && (i != j))
				{
					topRight = center[j];
					if (topRight.x < bottomLeft.x)
					{
						bottomLeft = center[j];
						topRight = center[i];
					}
				}
			}	
		}
	}

	TL = topLeft;
	TR = topRight;
	BR = bottomRight;
	BL = bottomLeft;
	
	Scalar color = Scalar(0, 255, 255);
	Mat calibration = input.clone();

	/*
	cout << "Top Left: (" << topLeft.x << "," << topLeft.y << ")\n";
	circle(calibration, topLeft, 20, color, 2, 8, 0);
	imshow("Calibration", calibration);
	waitKey(0);

	cout << "Bottom Right: (" << bottomRight.x << "," << bottomRight.y << ")\n";
	circle(calibration, bottomRight, 20, color, 2, 8, 0);
	imshow("Calibration", calibration);
	waitKey(0);

	cout << "Top Right: (" << topRight.x << "," << topRight.y << ")\n";
	circle(calibration, topRight, 20, color, 2, 8, 0);
	imshow("Calibration", calibration);
	waitKey(0);

	cout << "Bottom Left: (" << bottomLeft.x << "," << bottomLeft.y << ")\n";
	circle(calibration, bottomLeft, 20, color, 2, 8, 0);
	
	imshow("Calibration", calibration);
	waitKey(0);
	*/
}

void calibrateProjection(Mat &input)
{
	Mat gray, binary;
	cvtColor(input, gray, COLOR_BGR2GRAY);
	threshold(gray, binary, 0, 255, THRESH_OTSU);
	imshow("Binary", binary);

	//-------------- find patternss ----------------
	vector<CONT > contours;
	vector<Vec4i> hierarchy;
	contours = findLimitedConturs(binary, 8.00, 0.2 * input.cols * input.rows);
	if (!contours.empty()) sort(contours.begin(), contours.end(), compareContourAreas);
	vector<vector<CONT > > vecpair = getContourPair(contours);
	eliminatePairs(vecpair, 1.0, 10.0);
	
	//---- if a pattern is found -----------
	cout << "Marks Found: " << vecpair.size() << endl;
	if (vecpair.size() == 4)
	{
		/// Approximate contours to polygons + get bounding rects and circles
		vector<vector<Point> > contours_poly(vecpair.size());
		//vector<Rect> boundRect(vecpair.size());
		vector<Point2f> center(vecpair.size());
		vector<float> radius(vecpair.size());

		// finding center points
		for (int i = 0; i < vecpair.size(); i++)
		{
			approxPolyDP(Mat(vecpair.at(i).at(2)), contours_poly[i], 3, true);
			minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
		}

		// identify found points
		populateCalibrationPts(center, input);
	}
	else cout << "Didn't find 4 marks!";
}


// --------- calibrate colours ----------------
void populateColourPts(int code, float min, float max)
{
	if (code == 0) // pink
	{
		pinkMin = min;
		pinkMax = max;
		cout << "pinkMin: " << pinkMin << "\t pinkMax: " << pinkMax << endl;
	}
	else if (code == 1) // blue
	{
		blueMin = min;
		blueMax = max;
		cout << "blueMin: " << blueMin << "\t blueMax: " << blueMax << endl;
	}
	else if (code == 2) // green
	{
		greenMin = min;
		greenMax = max;
		cout << "greenMin: " << greenMin << "\t greenMax: " << greenMax << endl;
	}
	else if (code == 3) // gold
	{
		goldMin = min;
		goldMax = max;
		cout << "goldMin: " << goldMin << "\t goldMax: " << goldMax << endl;
	}
	else if (code == 4) // purple
	{
		purpleMin = min;
		purpleMax = max;
		cout << "purpleMin: " << purpleMin << "\t purpleMax: " << purpleMax << endl;
	}
	else if (code == 5) // teal
	{
		tealMin = min;
		tealMax = max;
		cout << "tealMin: " << tealMin << "\t tealMax: " << tealMax << endl;
	}
	else if (code == 6) // yellow green
	{
		ygMin = min;
		ygMax = max;
		cout << "ygMin: " << ygMin << "\t ygMax: " << ygMax << endl;
	}
	else if (code == 7) // red
	{
		redMin = min;
		redMax = max;
		cout << "redMin: " << redMin << "\t redMax: " << redMax << endl;
	}
}

float calibrateColours(Mat &input, int code)
{
	Mat gray, binary;
	cvtColor(input, gray, COLOR_BGR2GRAY);
	threshold(gray, binary, 0, 255, THRESH_OTSU);
	//imshow("Binary", binary);

	//-------------- find patternss ----------------
	vector<CONT > contours;
	vector<Vec4i> hierarchy;
	contours = findLimitedConturs(binary, 8.00, 0.2 * input.cols * input.rows);
	if (!contours.empty()) sort(contours.begin(), contours.end(), compareContourAreas);
	vector<vector<CONT > > vecpair = getContourPair(contours);
	eliminatePairs(vecpair, 1.0, 10.0);

	//---- if a pattern is found -----------
	if (vecpair.size() == 1)
	{
		/// Approximate contours to polygons + get bounding rects and circles
		vector<vector<Point> > contours_poly(vecpair.size());
		vector<Rect> boundRect(vecpair.size());

		approxPolyDP(Mat(vecpair.at(0).at(2)), contours_poly[0], 3, true);
		boundRect[0] = boundingRect(Mat(contours_poly[0]));
		Rect ROI(boundRect[0].tl(), boundRect[0].br());

		Mat hls;
		vector<Mat> channels;

		cvtColor(input, hls, COLOR_BGR2HLS);
		split(hls, channels);

		Mat crop = channels[0](ROI);
		Scalar meanHue = mean(crop);
	
		return meanHue[0];
	}
}

//------------------------------- identify found mark ------------------------

void identifyMark(Scalar m, Point2f centre)
{
	/*
	cout << endl;
	cout << pinkMin << " < Pink < " << pinkMax << endl;
	cout << blueMin << " < Blue < " << blueMax << endl;
	cout << greenMin << " < Green < " << greenMax << endl;
	cout << goldMin << " < Gold < " << goldMax << endl << endl;*/

	float hue = m[0];
	
	if ((hue >= redMin) && (hue <= redMax))
	{
		red = centre;
		cout << "Red: " << red << endl;
	}
	else if ((hue >= ygMin) && (hue <= ygMax))
	{
		yg = centre;
		cout << "Yellow Green: " << yg << endl;
	}
	else if ((hue >= goldMin) && (hue <= goldMax))
	{
		gold = centre;
		identified[3] = true;
		cout << "Gold: " << gold << endl;
	}
	else if ((hue >= greenMin) && (hue <= greenMax))
	{
		green = centre;
		identified[2] = true;
		cout << "Green: " << green << endl;
	}
	else if ((hue >= tealMin) && (hue <= tealMax))
	{
		teal = centre;
		cout << "Teal: " << teal << endl;
	}
	else if ((hue >= blueMin) && (hue <= blueMax))
	{
		blue = centre;
		identified[1] = true;
		cout << "Blue: " << blue << endl;
	}
	else if ((hue >= purpleMin) && (hue <= purpleMax))
	{
		purple = centre;
		cout << "Purple: " << purple << endl;
	}
	else if ((hue >= pinkMin) && (hue <= pinkMax))
	{
		pink = centre;
		identified[0] = true;
		cout << "Pink: " << pink << endl;
	}
	else
	{
		cout << "\nUnknown Hue!\n\n";
	}
}

void drawCircles(Mat &input)
{
	circle(input, red, 10, (0,0,255), 2, 8, 0);
	circle(input, gold, 10, (0, 255, 255), 2, 8, 0);
	circle(input, pink, 10, (255, 0, 255), 2, 8, 0);
	circle(input, green, 10, (0, 255, 0), 2, 8, 0);
	circle(input, yg, 10, (0, 128, 255), 2, 8, 0);
	circle(input, purple, 10, (255, 0, 127), 2, 8, 0);
	circle(input, teal, 10, (255, 255, 0), 2, 8, 0);
	circle(input, blue, 10, (255, 0, 0), 2, 8, 0);
}


// ------------------------- main function ---------------------------------

void findMarks(Mat &input, Mat &output)
{
	//-------------- get binary image -------------
	Mat gray, binary, hls, hsv;
	vector<Mat> channels;
	cvtColor(input, gray, COLOR_BGR2GRAY);
	threshold(gray, binary, 0, 255, THRESH_OTSU);
	imshow("Input", input);
	imshow("Binary", binary);


	//-------------- find patternss ----------------
	vector<CONT > contours;
	vector<Vec4i> hierarchy;
	contours = findLimitedConturs(binary, 8.00, 0.2 * input.cols * input.rows);
	if (!contours.empty()) sort(contours.begin(), contours.end(), compareContourAreas); // areas sorted from smallest to largest
	vector<vector<CONT > > vecpair = getContourPair(contours);
	eliminatePairs(vecpair, 1.0, 10.0);

	for (int i = 0; i < NUM_MARKS; i++)
		identified[i] = false;
	

	//---- if a pattern is found -----------
	if (!vecpair.empty())
	{
		cvtColor(input, hls, COLOR_BGR2HLS);
		split(hls, channels);

		/// Approximate contours to polygons + get bounding rects and circles
		vector<CONT > contours_poly(vecpair.size());
		vector<Rect> boundRect(vecpair.size());
		vector<Point2f> center(vecpair.size());
		vector<float> radius(vecpair.size());

		// isolating the colour on the inner circle
		for (int i = 0; i < vecpair.size(); i++)
		{
			approxPolyDP(Mat(vecpair.at(i).at(2)), contours_poly[i], 3, true);
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
			minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
		}

		/// Draw polygonal contour + bonding rects + circles
		cout << "Marks Found: " << vecpair.size() << endl;

		// for each found mark
		for (int i = 0; i < vecpair.size(); i++)
		{
			Rect ROI(boundRect[i].tl(), boundRect[i].br());
			

			Mat cropHue = channels[0](ROI);
			Mat crop = input(ROI);

			Scalar color = mean(crop);
			Scalar meanHue = mean(cropHue);
	
			//cout << "Mean Hue: " << meanHue[0] << endl;
			//imshow("Crop", crop);
			//Mat contours;
			//drawContours(contours, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			//imshow("Contours", contours);
			//rectangle(image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			//cout << "boundRect[i].tl() " << boundRect[i].tl() << endl;
			//cout << "boundRect[i].br() " << boundRect[i].br() << endl;
			//cout << "Point Location " << center[i] << endl << endl;
			//cout << "Radius: " << (int)radius[i] << endl;

			circle(input, center[i], 20, color, 2, 8, 0);

			identifyMark(meanHue, center[i]);
			//imshow("Found Marks", input);
			//waitKey(0);

		}
		cout << endl << endl;
		//imshow("Found Marks", input);

		bool found = true;
		for (int i = 0; i < NUM_MARKS; i++)
		{
			if (!identified[i])
				found = false;
		}

		if (found)
			transform(output);

		//drawCircles(image);
	}
}

Mat getOuterToInner()
{
	vector<Point2f> outerNewsVec, innerNewsVec;

	outerNewsVec.push_back(Point2f(52, 55)); // Top Left
	outerNewsVec.push_back(Point2f(590, 55)); // Top Right
	outerNewsVec.push_back(Point2f(590, 427)); // Bottom Right
	outerNewsVec.push_back(Point2f(52, 427)); // Bottom Left

	innerNewsVec.push_back(Point2f(274, 145)); // Top Left
	innerNewsVec.push_back(Point2f(535, 145)); // Top Right
	innerNewsVec.push_back(Point2f(535, 332)); // Bottom Right
	innerNewsVec.push_back(Point2f(274, 332)); // Bottom Left

	return getPerspectiveTransform(outerNewsVec, innerNewsVec);
}

void transform(Mat &output)
{
	vector<Point2f> calibrationVec, newspaperVec, outerNewsVec, innerNewsVec;
	Mat calibToNewsTrans, outerToInnerTrans, outerNews, innerNews;

	calibrationVec.push_back(TL); // topLeft
	calibrationVec.push_back(TR); // topRight
	calibrationVec.push_back(BR); // bottomRight
	calibrationVec.push_back(BL); // bottomLeft
	
	/*
	calibrationVec.push_back(Point2f(0,0)); // topLeft
	calibrationVec.push_back(Point2f(output.cols - 1, 0)); // topRight
	calibrationVec.push_back(Point2f(output.cols - 1, output.rows - 1)); // bottomRight
	calibrationVec.push_back(Point2f(0, output.rows - 1)); // bottomLeft
	*/
	// currently hardcoded
	newspaperVec.push_back(Point2f(pink.x, pink.y)); //topLeft
	newspaperVec.push_back(Point2f(blue.x, blue.y)); //topRight
	newspaperVec.push_back(Point2f(green.x, green.y)); //bottomRight
	newspaperVec.push_back(Point2f(gold.x, gold.y)); //bottomLeft

	calibToNewsTrans = getPerspectiveTransform(calibrationVec, newspaperVec);
	warpPerspective(output, outerNews, calibToNewsTrans, output.size());

	outerToInnerTrans = getOuterToInner();
	warpPerspective(outerNews, innerNews, outerToInnerTrans, output.size());
	
	namedWindow("Output", WINDOW_NORMAL);
	setWindowProperty("Output", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	//imshow("Output", outerNews);
	imshow("Output", innerNews);
}
