/*

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "Utilities.h"
#include <vector>
#include <iostream>
#define CONT vector<Point>

using namespace cv;
using namespace std;

struct FinderPattern{
	Point topleft;
	Point topright;
	Point bottomleft;
	FinderPattern(Point a, Point b, Point c) : topleft(a), topright(b), bottomleft(c) {}
};

// return true if 1 is larger than 2 (fabs returns the modulus)
bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	double i = fabs(contourArea(cv::Mat(contour1)));
	double j = fabs(contourArea(cv::Mat(contour2)));
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
		for (int j = i + 1; j < contours.size(); j++){
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

	cout << "vecpair.size() " << vecpair.size() << endl;

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
			cout << "area1/area2: " << area1 / area2 << endl;
			if (area1 / area2 < minRatio || area1 / area2 > maxRatio){
				cout << "erase\n";
				vecpair.erase(vecpair.begin() + m);
				flag = true;
				break;
			}
		}
		if (!flag){
			++m;
		}
	}
	//cout << "vecpair.size(): " << vecpair.size() << endl;

	//when == 4 send to alignment pairs and take either the largest ratio!
	if (vecpair.size() == 4) {
//		findAlignmentPattern(vecpair, minRatio, maxRatio);
	}
	if (vecpair.size() > 3){
		//eliminates alignment pattern
		eliminatePairs(vecpair, minRatio, maxRatio * 0.9);
	}
}


double getDistance(Point a, Point b){
	// this is probably expensive
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

FinderPattern getFinderPattern(vector<vector<CONT > > &vecpair){
	Point pt1 = getContourCentre(vecpair[0][vecpair[0].size() - 1]);
	Point pt2 = getContourCentre(vecpair[1][vecpair[1].size() - 1]);
	Point pt3 = getContourCentre(vecpair[2][vecpair[2].size() - 1]);
	double d12 = getDistance(pt1, pt2);
	double d13 = getDistance(pt1, pt3);
	double d23 = getDistance(pt2, pt3);
	double x1, y1, x2, y2, x3, y3;
	double Max = max(d12, max(d13, d23));
	// p3 is topleft
	Point p1, p2, p3;
	if (Max == d12){
		p1 = pt1;
		p2 = pt2;
		p3 = pt3;
	}
	else if (Max == d13){
		p1 = pt1;
		p2 = pt3;
		p3 = pt2;
	}
	else if (Max == d23){
		p1 = pt2;
		p2 = pt3;
		p3 = pt1;
	}
	x1 = p1.x;
	y1 = p1.y;
	x2 = p2.x;
	y2 = p2.y;
	x3 = p3.x;
	y3 = p3.y;
	//std::cout << "1: " << x1 << "," << y1 << "\n2: " << x2 << "," << y2 << "\n3: " << x3 << "," << y3 << endl;
	if (x1 == x2){
		if (y1 > y2){
			if (x3 < x1){
				return FinderPattern(p3, p2, p1);
			}
			else{
				return FinderPattern(p3, p1, p2);
			}
		}
		else{
			if (x3 < x1){
				return FinderPattern(p3, p1, p2);
			}
			else{
				return FinderPattern(p3, p2, p1);
			}
		}
	}
	else{
		double newy = (y2 - y1) / (x2 - x1) * x3 + y1 - (y2 - y1) / (x2 - x1) * x1;
		//std::cout << "newy: " << newy << endl;
		if (x1 > x2){
			if (newy < y3){
				//std::cout << "one\n";
				return FinderPattern(p3, p2, p1);
			}
			else{
				//std::cout << "two\n";
				return FinderPattern(p3, p1, p2);
			}
		}
		else{
			if (newy < y3){
				//std::cout << "three\n";
				return FinderPattern(p3, p1, p2);
			}
			else{
				//std::cout << "four\n";
				return FinderPattern(p3, p2, p1);
			}
		}
	}
}


void qr_code(Mat &src1, Mat &src2)
{
	imshow("Original Source", src1);

	Mat gray;
	cvtColor(src1, gray, COLOR_BGR2GRAY);

	Mat binary;
	threshold(gray, binary, 0, 255, THRESH_OTSU);
	//adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 49, 1);
	imshow("Binary", binary);
	
	vector<CONT > contours;
	contours = findLimitedConturs(binary, 8.00, 0.2 * src1.cols * src1.rows);

	if (!contours.empty()) sort(contours.begin(), contours.end(), compareContourAreas);
	vector<vector<CONT > > vecpair = getContourPair(contours);
	//Point alignmentPattern = findAlignmentPattern(vecpair, 1.0, 2.5);
	//cout << "alignmentPattern (" << alignmentPattern.x << "," << alignmentPattern.y << ")\n";
	eliminatePairs(vecpair, 1.0, 10.0);
	//cout << "there are " << vecpair.size() << " pairs left!!" << endl;

	if (vecpair.size() == 3)
	{
		FinderPattern fPattern = getFinderPattern(vecpair);
		//cout << "topleft = " << fPattern.topleft.x << ", " << fPattern.topleft.y << endl
			//<< "topright = " << fPattern.topright.x << ", " << fPattern.topright.y << endl
			//<< "bottomleft = " << fPattern.bottomleft.x << ", " << fPattern.bottomleft.y << endl;

		Mat drawing;
		src1.copyTo(drawing);

		//alignment pattern
		//circle(drawing, alignmentPattern, 3, Scalar(255, 255, 255), 2, 8, 0);

		//finder patterns
		circle(drawing, fPattern.topleft, 3, Scalar(255, 0, 0), 2, 8, 0);
		circle(drawing, fPattern.topright, 3, Scalar(0, 255, 0), 2, 8, 0);
		circle(drawing, fPattern.bottomleft, 3, Scalar(0, 0, 255), 2, 8, 0);

		//affine transformation
		vector<Point2f> vecsrc;
		vector<Point2f> vecdst;
		
		vecsrc.push_back(Point2f(0, 0));
		vecsrc.push_back(Point2f(src2.cols - 1, 0));
		vecsrc.push_back(Point2f(0, src2.rows -1));

		vecdst.push_back(fPattern.topleft);
		vecdst.push_back(fPattern.topright);
		vecdst.push_back(fPattern.bottomleft);
		
		Mat affineTrans = getAffineTransform(vecsrc, vecdst);

		Mat warped;

		warpAffine(src2, warped, affineTrans, src1.size());

		imshow("Finder Patterns", drawing);

		// uncomment below 2 lines to make fullscreen
		namedWindow("Projection", WINDOW_NORMAL);
		setWindowProperty("Projection", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
		imshow("Projection", warped);
	}
}
*/