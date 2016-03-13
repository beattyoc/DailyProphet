#include "Utilities.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


void cornerHarris_demo(Mat &src)
{
	Mat src_gray, harris_cornerness, harris_corners;
	cvtColor(src, src_gray, COLOR_BGR2GRAY);

	cornerHarris(src_gray, harris_cornerness, 3, 3, 0.02);
	imshow("Original", src_gray);

	GoodFeaturesToTrackDetector harris_detector(1000, 0.01, 10, 3, true);
	vector<KeyPoint> keypoints;
	harris_detector.detect(src_gray, keypoints);
	drawKeypoints(src, keypoints, harris_corners, Scalar(0, 0, 255));
	imshow("Harris Corners", harris_corners);

}