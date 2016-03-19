#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void findMarks(Mat &input, Mat &output);
void findColourMarks(Mat &input, Mat &output);
void findAlignmentMarks(Mat &input, Mat &output);
void calibrateProjection(Mat &input);
void transform(Mat &output);
void populatedCalibPt();
float calibrateColours(Mat &input, int code);
void populateColourPts(int code, float min, float max);