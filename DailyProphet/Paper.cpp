#include "Utilities.h"

RNG rng(12345);

void DrawCircles(Mat result_image, vector<Vec3f> circles, Scalar passed_colour = -1.0)
{
	int count = 1;
	Mat circle_image = result_image.clone();
	for (vector<cv::Vec3f>::const_iterator current_circle = circles.begin();
		(current_circle != circles.end()); current_circle++)
	{
		Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
		circle(circle_image, Point((*current_circle)[0], (*current_circle)[1]), (*current_circle)[2],
			(passed_colour.val[0] == -1.0) ? colour : passed_colour,
			2);
		int x = (*current_circle)[0];
		int y = (*current_circle)[1];
		int r = (*current_circle)[2];
		cout << "Circle " << count << "\tCentroid (x,y): (" << x << "," << y << ")" << "\tRadius: " << r << endl;
		count++;
	}
	imshow("Circles", circle_image);

	//draws lines
	int count2 = 1;
	for (vector<cv::Vec3f>::const_iterator current_circle2 = circles.begin();
		(current_circle2 != circles.end()); current_circle2++)
	{
		int x = (*current_circle2)[0];
		int y = (*current_circle2)[1];
		//cout << "Circle " << count2 << "\t(" << x << "," << y << ")\n";
		int count3 = 1;
		for (vector<cv::Vec3f>::const_iterator current_circle3 = circles.begin();
			(current_circle3 != circles.end()); current_circle3++)
		{
			int x_curr = (*current_circle3)[0];
			int y_curr = (*current_circle3)[1];
			//cout << "\tCircle " << count3 << "\t(" << x_curr << "," << y_curr << ")\n";
			int x_diff, y_diff;
			x_diff = abs(x - x_curr);
			y_diff = abs(y - y_curr);
			//cout << "x_diff: " << x_diff << "  y_diff: " << y_diff << endl;
			if (x_diff <= 5)
			{
				line(result_image, Point((*current_circle2)[0], (*current_circle2)[1]), Point((*current_circle3)[0], (*current_circle3)[1]), (50,50,50), 1, 8, 0);
			}
			if (y_diff <= 5)
			{
				line(result_image, Point((*current_circle2)[0], (*current_circle2)[1]), Point((*current_circle3)[0], (*current_circle3)[1]), (50, 50, 50), 1, 8, 0);
			}
			if (x_diff <= 10 && y_diff <= 10)
			{
				cout << "concentric!\n";
				cout << "count2: " << count2 << "\tcount3: " << count3 << endl;
				if (count2 != count3)
				{
					Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
					circle(result_image, Point((*current_circle3)[0], (*current_circle3)[1]), (*current_circle3)[2],
						(passed_colour.val[0] == -1.0) ? colour : passed_colour,
						2);
				}
			}
			count3++;
		}
		count2++;
	}
}

void find_circles(Mat &found_paper)
{
	Mat image1_gray;
	cvtColor(found_paper, image1_gray, COLOR_RGB2GRAY);

	// Hough for circles
	vector<Vec3f> circles;
	HoughCircles(image1_gray, circles, HOUGH_GRADIENT, 0.5, 8, 200, 20, 10, 25);//2,20,100,20,5,30);
	Mat hough_circles_image = found_paper.clone();
	DrawCircles(hough_circles_image, circles);
	imshow("Hough circles", hough_circles_image);
}

void find_paper(Mat &src)
{
	Mat gray_image, binary_image, opened_image;
	int thresh_min = 140;

	//
	// create binary image
	//
	cvtColor(src, gray_image, COLOR_RGB2GRAY);
	threshold(gray_image, binary_image, thresh_min, 255, THRESH_BINARY);

	//
	// Connected Components
	//
	Mat five_by_five_element(5, 5, CV_8U, Scalar(1));
	morphologyEx(binary_image, opened_image, MORPH_OPEN, five_by_five_element);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat opened_image_copy = opened_image.clone();

	findContours(opened_image_copy, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	Mat contours_image = Mat::zeros(opened_image.size(), CV_8UC3);
	for (int contour_number = 0; (contour_number<contours.size()); contour_number++)
	{
		Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
		drawContours(contours_image, contours, contour_number, colour, FILLED, 8, hierarchy);
	}

	//
	// Find Largest Component
	//

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	Mat largest_component;
	
	int size_curr, largest_size = 0, largest_i = 0;

	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));

		size_curr = boundRect[i].area();
		if (size_curr > largest_size)
		{
			largest_size = size_curr;
			largest_i = i;
		}
	}

	//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	rectangle(src, boundRect[largest_i].tl(), boundRect[largest_i].br(), (50,50,50), 2, 8, 0);				//apply boudning box
	largest_component = src(Rect(boundRect[largest_i]));									//extract found object
	
	//find_circles(largest_component);
	
	imshow("Largest Component", largest_component);
	imshow("Original", src);

	//imshow("Optimal Threshold", binary_image);
	//imshow("Opening (5x5)", opened_image);
	//imshow("Connected Components", contours_image);
}