#include "Utilities.h"

// Draw a passed line using a random colour if one is not provided
void DrawLine(Mat result_image, Point point1, Point point2, Scalar passed_colour = -1.0)
{
	Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
	line(result_image, point1, point2, (passed_colour.val[0] == -1.0) ? colour : passed_colour);
}

/*
// Draw line segments delineated by end points
void DrawLines(Mat result_image, vector<Vec4i> lines, Scalar passed_colour = -1.0)
{
	for (vector<cv::Vec4i>::const_iterator current_line = lines.begin();
		(current_line != lines.end()); current_line++)
	{
		Point point1((*current_line)[0], (*current_line)[1]);
		Point point2((*current_line)[2], (*current_line)[3]);
		DrawLine(result_image, point1, point2, passed_colour);
	}
}

// Draw lines defined by rho and theta parameters
void DrawLines(Mat result_image, vector<Vec2f> lines, Scalar passed_colour = -1.0)
{
	for (vector<cv::Vec2f>::const_iterator current_line = lines.begin();
		(current_line != lines.end()); current_line++)
	{
		float rho = (*current_line)[0];
		float theta = (*current_line)[1];
		// To avoid divide by zero errors we offset slightly from 0.0
		float cos_theta = (cos(theta) == 0.0) ? 0.000000001 : cos(theta);
		float sin_theta = (sin(theta) == 0.0) ? 0.000000001 : sin(theta);
		Point left(rho / cos(theta), 0.0);
		Point right((rho - (result_image.rows - 1)*sin(theta)) / cos(theta), (result_image.rows - 1));
		Point top(0.0, rho / sin(theta));
		Point bottom((result_image.cols - 1), (rho - (result_image.cols - 1)*cos(theta)) / sin(theta));
		Point* point1 = NULL;
		Point* point2 = NULL;
		if ((left.y >= 0.0) && (left.y <= (result_image.rows - 1)))
			point1 = &left;
		if ((right.y >= 0.0) && (right.y <= (result_image.rows - 1)))
			if (point1 == NULL)
				point1 = &right;
			else point2 = &right;
			if ((point2 == NULL) && (top.x >= 0.0) && (top.x <= (result_image.cols - 1)))
				if (point1 == NULL)
					point1 = &top;
				else if ((point1->x != top.x) || (point1->y != top.y))
					point2 = &top;
			if (point2 == NULL)
				point2 = &bottom;
			DrawLine(result_image, *point1, *point2, passed_colour);
	}
}
*/
void canny_demo(Mat &src)
{
	// Contours and straight line segments
	Mat canny_edge_image;
	Canny(src, canny_edge_image, 125, 350);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat canny_edge_image_copy = canny_edge_image.clone();
	findContours(canny_edge_image_copy, contours, hierarchy, RETR_TREE ,CHAIN_APPROX_NONE);

	vector<Vec4i> line_segments;
	vector<vector<Point>> approx_contours(contours.size());
	for (int contour_number = 0; (contour_number<contours.size()); contour_number++)
	{	// Approximate each contour as a series of line segments.
		approxPolyDP(Mat(contours[contour_number]), approx_contours[contour_number], 3, true);
	}
	// Extract line segments from the contours.
	for (int contour_number = 0; (contour_number<contours.size()); contour_number++)
	{
		for (int line_segment_number = 0; (line_segment_number<approx_contours[contour_number].size() - 1); line_segment_number++)
		{
			line_segments.push_back(Vec4i(approx_contours[contour_number][line_segment_number].x, approx_contours[contour_number][line_segment_number].y,
				approx_contours[contour_number][line_segment_number + 1].x, approx_contours[contour_number][line_segment_number + 1].y));
		}
	}
	// Draw the contours and then the segments
	Mat contours_image = Mat::zeros(canny_edge_image.size(), CV_8UC3);
	//Mat line_segments_image = Mat::zeros(canny_edge_image.size(), CV_8UC3);
	for (int contour_number = 0; (contour_number<contours.size()); contour_number++)
	{
		Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
		drawContours(contours_image, contours, contour_number, colour, 1, 8, hierarchy);
	}
	//DrawLines(line_segments_image, line_segments);
	imshow("Original", src);
	imshow("Canny edges", canny_edge_image);
	imshow("Individual contours", contours_image);
	//imshow("Line segments", line_segments_image);
}