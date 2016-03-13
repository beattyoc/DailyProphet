#include "Utilities.h"

void play_video(VideoCapture &video)
{

	double count = video.get(CAP_PROP_FRAME_COUNT);
	cout << "Total Frames: " << count << endl;
	double FPS = video.get(CAP_PROP_FPS);
	cout << "Frames Per Second: " << FPS << endl;

	Mat current_frame;

	video >> current_frame;

	while (!current_frame.empty())
	{
		namedWindow("Video Playing", WINDOW_AUTOSIZE);
		imshow("Video Playing", current_frame);
		video >> current_frame;
	}
	//destroyWindow("Video Playing");

}