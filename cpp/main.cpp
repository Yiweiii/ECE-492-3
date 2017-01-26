#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv/cv.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <string> 
#include "Robot.h"

using namespace cv;
using namespace std;

//func_a
Mat img, gray, bw;
vector<Vec4i> hierarchy;
vector<vector<Point>> contours;
int threshval = 50;
Rect r;
Rect maxrect, brect;
int idx, n;

int main()
{
	VideoCapture capture(1);
	double rate = capture.get(CV_CAP_PROP_FPS);
	long totalFramenumber = (long)capture.get(CV_CAP_PROP_FRAME_COUNT);
	bool stop(false);

	Robot a(1);
	Robot b(2);

	while (true)
	{
		a.recordFrame(capture);
		a.location();
		cout << "Robot A" << a.getX() << " " << a.getY() << " " << a.getDir() << endl;

		b.recordFrame(capture);
		b.location();
		cout << "Robot B" << b.getX() << " " << b.getY() << " " << b.getDir() << endl;
	}

	capture.release();
	
	system("pause");
	return 0;
}
