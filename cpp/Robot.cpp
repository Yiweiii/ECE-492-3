#include "Robot.h"

Robot::Robot()
{
	id = 0;
	xpos = 0.0;
	ypos = 0.0;
	dir = 0.0;
	iLowH = 0;
	iHighH = 0;
}

Robot::Robot(int ID)
{
	id = ID;
	switch(ID)
	{
	case 1:
		iLowH = 90;
		iHighH = 120;
		pathtemp = "C:\\Users\\fywje\\Documents\\video\\1\\";
		break;
	case 2:
		iLowH = 160;
		iHighH = 179;
		pathtemp = "C:\\Users\\fywje\\Documents\\video\\2\\";
		break;
	default:
		break;
	}
}

double Robot::getX()
{
	return xpos;
}

double Robot::getY()
{
	return ypos;
}

double Robot::getDir()
{
	return dir;
}

void Robot::recordFrame(VideoCapture capture)
{

	int iLowS = 90;
	int iHighS = 255;

	int iLowV = 90;
	int iHighV = 255;

	string temp;
	int r = pathtemp.find_last_of('\\');
	temp.assign(pathtemp, 0, (r + 1));

	Mat frame;
	capture.read(frame);

	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(frame, imgHSV, COLOR_BGR2HSV);

	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

	string tempThresPath = "Thresholded Image" + id;
	imshow("Original", frame);
	imshow(tempThresPath, imgThresholded);

	imagepath = temp + "image" + ".jpg";
	imwrite(imagepath, imgThresholded);

	char key = (char)waitKey(300);
}

void Robot::location() {
	Mat src;
	Mat src_gray;
	int thresh = 10;
	int max_thresh = 255;

	double coordinate[2][2];
	double tempx = 0;
	double tempy = 0;
	int flag = 0;

	src = imread(imagepath, CV_LOAD_IMAGE_COLOR);
	cvtColor(src, src_gray, CV_BGR2GRAY);
	GaussianBlur(src, src, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
	blur(src_gray, src_gray, Size(3, 3));

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Canny(src_gray, canny_output, thresh, thresh * 3, 3);

	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	vector<Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) < 50)
			continue;
		mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		tempx += mc[i].x;
		tempy += mc[i].y;
		if (i % 2) {
			if (flag == 0) {
				coordinate[0][0] = mc[i].x;
				coordinate[0][1] = mc[i].y;
				flag = 1;
			}
			else {
				coordinate[1][0] = mc[i].x;
				coordinate[1][1] = mc[i].y;
				flag = 0;
			}
		}
	}

	double k = (double)(coordinate[0][1] - coordinate[1][1]) / (coordinate[0][0] - coordinate[1][0]);

	dir = abs(atan(k) * 180 / 3.14159);
	xpos = tempx / 4;
	ypos = tempy / 4;

	src.release();
	src_gray.release();
}