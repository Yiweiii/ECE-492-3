#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv/cv.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <string> 
#include <sstream>  
//#include <pthread.h>

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


VideoCapture capture(1);

int iLowS = 90;
int iHighS = 255;

int iLowV = 90;
int iHighV = 255;

string imagepath1, imagepath2, imagepath3, imagepath4, imagepathb;
string pathtemp1; //blue
string pathtemp2; //red
string pathtemp3; //yellow
string pathtemp4; //
string pathtempb; //black
string temp1, temp2, temp3, temp4, tempb;
int countVal1 = 1;
int countVal2 = 1;
//int countVal3 = 1;
//int countVal4 = 1;
//int countValb = 1;

int recordFrame1(int, int, string);
int recordFrame2(int, int, string);
//int recordFrame3(int, int, string);
//int recordFrame4(int, int, string);
//int recordFrameb(int, int, string);
void location1(string);
void location2(string);

//func_b
Mat src1, src2;
Mat src_gray1, src_gray2;
int thresh = 10;
int max_thresh = 255;

int main()
{
	int countPos = 0;

	string path1("C:\\Users\\fywje\\Documents\\video\\1\\");
	string path2("C:\\Users\\fywje\\Documents\\video\\2\\");
	//string path3("C:\\Users\\fywje\\Documents\\video\\3\\");
	//string path4("C:\\Users\\fywje\\Documents\\video\\4\\");
	//string pathb("C:\\Users\\fywje\\Documents\\video\\block\\");

	if (!capture.isOpened())  // if not success, exit program  
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	/*
	pthread_t tid[2];
	pthread_attr_t attr;

	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	*/

	double rate = capture.get(CV_CAP_PROP_FPS);
	long totalFramenumber = (long)capture.get(CV_CAP_PROP_FRAME_COUNT);
	bool stop(false);
	int r1 = path1.find_last_of('\\');
	int r2 = path2.find_last_of('\\');
	//int r3 = path3.find_last_of('\\');
	//int r4 = path4.find_last_of('\\');
	//int rb = pathb.find_last_of('\\');
	pathtemp1.assign(path1, 0, (r1 + 1));
	pathtemp2.assign(path2, 0, (r2 + 1));
	//pathtemp3.assign(path3, 0, (r3 + 1));
	//pathtemp4.assign(path4, 0, (r4 + 1));
	//pathtempb.assign(pathb, 0, (rb + 1));


	while (true)
	{
		if (recordFrame1(90, 120, pathtemp1) == -1)
			break;
		location1(imagepath1);

		if (recordFrame2(160, 179, pathtemp2) == -1)
			break;
		location2(imagepath2);
		
	}

	capture.release();

	system("pause");

	return 0;
}

int recordFrame1(int iLowH, int iHighH, string pathtemp) {

	Mat frame;
	if (!capture.read(frame))
		return -1;

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

	imshow("Original", frame);
	imshow("Thresholded Image1", imgThresholded);

	stringstream ss;
	ss << countVal1++;
	ss >> temp1;

	if (countVal1<10)
	{
		imagepath1 = pathtemp + "image0000" + temp1 + ".jpg";
		imwrite(imagepath1, imgThresholded);
	}
	else if (countVal1<100)
	{
		imagepath1 = pathtemp + "image000" + temp1 + ".jpg";
		imwrite(imagepath1, imgThresholded);
	}
	else if (countVal1<1000)
	{
		imagepath1 = pathtemp + "image00" + temp1 + ".jpg";
		imwrite(imagepath1, imgThresholded);
	}
	else if (countVal1<10000)
	{
		imagepath1 = pathtemp + "image0" + temp1 + ".jpg";
		imwrite(imagepath1, imgThresholded);
	}
	else if (countVal1<100000)
	{
		imagepath1 = pathtemp + "image" + temp1 + ".jpg";
		imwrite(imagepath1, imgThresholded);
	}

	char key = (char)waitKey(300);
	if (key == 27)
		return -1;

	return 0;
}

int recordFrame2(int iLowH, int iHighH, string pathtemp) {

	Mat frame;
	if (!capture.read(frame))
		return -1;

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

	imshow("Thresholded Image2", imgThresholded);

	stringstream ss;
	ss << countVal2++;
	ss >> temp2;

	if (countVal2<10)
	{
		imagepath2 = pathtemp + "image0000" + temp2 + ".jpg";
		imwrite(imagepath2, imgThresholded);
	}
	else if (countVal2<100)
	{
		imagepath2 = pathtemp + "image000" + temp2 + ".jpg";
		imwrite(imagepath2, imgThresholded);
	}
	else if (countVal2<1000)
	{
		imagepath2 = pathtemp + "image00" + temp2 + ".jpg";
		imwrite(imagepath2, imgThresholded);
	}
	else if (countVal2<10000)
	{
		imagepath2 = pathtemp + "image0" + temp2 + ".jpg";
		imwrite(imagepath2, imgThresholded);
	}
	else if (countVal2<100000)
	{
		imagepath2 = pathtemp + "image" + temp2 + ".jpg";
		imwrite(imagepath2, imgThresholded);
	}
	char key = (char)waitKey(300);
	if (key == 27)
		return -1;

	return 0;

}

void location1(string abspath) {
	double coordinate[2][2];
	double xpos = 0;
	double ypos = 0;
	int flag = 0;

	src1 = imread(abspath, CV_LOAD_IMAGE_COLOR);
	cvtColor(src1, src_gray1, CV_BGR2GRAY);
	GaussianBlur(src1, src1, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
	blur(src_gray1, src_gray1, Size(3, 3));
	//namedWindow("image", CV_WINDOW_AUTOSIZE);
	//imshow("image", src1);
	//moveWindow("image", 20, 20);
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Canny(src_gray1, canny_output, thresh, thresh * 3, 3);
	//namedWindow("canny", CV_WINDOW_AUTOSIZE);
	//imshow("canny", canny_output);
	//moveWindow("canny", 550, 20);

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
		xpos += mc[i].x;
		ypos += mc[i].y;
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
	double angle = abs(atan(k) * 180 / 3.14159);
	cout << "INFO Robot A" << endl;
	cout << xpos/4 << " " << "," << " " << ypos/4 << endl;
	cout << angle << endl;

	//waitKey(0);
	src1.release();
	src_gray1.release();
}

void location2(string abspath) {
	double coordinate[2][2];
	double xpos = 0;
	double ypos = 0;
	int flag = 0;

	src2 = imread(abspath, CV_LOAD_IMAGE_COLOR);
	cvtColor(src2, src_gray2, CV_BGR2GRAY);
	GaussianBlur(src2, src2, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
	blur(src_gray2, src_gray2, Size(3, 3));
	//namedWindow("image", CV_WINDOW_AUTOSIZE);
	//imshow("image", src2);
	//moveWindow("image", 20, 20);
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Canny(src_gray2, canny_output, thresh, thresh * 3, 3);
	//namedWindow("canny", CV_WINDOW_AUTOSIZE);
	//imshow("canny", canny_output);
	//moveWindow("canny", 550, 20);

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
		xpos += mc[i].x;
		ypos += mc[i].y;
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
	double angle = abs(atan(k) * 180 / 3.14159);
	cout << "INFO Robot B" << endl;
	cout << xpos / 4 << " " << "," << " " << ypos / 4 << endl;
	cout << angle << endl;

	//waitKey(0);
	src2.release();
	src_gray2.release();
}