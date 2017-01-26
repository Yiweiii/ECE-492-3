#ifndef ROBOT_H
#define ROBOT_H 
#include <opencv/cv.hpp>  
#include <string>  

using namespace std;
using namespace cv;

class Robot
{
private:
	int id;
	double xpos;
	double ypos;
	double dir;

	//func a;
	int iLowH;
	int iHighH;
	string pathtemp;
	string imagepath;

public:
	Robot();
	Robot(int ID);
	void recordFrame(VideoCapture);
	void location();
	double getX();
	double getY();
	double getDir();
};

#endif
