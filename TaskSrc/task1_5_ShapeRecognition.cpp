#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include<opencv2/opencv.hpp>
#include<string>
#include<iostream>
#include "task1_1_functions.h"

using namespace std;
using namespace cv;

int main()
{
	Mat src, look, lookcircle; //原图像, 找轮廓, 找圆
	int count = 1;
	src = imread("task1_3.png");
	cvtColor(src, look, COLOR_BGR2GRAY);
	cvtColor(src, lookcircle, COLOR_BGR2GRAY);

	threshold(look, look, 0, 255, THRESH_BINARY | THRESH_OTSU); //二值化, 提高与背景对比度

	vector<vector<Point>>pics; //存储所有轮廓
	vector<Vec4i>hierarchy; //轮廓的树结构
	Rect box; //存储最小外接矩形
	//单独储存圆的信息
	vector<Point>point;
	vector<Vec3f>circles;

	//找图中所有轮廓，放在pics
	Mat src_1 = imread("task1_3.png", IMREAD_UNCHANGED);
	Mat matfind, matHSV;
	
	cvtColor(src_1, matHSV, COLOR_BGR2HSV); //产生HSV风格图像便于后期色彩辨识
	cvtColor(src_1, matfind, COLOR_BGR2GRAY); //产生灰度图像进行预处理
	
	//预处理
	equalizeHist(matfind, matfind); //增强对比度
	GaussianBlur(matfind, matfind, Size(3, 3), 3, 3); //降噪
	threshold(matfind, matfind, 0, 255, THRESH_BINARY | THRESH_OTSU); //二值化...好像不成功
	
	findContours(matfind, pics, hierarchy, RETR_LIST, CHAIN_APPROX_NONE); //找轮廓

	HoughCircles(lookcircle, circles, HOUGH_GRADIENT, 1, lookcircle.cols / 10); //找圆

	//开始标记和输出
	Rect tmp_box;
	vector<Point> tmp_point;
	MarkAngle(src, pics, box, tmp_point, matHSV, count);
	DrawOut(src, pics, box, tmp_point);
	MarkCircle(src, circles, matHSV, count);

	namedWindow("result", 0);
	imshow("result", src);
	imwrite("result3.png", src);
	waitKey(0);
	return 0;
}