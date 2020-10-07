#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include<opencv2/calib3d.hpp>
#include<iostream>
#include<string>

using namespace cv;

std::vector<Point3f> objectPoints;
std::vector<Point2d> imagePoints;

int findRealContour(std::vector<std::vector<Point>>pics, std::vector<Point>point, Rect box, Point& left_up)
{
	for (size_t t = 0; t < pics.size(); t++)
	{
		Point point1, point2;

		box = boundingRect(pics[t]);
		point1 = Point(box.x, box.y);
		point2 = Point(box.x + box.width, box.y + box.height);

		int epsilon = 0.01 * arcLength(pics[t], true); //提高精度

		approxPolyDP(pics[t], point, epsilon, true);
		if (point.size() == 4 && contourArea(pics[t]) > 90000 && arcLength(pics[t], true) > 1600)
		{
			imagePoints.clear();
			imagePoints.push_back(point[2]);
			imagePoints.push_back(point[3]);
			imagePoints.push_back(point[0]);
			imagePoints.push_back(point[1]);
			left_up = point[0];
			/*std::cout << point[0] << std::endl;
			std::cout << point[1] << std::endl;
			std::cout << point[2] << std::endl;
			std::cout << point[3] << std::endl;*/
			return t;
		}
	}
}

int main()
{
	objectPoints.clear();
	objectPoints.push_back(Point3f(+33.2f, +33.2f, 0));
	objectPoints.push_back(Point3f(-33.2f, +33.2f, 0));
	objectPoints.push_back(Point3f(-33.2f, -33.2f, 0));
	objectPoints.push_back(Point3f(+33.2f, -33.2f, 0));

	Mat img = imread("dis_7.jpg"), src_1;

	cvtColor(img, src_1, COLOR_BGR2GRAY);
	GaussianBlur(src_1, src_1, Size(3, 3), 3, 3);
	Canny(src_1, src_1, 35, 125);

	std::vector<std::vector<Point>>pics;
	std::vector<Vec4i>hierarchy;
	findContours(src_1, pics, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

	std::vector<Point> point_t;
	Rect box_t;
	Point left_up;
	int picsId = findRealContour(pics, point_t, box_t, left_up);

	float F[3][3] = { 5.3910270113165666e+02, 0., 3.3437156013314626e+02, 0.,
	   5.3898388577579635e+02, 2.4144720360478962e+02, 0., 0., 1. };
	float G[5][1] = { -2.8615912313531972e-01, 9.6011874888963913e-02,
	   5.0214931587498190e-04, -7.7460234373011869e-04, 0. };

	Mat cameraMat, distCoeff;
	cameraMat = Mat(3, 3, CV_32F, F), distCoeff = Mat(5, 1, CV_32F, G);

	Mat rvecs = Mat::zeros(3, 1, CV_64FC1);
	Mat tvecs = Mat::zeros(3, 1, CV_64FC1);

	solvePnP(objectPoints, imagePoints, cameraMat, distCoeff, rvecs, tvecs);

	Mat rotationMat = Mat::eye(3, 3, CV_64FC1);
	Rodrigues(rvecs, rotationMat);

	float theta_x, theta_y, theta_z;
	theta_x = atan2(rotationMat.at<double>(2, 1), rotationMat.at<double>(2, 2)) * 57.2958;
	theta_y = atan2(-rotationMat.at<double>(2, 0), sqrt(rotationMat.at<double>(2, 0) * rotationMat.at<double>(2, 0) + rotationMat.at<double>(2, 2) * rotationMat.at<double>(2, 2))) * 57.2958;
	theta_z = atan2(rotationMat.at<double>(1, 0), rotationMat.at<double>(0, 0)) * 57.2958;
	
	Mat P;
	P = (rotationMat.t()) * tvecs;

	std::cout << "角度" << std::endl;
	std::cout << theta_x << std::endl;
	std::cout << theta_y << std::endl;
	std::cout << theta_z << std::endl;
	std::cout << P << std::endl;

	drawContours(img, pics, picsId, Scalar(255, 0, 255), 5);
	putText(img, std::to_string(P.at<double>(2, 0)), left_up, FONT_HERSHEY_PLAIN, 10, Scalar(255, 0, 255), 5, 8);
	namedWindow("test", 0);
	imshow("test", img);
	waitKey();
}