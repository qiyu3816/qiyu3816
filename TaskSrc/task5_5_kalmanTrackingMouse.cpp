#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include<iostream>

using namespace cv;

void mouseMoveCallback(int event, int x, int y, int flags, void* userData);
void drawCross(cv::Mat& img, cv::Point center, cv::Scalar color);

Point mousePos(0, 0);

int main(void) {

    KalmanFilter kalmanFilter(4, 2, 0); //状态向量四维, 测量变量二维, 无控制向量

    float F[4][4] = { { 1, 0, 1, 0 },
                      { 0, 1, 0, 1 },
                      { 0, 0, 1, 0 },
                      { 0, 0, 0, 1 } };
    kalmanFilter.transitionMatrix = Mat(4, 4, CV_32F, F);

    Mat matMeasurement(2, 1, CV_32F, Scalar::all(0));

    kalmanFilter.statePre.at<float>(0) = (float)mousePos.x;
    kalmanFilter.statePre.at<float>(1) = (float)mousePos.y;
    kalmanFilter.statePre.at<float>(2) = 0;
    kalmanFilter.statePre.at<float>(3) = 0;

    setIdentity(kalmanFilter.measurementMatrix); //H
    setIdentity(kalmanFilter.processNoiseCov, Scalar::all(1e-4)); //Q
    setIdentity(kalmanFilter.measurementNoiseCov, Scalar::all(10)); //R
    setIdentity(kalmanFilter.errorCovPost, Scalar::all(.1)); //\sigma_k

    Mat image(500, 500, CV_8UC3);

    std::vector<Point> mousevec, kalmanvec; //这是为了画出轨迹而准备的点的向量组

    mousevec.clear();
    kalmanvec.clear();

    namedWindow("image");
    setMouseCallback("image", mouseMoveCallback);

    while (1)
    {
        Mat matPrediction = kalmanFilter.predict(); // function predict() computes a predicted state

        Point predictedPoint((int)matPrediction.at<float>(0), (int)matPrediction.at<float>(1));     // this does not seem to be used ??

        matMeasurement.at<float>(0, 0) = (float)mousePos.x;
        matMeasurement.at<float>(1, 0) = (float)mousePos.y;

        Mat estimated = kalmanFilter.correct(matMeasurement);        // function correct() updates the predicted state from the measurement

        Point statePt((int)estimated.at<float>(0), (int)estimated.at<float>(1));

        Point measPt((int)matMeasurement.at<float>(0, 0), (int)matMeasurement.at<float>(1, 0));

        imshow("image", image);
        image = Scalar::all(0);

        mousevec.push_back(measPt);
        kalmanvec.push_back(statePt);

        circle(image, statePt, 9, Scalar(255, 0, 255), -1); //状态向量, 用实心点表示
        circle(image, measPt, 13, Scalar(0, 255, 0), 3); //真实点, 用空心圈表示
        //std::cout << "predicted " << statePt.x << " " << statePt.y << " ";
        //std::cout << "actual " << measPt.x << " " << measPt.y << std::endl;

        //画出轨迹
        /*for (int i = 0; i < mousev.size() - 1; i++) {
            line(image, mousev[i], mousev[i + 1], cv::Scalar(255, 255, 0), 1);
        }

        for (int i = 0; i < kalmanv.size() - 1; i++) {
            line(image, kalmanv[i], kalmanv[i + 1], cv::Scalar(0, 155, 255), 1);
        }*/

        if(waitKey(10)==27)break;
    }

    return 0;
}

void mouseMoveCallback(int event, int x, int y, int flags, void* userData) {
    if (event == EVENT_MOUSEMOVE) {
        mousePos.x = x;
        mousePos.y = y;
    }
}

//一开始用的叉状点, 后来还是改成了效果更好的一个圈和一个点
void drawCross(Mat& img, Point center, Scalar color)
{
    cv::line(img, Point(center.x - 5, center.y - 5), Point(center.x + 5, center.y + 5), color, 2);
    cv::line(img, Point(center.x + 5, center.y - 5), Point(center.x - 5, center.y + 5), color, 2);
}
