#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>  
#include <vector>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

int Red = 160, Blue = 130;
int sat = 1;
Point2d tmp_last; //暂存上一次中心点所在
Mat prepareFinish(Mat& inImg);
void markCenter(Mat& inputImg, Mat foreImg);
Point findCenter(vector<RotatedRect>tmp_store_Rect);

int main(int argc, char* argv[]) {
    if(argc < 3)
    {
        cout << "params wrong\n";
    }

	VideoCapture capture(argv[1]);
    try	{
		boost::asio::io_service io;
		boost::asio::serial_port sp(io, argv[2]);
 
		sp.set_option(boost::asio::serial_port::baud_rate(38400)); //设置波特率
		sp.set_option(boost::asio::serial_port::flow_control());   //流量控制
		sp.set_option(boost::asio::serial_port::parity());         //奇偶校验
		sp.set_option(boost::asio::serial_port::stop_bits());      //停止位
		sp.set_option(boost::asio::serial_port::character_size(8));//有效位长度
 
		boost::asio::write(sp, boost::asio::buffer("center following", 16));

        while (1) {
            Mat frame;
            capture >> frame;
            if (frame.empty()) break;
            namedWindow("control", 0);
            namedWindow("gray", 0);

            createTrackbar("Track", "control", &Red, 10);
            createTrackbar("Saturation", "control", &sat, 10);

            string target_cor = to_string(tmp_last.x) + "," + to_string(tmp_last.y);
            boost::asio::write(sp, boost::asio::buffer(target_cor, target_cor.length()));

            prepareFinish(frame);
            if (waitKey(10) == 27) break;
        }
 
		io.run();
	}
	catch (exception& err)
	{
		cout << "Exception Error: " << err.what() << endl;
	}

	return 0;
}

//筛选出来的灯条中找到正确的中心
Point findCenter(vector<RotatedRect>tmp_store_Rect)
{
	//额...前期筛选有点问题, 会出现只发现一个灯条的情况, 但出现次数较少, 这里根据上一次所选中心进行简单校对
	if (tmp_store_Rect.size() == 1)
	{
		if (fabs(tmp_store_Rect[0].center.x - tmp_last.x) < 10.0)
		{
			return Point(tmp_last.x, tmp_last.y);
		}
		if (tmp_store_Rect[0].center.x >= tmp_last.x)
		{
			return Point(tmp_store_Rect[0].center.x - max(tmp_store_Rect[0].size.height, tmp_store_Rect[0].size.width), (tmp_store_Rect[0].center.y + tmp_last.y) / 2);
		}
		if (tmp_store_Rect[0].center.x < tmp_last.x)
		{
			return Point(tmp_store_Rect[0].center.x + max(tmp_store_Rect[0].size.height, tmp_store_Rect[0].size.width), (tmp_store_Rect[0].center.y + tmp_last.y) / 2);
		}
	}
	int i = 0;
	while (i < tmp_store_Rect.size())
	{
		int j = i + 1;
		while (j < tmp_store_Rect.size())
		{
			if (fabs(tmp_store_Rect[i].size.area() - tmp_store_Rect[j].size.area()) < 50 && fabs(tmp_store_Rect[i].angle - tmp_store_Rect[j].angle) < 5.0)
			{
				return Point((tmp_store_Rect[i].center.x + tmp_store_Rect[j].center.x) / 2, (tmp_store_Rect[i].center.y + tmp_store_Rect[j].center.y) / 2);
			}
			j++;
		}
		i++;
	}
}

//颜色检查, 图像预处理
Mat prepareFinish(Mat& src) {
	Mat gray_src;
	gray_src.create(src.size(), CV_8UC1);
	Mat splited[3];
	int a = src.channels();
	float B, G, R;
	split(src, splited);
	for (int i = 0; i < src.rows; i++)
		for (int j = 0; j < src.cols; j++) {
			B = splited[0].at<uchar>(i, j);
			G = splited[1].at<uchar>(i, j);
			R = splited[2].at<uchar>(i, j);
			float maxValue = max(B, max(G, R));
			float minValue = min(B, min(G, R));
			double S = (1 - 3.0 * minValue / (R + G + B));
			if ((R > Red && R >= G && R >= B && S > ((255 - R) * sat / Red))) gray_src.at<uchar>(i, j) = 255; //蓝色判断(B > Blue && B >= G && B >= R && S > ((255 - B) * sat / Blue))
			else gray_src.at<uchar>(i, j) = 0;
		}
	medianBlur(gray_src, gray_src, 5);
	dilate(gray_src, gray_src, Mat(5, 5, CV_8UC1)); //防止灯柱过细
	imshow("gray", gray_src);
	markCenter(src, gray_src);
	return gray_src;
}

//Draw track rectangles on graphical pictures.
void markCenter(Mat& src, Mat gray_src) {
	vector<vector<Point>> pics;
	findContours(gray_src, pics, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	RotatedRect rect; //用于暂存灯条轮廓的外接旋转矩形

	int maxlength, minlength; //保存旋转矩形长边和短边
	bool lengthDis = false; //边长筛选
	bool hw_ratioDis = false; //长宽比筛选
	vector<RotatedRect> tmp_store_Rect; //保存经过筛选后的灯条所在旋转矩形
	int i = 0;
	while (i < pics.size())
	{
		if (contourArea(pics[i]) < 500) //轮廓面积太小直接跳过
		{
			i++;
			continue;
		}
		rect = fitEllipse(pics[i]); //找出旋转矩形
		maxlength = max(rect.size.height, rect.size.width); //得到旋转矩形较长边
		minlength = min(rect.size.height, rect.size.width); //得到旋转矩形较短边

		lengthDis = maxlength > 20; //边长判断
		if (lengthDis)
		{
			hw_ratioDis = (maxlength / minlength > 1.5) && (maxlength / minlength < 20.0); //长宽比判断
			if (hw_ratioDis)
			{
				tmp_store_Rect.push_back(rect); //当作灯条
			}
		}
		i++;
	}

	cout << "\n" << tmp_store_Rect[0].center << " " << tmp_store_Rect[0].size << endl;
	if (tmp_store_Rect.size() == 2)
		cout << tmp_store_Rect[1].center << " " << tmp_store_Rect[1].size << endl;

	Point target = findCenter(tmp_store_Rect);

	tmp_last = target;

	circle(src, target, 5, Scalar(255, 0, 255), -1);
	imshow("result", src);
}
