#include<opencv2\opencv.hpp>
#include<iostream>
#include<time.h>
using namespace std;
using namespace cv;

Mat img_Canny(Mat img);
Mat img_hough(Mat img);
Mat La(Mat img);
Vec2f cornerDetect(Mat img);

int main() {
	
	vector<Point2f>corners;
	Mat img = imread("D:\\code\\python\\cv2practice\\4.jpg");
	//imshow("first", img);
	
	img = img_Canny(img);
	imshow("canny", img);
	img = img_hough(img);
	goodFeaturesToTrack(img, corners, 4, 0.01, 200);
	for (int i = 0; i < 4; i++) {
		circle(img, corners[i],20, Scalar(255, 255, 255), 2, 8, 0);
	}
	imshow("done", img);
	
	waitKey(0);
	destroyAllWindows();
	return 0;
}

Mat img_Canny(Mat img) {
	
	cvtColor(img, img, COLOR_RGB2GRAY);
	//Laplacian(img, img, CV_16S, 3);
	Canny(img, img, 100, 200, 3);
	return img;
}

Mat img_hough(Mat img) {
	Mat src1(400, 400, CV_16UC3);
	Mat src2(400, 400, CV_16UC3);
	vector<Vec4f>plines;
	vector<Vec2f>lines;
	//HoughLinesP(img, plines, 50, CV_PI / 180, 150);
	HoughLines(img, lines, 1, CV_PI / 180, 100);
	//for (int i = 0; i < plines.size(); i++) {
	//	Vec4f point1 = plines[i];
	//	//cout << point1 << endl;
	//	line(src1, Point(point1[0], point1[1]), Point(point1[2], point1[3]), Scalar(255, 0, 0), 3);
	//}
	for (int j = 0; j < lines.size(); j++) {
		float rho = lines[j][0], theta = lines[j][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(img, pt1, pt2, Scalar(255, 255, 0), 1, LINE_AA);
	}
	//line(img, Point(1, 1), Point(250, 250), Scalar(255, 0, 0), 5);
	//imshow("hough1", src1);
	imshow("hough2", img);
	return img;
}

//Vec2f cornerDetect(Mat img) {
//
//
//
//}