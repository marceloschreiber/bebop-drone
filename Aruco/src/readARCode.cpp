#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	VideoCapture inputVideo(0);

	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

	while(true)
	{
		Mat image;
		inputVideo >> image;

		vector<int> ids;

		vector<vector<Point2f> > corners, rejected;

		aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);

		// if at least one marker detected
		if (ids.size() > 0){
			for(int i = 0; i < ids.size(); i++){
				if(ids[i] == 249){
					line(image, corners[i][0], corners[i][1], Scalar(0, 0, 255), 8);
					line(image, corners[i][1], corners[i][2], Scalar(0, 0, 255), 8);
					line(image, corners[i][2], corners[i][3], Scalar(0, 0, 255), 8);
					line(image, corners[i][3], corners[i][0], Scalar(0, 0, 255), 8);

					Point L = corners[i][0];
					Point M = corners[i][2];

					int centerX = L.x + (M.x - L.x)/2;
					int centerY = L.y + (M.y-L.y)/2;

					circle(image, Point(centerX, centerY), 10, Scalar(0, 255, 0), -1, 8, 0);
				}
			}
		}

		imshow("out", image);
		char c = (char)waitKey(33); // Esc key to exit
		if( c == 27 ) break;
	}
}
