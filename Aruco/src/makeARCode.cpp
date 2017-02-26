#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;

int main(int argc, char *argv[])
{
  int borderBits = 1;
  int markerSize = 680;
  int markerId = 8;

  Ptr<aruco::Dictionary> dictionary =
  aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

  Mat markerImg;
  aruco::drawMarker(dictionary, markerId, markerSize, markerImg, borderBits);

  imshow("marker", markerImg);

  imwrite("ar5.jpg", markerImg);

  waitKey(0);
  return 0;
}
