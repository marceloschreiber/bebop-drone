#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

extern "C"{
  #include "BebopDroneDecodeStream.h"
  #include "DecoderManager.h"
  #include "ihm.h"
}

using namespace cv;

int main()
{
  BD_MANAGER_t * deviceManager = (BD_MANAGER_t *) malloc(sizeof(BD_MANAGER_t));

  BebopDroneDecodeStreamMain(deviceManager);

  Mat image;
  image = imread("../src/lena.png");

  namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
  imshow( "Display Image", image );

  imwrite("output.jpg", image);

  waitKey(0);
  return 0;
}
