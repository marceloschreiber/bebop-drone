#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>
#include <string>
//#include "yuv.h"

extern "C"{
  #include "BebopDroneDecodeStream.h"
  #include "DecoderManager.h"
  #include "ihm.h"
}
