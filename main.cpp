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

  VideoCapture cap(0);
  Mat frame, edges;
  namedWindow("Webcam", 1);

  // Vidal fazendo merda
  if (!deviceManager->failed)
  {
      while (deviceManager->gIHMRun)
      {
        cap >> frame;
        imshow("Webcam", frame);
        waitKey(1);
          // if (deviceManager -> frame != NULL)
          // {
          //     FrameNum++;
          //     video.push_back(new FeatureFrame);
          //     (*video.back()).FrameNum = FrameNum;
          //     (*video.back()).Frame = tmp;
          //     (*video.back()).Frame.data = (*deviceManager).frame;
          //
          //     get_feature(video, extractor);//generate feature
          //
          //     if (video.size() != 1)
          //     {
          //         get_match(video, matcher);
          //         cv::drawMatches( (*video.back()).Frame, (*video.back()).KeyPts,(*video[video.size()-2]).Frame, (*video[video.size()-2]).KeyPts, (*video.back()).MatchPoint.MatchList, FrameOut, cv::Scalar::all(-1), cv::Scalar::all(-1), cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
          //
          //         cv::imshow("Match", FrameOut);
          //
          //         get_extrinsicMatrix(video, cameraMatrix);
          //
          //     }
          //
          //     if (video.size() >= 24)
          //     {
          //         delete (*video.begin());
          //         video.erase (video.begin());
          //     }
          //
          //     cv::waitKey(1);
          // }
      }
      IHM_PrintInfo(deviceManager->ihm, (char *)std::string("Disconnecting ...").c_str());
  }

  Drone_shutdown(deviceManager);

  return 0;
}
