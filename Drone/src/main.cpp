#include "main.h"
/** the size of the drone's video stream **/
#define W 640
#define H 368

/** The maximum speed for each axis **/
#define MAX_Y_SPEED 100.0
#define MAX_Z_SPEED 20.0
#define MAX_X_SPEED 50

/** The range of distance it was sampled, anything outside this range will result in a wrong approximation **/
#define Z_CLOSE 64.0
#define Z_AWAY 325.0

using namespace cv;
using namespace std;

/* define a class that conventers the YUV image to BGR */
class ImageProcessing{
public:
	ImageProcessing(BD_MANAGER_t * deviceManager);
	void ResizeandMerge();
	Mat Frame; // contains the image in BGR
	~ImageProcessing();

private:
	// y, cb_half and cr_half are the original values from the BebopDroneDecodeStream.c file
	IplImage* y=cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 1);
	IplImage* cb_half=cvCreateImage(cvSize(W/2,H/2), IPL_DEPTH_8U, 1);
	IplImage* cr_half=cvCreateImage(cvSize(W/2,H/2), IPL_DEPTH_8U, 1);

	IplImage *bgr = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 3); // bgr image in the IplImage
};

/* Release all the temporary images from memory */
ImageProcessing::~ImageProcessing(){
	cvReleaseImage(&y);
	cvReleaseImage(&cb_half);
	cvReleaseImage(&cr_half);
	cvReleaseImage(&bgr);
}

/* Rezise the cb and cr channel to the same size as the Y and merge them */
void ImageProcessing::ResizeandMerge(){
	IplImage *ycrcb = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 3);
	IplImage *cb = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 1);
	IplImage *cr = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 1);

	cvResize(cb_half, cb, CV_INTER_CUBIC);
	cvResize(cr_half, cr, CV_INTER_CUBIC);

	cvMerge(y, cr, cb, NULL, ycrcb); // merge all channels
	cvCvtColor(ycrcb, bgr, CV_YCrCb2BGR); // convert from YCrCb to BGR

	Frame= cvarrToMat(bgr); // convert IplImage to a Mat object

	// free the memory
	cvReleaseImage(&ycrcb);
	cvReleaseImage(&cb);
	cvReleaseImage(&cr);
}

/* Constructor of the class */
ImageProcessing::ImageProcessing(BD_MANAGER_t * deviceManager){
	y->imageData = deviceManager->Y;
	cb_half->imageData = deviceManager->U;
	cr_half->imageData = deviceManager->V;

	Frame = Mat(H,W,CV_8UC3);
}

/* Return the distance of two points */
int distanceTwoPoints(Point pointA, Point pointB)
{
	return int(sqrt(pow((pointA.x - pointB.x), 2) + pow((pointA.y - pointB.y), 2)));
}

/** Calculate the size of the biggest edge of the AR code **/
float calcBiggestEdge(Point id_corner_A, Point id_corner_B, Point id_corner_C, Point id_corner_D){
	int maxDistance = distanceTwoPoints(id_corner_A, id_corner_B);
	int auxDistance = distanceTwoPoints(id_corner_B, id_corner_C);

	if(auxDistance > maxDistance){
		maxDistance = auxDistance;
		auxDistance = distanceTwoPoints(id_corner_C, id_corner_D);
	}if(auxDistance > maxDistance){
		maxDistance = auxDistance;
		auxDistance = distanceTwoPoints(id_corner_D, id_corner_A);
	}if(auxDistance > maxDistance){
		maxDistance = auxDistance;
	}
	return maxDistance;
}

/** Main function **/
int main()
{
	// define a BD_MANAGER_t, which will be responsible for several drone functions and information
	BD_MANAGER_t * deviceManager = (BD_MANAGER_t *) malloc(sizeof(BD_MANAGER_t));
	BebopDroneDecodeStreamMain(deviceManager); // old main function of the drone

	// define the parameters and dictionary for the AR code detection
	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

	Mat finalImage = imread("../media/background.jpg"); // load the background of our interface
	namedWindow("Program", 1); // name the program's window
	int sliderValue = 0; // slider to select the ID
	createTrackbar("ID", "Program", &sliderValue, 249); // create a trackbar

	Mat top(finalImage, Rect(23, 61, W, H)); // place where the stream will be displayed
	Mat bottom(finalImage, Rect(25, 497, 254, 234)); // place where the information will be displayed

	deviceManager->ihm->autonomousMode = false; // make sure that autonomous mode is disabled when the program start

	deviceManager->speedX = 50;
	deviceManager->speedY = 50;
	deviceManager->speedZ = 50;

	/** Calculate the two coeficients for the Z axis movement **/
	float aZ = (2.0*MAX_Z_SPEED)/(Z_AWAY - Z_CLOSE);
	float bZ = ((-2.0*MAX_Z_SPEED)/(Z_AWAY - Z_CLOSE)*(Z_AWAY + Z_CLOSE))/2.0;

	/** Variables used for fixing the inertia problem when going forward/backward **/
	vector<float> distances;
	float actualZspeed = 0;
	bool stabilization = 0;

	bool initialRotation = 0;

	// previous if from the old main
	if (!deviceManager->failed)
	{
		while (deviceManager->gIHMRun) // eternal loop
		{
			if(deviceManager->frameReady != NULL){ // if the information was decoded
				// create a new ImageProcessing object in order to convert the YUV to BGR
				ImageProcessing *image= new ImageProcessing(deviceManager);
				image->ResizeandMerge();

				// The AR code detection part
				vector<int> ids;
				vector<vector<Point2f> > corners, rejected;
				aruco::detectMarkers(image->Frame, dictionary, corners, ids, parameters, rejected);

				if(deviceManager->ihm->autonomousMode){ // when autonomous mode is ON track the AR code
					// if at least one marker detected
					if (ids.size() > 0){ // if it found at least one AR code
						for(int i = 0; i < ids.size(); i++){ // loop through all AR codes that were found
							if(ids[i] == sliderValue){ // found the desired AR code
								initialRotation = 1;
								// the four corners of the AR code
								Point id_corner_A = corners[i][0];
								Point id_corner_B = corners[i][1];
								Point id_corner_C = corners[i][2];
								Point id_corner_D = corners[i][3];

								// draw lines around the AR code
								line(image->Frame, id_corner_A, id_corner_B, Scalar(0, 0, 255), 5);
								line(image->Frame, id_corner_B, id_corner_C, Scalar(0, 0, 255), 5);
								line(image->Frame, id_corner_C, id_corner_D, Scalar(0, 0, 255), 5);
								line(image->Frame, id_corner_D, id_corner_A, Scalar(0, 0, 255), 5);

								// calculate the distance from the two farther corners
								float maxEdge = calcBiggestEdge(id_corner_A, id_corner_B, id_corner_C, id_corner_D);

								// calculate the center of the AR code
								float centerX = id_corner_A.x + (id_corner_C.x - id_corner_A.x)/2;
								float centerY = id_corner_A.y + (id_corner_C.y-id_corner_A.y)/2;

								// draw a circle in the displayed image
								circle(image->Frame, Point(centerX, centerY), 6, Scalar(0,0,255), -1, 8, 0);

								// Calculate the Z distance in cm using the approximation done in Matlab
								float distanceZcm = int(3.79019965878522*pow(10, -6)*pow(maxEdge,4) -0.00185037753242*pow(maxEdge, 3)
								+ 0.303554115500631*pow(maxEdge, 2) -21.2166503878915*maxEdge + 6.42418782698161*pow(10, 2));

								// set some hard limits, any value outside this range wouln't reflect the physical distance
								if(distanceZcm < Z_CLOSE)
									distanceZcm = Z_CLOSE;
								else if(distanceZcm > Z_AWAY)
									distanceZcm = Z_AWAY;

								float distanceX = centerX - W/2; // distanceX in pixels
								float distanceXcm = distanceX/maxEdge*17.78; // physical X distance

								float theta = atan2(distanceXcm, distanceZcm)*180/M_PI; // calculate the angle

								/** Stabilization when the drone has inertia **/
								distances.push_back(distanceZcm); // for every frame add the distance in a vector

								if(distances.size() >= 30){ // when this vector has more than 30 elements (1s)
									actualZspeed = (distances[29] - distances[0]); // calculate the speed
									distances.erase(distances.begin()); // delete the first element
								}

								if(stabilization){ // if the drone need to be stabilized
									if(actualZspeed >= -5 && actualZspeed <= 5){ // in this range consider the drone stabilized
										stabilization = false;
									}else{
										deviceManager->speedZ = int(0.75*actualZspeed); // use the actual speed to compensate the inertia
									}
								}else{
									if(distanceZcm <= 132 && distanceZcm >= 112) // if the drone is near the zone where it should stand still
									{
										stabilization = true; // enable the stabilization
									}
									if(distanceZcm < 122){ // if the drone is too close
										deviceManager->speedZ = int((30.0*distanceZcm - 122.0*30.0)/58.0);
									}else{ // if the drone is too far
										deviceManager->speedZ = int((20.0*distanceZcm - 122.0*20.0)/203.0);
									}
								}

								/** Calculate the speed for the Y and X axis **/
								deviceManager->speedY = int((-MAX_Y_SPEED/184.0)*centerY + MAX_Y_SPEED);
								deviceManager->speedX = int(theta*MAX_X_SPEED/37);

								deviceManager->ihm->onInputEventCallback (IHM_INPUT_EVENT_UP, deviceManager->ihm->customData);
								usleep(50);
								deviceManager->ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_RIGHT, deviceManager->ihm->customData);
								usleep(50);
								deviceManager->ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, deviceManager->ihm->customData);
								usleep(50);

								/** Convert the numbes to string **/
								std::string speedYStr = std::to_string(deviceManager->speedY);
								std::string speedZStr = std::to_string(deviceManager->speedZ);
								std::string speedXStr = std::to_string(deviceManager->speedX);
								std::string distanceStr = std::to_string(int(distanceZcm));
								std::string thetaStr = std::to_string(int(theta));

								putText(bottom, "Speed X: " + speedXStr, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
								putText(bottom, "Speed Y: " + speedYStr, Point(10, 80), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
								putText(bottom, "Speed Z: " + speedZStr, Point(10, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
								putText(bottom, "Distance: " + distanceStr + " cm", Point(10, 140), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
								putText(bottom, "Angle: " + thetaStr + " degrees", Point(10, 160), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
							}	// if right id
							else if(!initialRotation){
								deviceManager->speedX = 30;
								deviceManager->ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_RIGHT, deviceManager->ihm->customData);
								usleep(50);
							}
						} // end if for
					} // end ids > 0
					else if(!initialRotation){
						deviceManager->speedX = 30;
						deviceManager->ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_RIGHT, deviceManager->ihm->customData);
						usleep(50);
					}
				} // end of autonomousMode

				if(deviceManager->ihm->autonomousMode)
					putText(bottom, "Autonomous Mode: ON", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
				else
					putText(bottom, "Autonomous Mode: OFF", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);

				putText(bottom, "Speed X:", Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
				putText(bottom, "Speed Y:", Point(10, 80), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
				putText(bottom, "Speed Z:", Point(10, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
				putText(bottom, "Distance:", Point(10, 140), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);
				putText(bottom, "Angle:", Point(10, 160), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);

				std::string trackingStr = std::to_string(sliderValue);
				putText(bottom, "Tracking ID: " + trackingStr, Point(10, 200), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);

				image->Frame.copyTo(top);
				// show all the images
				imshow("Program", finalImage);
				bottom = Scalar(255, 255, 255);

				delete(image); // free the memory
				waitKey(1);
			} // frame ready
		} // end of the while loop
		IHM_PrintInfo(deviceManager->ihm, (char *)std::string("Disconnecting ...").c_str());
	}
	Drone_shutdown(deviceManager);
	destroyAllWindows();
	return 0;
}
