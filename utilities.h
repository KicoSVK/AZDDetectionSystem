#pragma comment( lib, "rpcrt4.lib" )
#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include "MonitoredArea.h"
#include "outputFrame.h"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"
#include "InferenceTimer.h"
#include "threadQueue.h"
#include "DetectedObject.h"
#include "TrafficArea.h"
#include "TrafficLight.h"
#include "IndexController.h"
#include "DetectedOtherObject.h"
#include "EventType.h"
#include "AppWindow.h"
#include "Barriers.h"
#include "config.h"
#include <string>

#include <sys/types.h>
#include <sys/stat.h>
//#include <unistd.h> // comment Kiac

# define M_PI           3.14159265358979323846  /* pi */

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // Video write

#include <iostream> // std::cout, std::endl
#include <iomanip>  // std::setw
#include "json.hpp" // nlohmann

#include <chrono>

#include <fstream>
#include <sstream>
#include <iostream>
#include "config.h"
#include <sys/stat.h>
#include "multiPlatformSleep.h"

#include "piau.h"
#include "outputFrame.h"
#include "LPReader.h"

#include "EventVideoThread.h"
#include "JsonSendThread.h"
#include <fstream>

#if __has_include("cameraGrabModule.h")    
#include <uuid/uuid.h>
#else
#include <Rpc.h>
#endif

#if 1
#define turboDebug() std::cout << __LINE__ << " " << __func__ << std::endl;
#else
#define turboDebug()
#endif

/*! \fn
 * Function run on program startup and initialize app 
 */
void onLoad(EventVideoThread& eventVideoThread, JsonSendThread& jsonSendThread);

// point structure for nlohmann json
struct Point { float x, y; };

// monitoredArea structure for nlohmann json
struct MonitoredAreaStruct { std::vector<Point> monitoredArea; std::vector<int> monitoredAreaDirection; };

// trafficArea structure for nlohmann json
struct TrafficAreaStruct { std::vector<Point> trafficArea; int trafficAreaIndex; int trafficAreaDirection; };

// init json nlohmann
using json = nlohmann::json;

void onExit(int s, bool& stopGrabFrames, cv::VideoCapture& cap, cv::VideoWriter& videoWriterWithAnnotations, cv::VideoWriter& videoWriterOriginal, cv::dnn::Net &net);

// Find right-bottommost point
cv::Point findRightBottommostPoint(std::vector<cv::Point> contours);

// Find left-bottommost point
cv::Point findLeftBottommostPoint(std::vector<cv::Point> contours);

// Get center of rectangle
cv::Point getCenterOfRect(cv::Rect  rect);

// Check count of non zero pixels in frame
bool isSomethingInArea(cv::Mat frame);

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);

// Find nearest point
//int getIndexOfNearestPointInMonitoredArea(cv::Point point);

// Find index of nearest point
int getIndexOfNearestPointInTrafficArea(cv::Point point);

int getIndexOfNearestPointInTrafficLightArea(cv::Point point);

cv::Scalar getScalarColor(int index);

// Draw the predicted bounding box
void drawPred(int index, cv::String objectName, float conf, float speed, int left, int top, int right, int bottom, cv::Mat& frame, std::vector<std::string>& classes);

// Draw object trajectory
void drawObjectTrajectory(cv::Mat& frame, DetectedObject& detectedObjectsInScene);

// Get index of traffic area
int getIndexOfTrafficArea(cv::Point point);

// Set demonstration car position
void setCarPosition(int x, int y);

// Mouse event function for demonstration car
void mouseEventForCarMoving(int event, int x, int y, int flags, void* userdata);

// Call back function for define monitored and traffic area
void mouseEvent(int event, int x, int y, int flags, void* userdata);

void postprocess(OutputFrame& outputFrame, int camera, const std::vector<cv::Mat>& outs, std::vector<std::string>& classes, float& confThreshold, float& nmsThreshold, bool& isDebugMode);

void drawCar(cv::Mat frame);

void monitoredAreaCheck(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene);

void checkUnpermittedStoppingInMonitoredArea(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene);

bool isContainsMonitoredAreaAnyDetectedObject(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene);

void trafficLightCheck(OutputFrame& outputFrame);

void loadSystemConfiguration();

void loadConfiguration();

void saveConfiguration();

void addAccessControlArea(cv::Mat& frame);

void drawAccessControlArea(cv::Mat& frame);

void addNoDetectingArea(cv::Mat& frame);

void addTrafficLightArea(cv::Mat& frame);

void addMonitoredArea(cv::Mat& frame);

void addUnpermittedVehiclePassage(cv::Mat& frame);

void drawMonitoredArea(cv::Mat& frame);

void addTrafficArea(cv::Mat& frame);

void drawAllTrafficAreas(cv::Mat& frame);

void addControlLine(cv::Mat& frame);

bool isLinesInterected(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2);

void checkAndDrawControlLine(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene, std::vector<int>& detectedIndexes);

void checkIfBusIsLocatedInFrame(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene);

void checkObjectMovementInTrafficArea(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene);

void checkObstacleInMonitoredArea(cv::Mat& frame, cv::Mat& frameForDnn, std::vector<DetectedObject>& detectedObjectsInScene);

void checkUnpermittedVehiclePassage(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene);

void drawUnpermittedVehiclePassageArea(cv::Mat& frame);

void setMonitoredAreaDirection();

void drawOtherDetectedObjects(OutputFrame& outputFrame, string object, float confidence, cv::Rect box);

//cv::VideoWriter createWindowsVideoWriter(string folderForVideoRecorder, int videoWidth, int videoHeight);
//
//cv::VideoWriter createJetsonVideoWriter(string folderForRecordedVideo, int videoWidth, int videoHeight, double nsSplit = 0);
//
//cv::VideoWriter createJetsonVideoWriter(string folderForRecordedVideo, string videoName, int videoWidth, int videoHeight, double nsSplit = 0);

void jetsonImShow(std::string kWinName, cv::Mat frame);

//inline bool fileExists(const std::string& name);

//extern bool bufferReady;

//////////////////////// Vars for JSONs sending and access to server //////////////////////////////
//
//extern string cameraIdJson;
//extern string nameJson;
//extern string cityJson;
//extern string streetJson;
////extern string directionJson;
//extern string infoJson;
//extern string locationTypeJson;
//extern int laneNumberJson;
//extern string trafficAreaNames;
//
//extern string signalTypeJson;
//
//extern string dnnModelConfigurationPathJson;
//extern string dnnModelWeightsPathJson;
//extern string dnnModelClassesJson;
//extern int warningDisplayTimeJson;
//
//extern std::vector<std::string> trafficAreaNamesJson;
//
//extern string sslAderosCertificatePathJson;
//extern string sslCameraCertificatePathJson;
//extern string sslCameraKeyPathJson;
//extern string serverIpAddressJson;
//extern string serverPortJson;
////extern bool slowConnectionJson;

extern string pathToVideos;

//### Parameter from config.cpp
extern std::string kWinName;
extern int carPositionX, carPositionY;
extern cv::Rect carPosition;

extern const char* targetIPaddress;
extern const char* targetPort;
extern const char* sslAderosCertificatePath;
extern const char* sslCameraCertificatePath;
extern const char* sslCameraKeyPath;

extern std::string modelConfiguration;

extern float distanceConstant;

extern cv::Mat birdViewFrame;
extern cv::Mat perspectiveMatrix;
extern string birdViewImagePath;
extern string appConfigPathJson;
extern string appSystemConfigName;

extern std::vector<cv::Point2f> imgPoints;
extern std::vector<cv::Point2f> outputImgPoints;

enum PartsOfDay { day, night};

extern int obstacleDetectingTimeInSeconds;

extern int frameBrightnessThresholdForTrafficLight;

extern int initializeTimeForReferenceMatInSeconds;

extern int checkPartOfDayIntervalInSeconds;

//extern int lastTrajectoryDistanceForObjectMovementInTrafficArea;
//extern int minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea;

// Width of network's input image
extern int inpWidth;
// Height of network's input image
extern int inpHeight;

extern bool isCarDisplayed;
extern bool isDisplayedMonitoredAreaScenario;
extern int indexOfActivePointInMonitoringArea;
extern bool isDisplayedTrafficLightAreaScenario;
//bool isSelectingMonitoredArea = false;
//extern bool isSelectingTrafficLightArea;
extern bool isDisplayedControlLineScenario;

extern bool isDisplayedTrafficAreaScenario;
extern std::vector<cv::Point> trafficLightArea;
// vector for traffic
//extern std::vector<std::vector<cv::Point>> trafficArea;
extern std::vector<TrafficArea> trafficArea;
// index of active/current traffic line
extern int indexOfActiveTrafficArea;
// current index of point in traffic area
extern int indexOfActivePointInTrafficArea;
// Monitored area
extern MonitoredArea monitoredArea;

extern vector<cv::Point> noDetectingArea;
extern bool isDisplayedNoDetectingAreaScenario;

// Access control area
extern std::vector<cv::Point> unpermittedVehiclePassageArea;
extern bool isDisplayedUnpermittedVehiclePassageScenario;
//extern bool isSelectingAccessControlArea;

extern bool isClickedMonitoredArea;

extern bool isNormalState;

// Initialize the parameters
// Confidence threshold
extern float confThreshold;
// Non-maximum suppression threshold
extern float nmsThreshold;

extern bool isDisplayedMainMenu;
extern bool isPlaying;

extern int circleRadiusInPixels;

extern int circleClickRadiusOfSensitivity;

extern cv::Mat fgMask;

extern int frameCounter;
//extern int frameCounterDelay;
extern cv::Mat referenceMat;
extern std::vector<DetectedOtherObject> detectedOtherObjectInMonitoredArea;
extern bool isDetectedObstacleInMonitoredArea;
extern cv::Rect obstacleInMonitoredArea;

extern int obstacleDurationTimeFrameCounting;

// Controller for detected object indexes
extern IndexController indexController;

// Count of frame, for frame brightness evalue
extern int frameBrightnessCounter;

// Event type handler
extern EventType* evenTypeHandler;

extern std::chrono::high_resolution_clock::time_point lastFrameShowedTime;
extern std::chrono::high_resolution_clock::time_point actualFrameShowedTime;

extern bool stopGrabFrames;
extern cv::VideoCapture cap;
extern cv::VideoWriter videoWriterWithAnnotations;
extern cv::VideoWriter videoWriterOriginal;

extern std::thread t1;
extern std::thread statusSendingThread;
extern std::thread statisticSendingThread;

extern std::thread netForwardThread;
extern std::thread postProcessThread;
extern std::thread drawQueueThread;

// Neural network classes
extern std::vector<std::string> classes;

// Vector for all detected objects
////////////////////////////extern std::vector<DetectedObject*> detectedObjectsInScene;

extern int minObstacleAreaInPixels;
extern int maxObstacleAreaInPixels;

extern std::vector<string> allowedDetectingObjects;

extern cv::Mat newFrame;

extern int cameraFps;

extern std::thread th1;

extern int frameWidth;
extern int frameHeight;

extern bool birdView;

extern int warningDisplayTime;

//extern std::vector<std::vector<uchar>> bufferedImages;
extern std::vector<cv::Mat> bufferedImages;

extern cv::dnn::Net net;

extern float scaleFactor;

extern std::vector<std::string> counterLineJson;

// Function chceck if file is exists in project directiory
bool isFileExist(std::string& name);

void onExitSignal(int s);

//void bufferImage(cv::Mat frame);

//std::vector<std::vector<uchar>> getBufferedImages();
//std::vector<cv::Mat> getBufferedImages();

//void writeEventVideo();

//void writeEventVideo(string videoName);

///////statistics
#if __has_include("cameraGrabModule.h")

nlohmann::json getStatisticJson();

void statisticSending();
std::thread startStatisticSending();
#else
void statisticSending();
std::thread startStatisticSending();
#endif

std::string generateGUID();

std::string getContentPath();

void createFolder(std::string folderName);

cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);

cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);

#endif