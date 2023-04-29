#ifndef CONFIG_H_
#define CONFIG_H_
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
#include "IndexController.h"
#include "DetectedOtherObject.h"
#include "EventType.h"

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // Video write

#include <iostream> // std::cout, std::endl
#include <iomanip>  // std::setw
#include "json.hpp" // nlohmann

#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>

extern MonitoredArea monitoredArea;

////////////////////// Vars for JSONs sending and access to server //////////////////////////////

extern std::string cameraIdJson;
extern std::string nameJson;
extern std::string cityJson;
extern std::string streetJson;
//extern std::string directionJson;
extern std::string infoJson;
extern std::string locationTypeJson;
//extern int laneNumberJson;
extern std::string trafficAreaNames;

extern std::string signalTypeJson;

extern std::string dnnModelConfigurationPathJson;
extern std::string dnnModelWeightsPathJson;
extern std::string dnnModelClassesPathJson;

extern std::string folderForRecordedVideosOriginal;
extern std::string folderForRecordedVideosAnnotated;

//extern std::vector<std::string> unpermittedDirectionTakenJson;
extern std::vector<std::string> unpermittedLaneTakenJson;
extern std::vector<std::string> redLightViolationJson;
extern std::vector<std::string> unpermittedStoppingJson;
extern std::vector<std::string> unpermittedVehiclePassageJson;
extern std::vector<std::string> vehiclePassageJson;
extern bool obstacleInCrossingJson;

extern int warningDisplayTimeJson;

extern std::vector<std::string> trafficAreaNamesJson;

extern std::string sslAderosCertificatePathJson;
extern std::string sslCameraCertificatePathJson;
extern std::string sslCameraKeyPathJson;
extern std::string serverIpAddressJson;
extern std::string serverPortJson;

/**
* Function for load system configuration
*/
void loadSystemConfiguration();

#endif


