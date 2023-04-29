#include "config.h"

string swVersion = "1.0";

////////////////////// From utilities.cpp //////////////////////////////
std::string kWinName = "AZD detection system";

// Computing testing car position for drawing
int carPositionX = 100, carPositionY = 100;
cv::Rect carPosition = cv::Rect(cv::Point(carPositionX - 75, carPositionY - 50), cv::Point(carPositionX + 75, carPositionY + 50));

// Width of network's input image
int inpWidth = 320;
// Height of network's input image
int inpHeight = 320;

// Initialize the parameters
// Confidence threshold
float confThreshold = 0.5;

// Non-maximum suppression threshold
float nmsThreshold = 0.4;

// scale factor number for text
float scaleFactor = 0.9;

int frameWidth = 0;
int frameHeight = 0;

// Circle radius for frame drawing
int circleRadiusInPixels = 12;

// Circle radius sensitivity
int circleClickRadiusOfSensitivity = 18;

int textThicknessInGUI = 2;

// Krizikova
//std::string pathToVideoFile = "W:/Krizikova/Basler/kvetenExposure20000gain25/time1.mp4"; //time18.mp4
std::string pathToVideoFile = "W:/Krizikova/Disk1/converted/20190808_095319-1.m4v";
//std::string pathToVideoFile = "W:/Krizikova/Basler/exposure20000gain25/time18.mp4"; //time18.mp4
//std::string pathToVideoFile = "W:/Krizikova/Basler2022/164795775301.mp4";

// Moravany
//std::string pathToVideoFile = "W:/Moravany/18-2-2020/Basler acA2440-35uc (23064099)_20200218_144313689.mp4"; //time18.mp4

// Kozodirky
//std::string pathToVideoFile = "W:/libanKozodirky/corrected/liban25fpsDen.mp4";
//std::string pathToVideoFile = "W:/toEvaluate/kozodirky1concat3.mp4";
//std::string pathToVideoFile = "W:/libanKozodirky/Kozodirky/grabberKozodirky1/toConcat/concat3.mp4";

// Liban
//std::string pathToVideoFile = "W:/libanKozodirky/Liban/toConcat/denConcat.mp4";
//std::string pathToVideoFile = "W:/libanKozodirky/corrected/liban25fpsDen.mp4";

// Video for detecting obstacle
// cap.set(cv::CAP_PROP_POS_MSEC, 3720000);
// cv::threshold(fgMask, fgMask, 12, 255, cv::THRESH_BINARY);
//std::string pathToVideoFile = "W:/Moravany/18-2-2020/Basler_acA2440-35uc_(23064099)_20200218_150525997.mp4"; //time18.mp4

// Video for detecting obstacle
//cap.set(cv::CAP_PROP_POS_MSEC, 3844000);
//std::string pathToVideoFile = "W:/toEvaluate/moravany.mp4"; //time18.mp4

bool isCuda = true;

string appConfigPathJson = "areaConfig.json";
string appSystemConfigName = "config.json";

string modelConfiguration = "yolov4-custom.cfg";
string modelWeights = "yolov4-custom.weights";

string modelClasses = "yolov4-custom.names";

// Warning display time
int warningDisplayTime = 50;

// Camera frame fps
int cameraFps = 13;

//Show FPS
bool showGlobalFps = false;
bool showPartialFps = false;
bool showFramesResize = false;
bool showCameraFPS = false;

////////////////////// Default values configurable with config.json //////////////////////////////
const char* targetIPaddress;
const char* targetPort;
const char* sslAderosCertificatePath;
const char* sslCameraCertificatePath;
const char* sslCameraKeyPath;

// Parameter for slow connection
bool slowConnection = false;

// Position of area for computing a average pixels value (is day or night)
// x, y, width, heigh
cv::Rect selectedAreaForDayOrNightComputing(0, 0, 100, 100);

// Allow live player
bool showFrames = true;

// Detected object TTL
int detectedObjectTTL = 3;

//int trafficLightStateDelay = cameraFps / 2;

// Parameters for detecting obstacle
bool showBgMask = false;
int initializeTimeForReferenceMatInSeconds = 60;

// Time after obstacle will be detected
int obstacleDetectingTimeInSeconds = 10;
int minObstacleAreaInPixels = 100;
int maxObstacleAreaInPixels = 10000;

// Parameters for object direction movement
int lastTrajectoryDistanceForObjectMovementInTrafficArea = 8; // in pixels
int minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea = 2; // in pixels
int trafficAreaObjectDirectionMovementTTL = 3; // max incorect iterations

// Parameter for unpermitted stopping
int objectHistoryMovingSteps = 12;
int minLastObjectMovingDistanceForStopping = 4; // in pixels
int maxStoppingTimeInSeconds = 5;
int monitoredAreaObjectMovingTTL = maxStoppingTimeInSeconds * cameraFps;

// Allowed detecting objects for analysing
std::vector<string> allowedDetectingObjects = { "car", "bus", "truck", "van", "motorbike", "bicycle" };

// Parameter for check if is day or night in frame
int frameBrightnessThresholdForTrafficLight = 70; // pixel value
int checkPartOfDayIntervalInSeconds = 3600;  // check interval in seconds

// Allow write video
bool writeVideoWithAnnotations = true;
bool writeVideoOriginal = false;

//Allow buffering and sending video with incident
bool bufferAndSendVideo = true;
//Event video duration in seconds
int eventVideoDuration = 140;

// Path to save a recorded video
std::string pathToVideos = "/mnt/nvme/";

bool birdView = false;
cv::Mat birdViewFrame;
string birdViewImagePath = "krizikova.png";

// Perspective transformated matrix
//cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(imgPoints, outputImgPoints);
cv::Mat perspectiveMatrix;

std::vector<cv::Point2f> imgPoints = { cv::Point2f(729, 972), cv::Point2f(1415, 984), cv::Point2f(1587, 1640), cv::Point2f(552, 1496) };
std::vector<cv::Point2f> outputImgPoints = { cv::Point2f(354, 420), cv::Point2f(295, 280), cv::Point2f(845, 375), cv::Point2f(820, 458) };

float distanceInMeters = 6.4;
float distanceInPixels = 94.26;
float distanceConstant = distanceInMeters / distanceInPixels;

////////////////////// Vars for JSONs sending and access to server //////////////////////////////

std::string cameraIdJson = "unkown";
std::string nameJson = "unkown";
std::string cityJson = "unkown";
std::string streetJson = "unkown";
//std::string directionJson = "unkown";
std::string infoJson = "unkown";
std::string locationTypeJson = "unkown";
//int laneNumberJson = 0;
std::vector<std::string> trafficAreaNumberToNamesJson;
//std::map<int, std::string> laneNumberToNameDictJson;

std::string signalTypeJson = "unkown";
bool barriersJson = false;

int warningDisplayTimeInSeconds = 5;
int warningDisplayTimeJson = warningDisplayTimeInSeconds * cameraFps;

std::vector<std::string> trafficAreaNamesJson;

int toleratedRedDelayJson = 10;

std::string sslAderosCertificatePathJson;
std::string sslCameraCertificatePathJson;
std::string sslCameraKeyPathJson;
std::string serverIpAddressJson;
std::string serverPortJson;

std::string dnnModelConfigurationPathJson;
std::string dnnModelWeightsPathJson;
std::string dnnModelClassesPathJson;

std::string folderForRecordedVideosOriginal;
std::string folderForRecordedVideosAnnotated;

//std::vector<std::string> unpermittedDirectionTakenJson = { "car", "bus", "truck", "van" };
std::vector<std::string> unpermittedLaneTakenJson = { "car", "bus", "truck", "van" };
std::vector<std::string> redLightViolationJson = { "car", "bus", "truck", "van" };
std::vector<std::string> unpermittedStoppingJson = { "car", "bus", "truck", "van" };
std::vector<std::string> unpermittedVehiclePassageJson = { "car", "bus", "truck", "van" };
std::vector<std::string> vehiclePassageJson = { "car", "bus", "truck", "van" };
std::vector<std::string> counterLineJson = { "car", "bus", "truck", "van" };
bool obstacleInCrossingJson = true;

void loadSystemConfiguration()
{
    // read a JSON file
    std::ifstream inputStream(appSystemConfigName);
    nlohmann::json jsonReader;

    if (inputStream.is_open())
    {
        inputStream >> jsonReader;
        inputStream.close();
        if (jsonReader.contains("cameraId"))
            cameraIdJson = jsonReader["cameraId"];
        if (jsonReader.contains("name"))
            nameJson = jsonReader["name"];
        if (jsonReader.contains("city"))
            cityJson = jsonReader["city"];
        if (jsonReader.contains("street"))
            streetJson = jsonReader["street"];
        //if (jsonReader.contains("direction"))
        //    directionJson = jsonReader["direction"];
        if (jsonReader.contains("info"))
            infoJson = jsonReader["info"];
        if (jsonReader.contains("locationType"))
            locationTypeJson = jsonReader["locationType"];
        //if (jsonReader.contains("laneNumber"))
        //    laneNumberJson = jsonReader["laneNumber"];

        if (jsonReader.contains("trafficAreaNames"))
            trafficAreaNamesJson = jsonReader["trafficAreaNames"].get<std::vector<std::string>>();

        if (jsonReader.contains("signalType"))
            signalTypeJson = jsonReader["signalType"];

        if (jsonReader.contains("barriers"))
        {
            barriersJson = jsonReader["barriers"];
        }

        if (jsonReader.contains("toleratedRedDelay"))
            toleratedRedDelayJson = jsonReader["toleratedRedDelay"];

        if (jsonReader.contains("sslAderosCertificatePath"))
        {
            sslAderosCertificatePathJson = jsonReader["sslAderosCertificatePath"];
            sslAderosCertificatePath = sslAderosCertificatePathJson.c_str();
        }

        if (jsonReader.contains("sslCameraCertificatePath"))
        {
            sslCameraCertificatePathJson = jsonReader["sslCameraCertificatePath"];
            sslCameraCertificatePath = sslCameraCertificatePathJson.c_str();
        }
        if (jsonReader.contains("sslCameraKeyPath"))
        {
            sslCameraKeyPathJson = jsonReader["sslCameraKeyPath"];
            sslCameraKeyPath = sslCameraKeyPathJson.c_str();
        }

        if (jsonReader.contains("serverIpAddress"))
        {
            serverIpAddressJson = jsonReader["serverIpAddress"];
            targetIPaddress = serverIpAddressJson.data();
        }
        if (jsonReader.contains("serverPort"))
        {
            serverPortJson = jsonReader["serverPort"];
            targetPort = serverPortJson.c_str();
        }

        if (jsonReader.contains("dnnModelConfigurationPath"))
        {
            dnnModelConfigurationPathJson = jsonReader["dnnModelConfigurationPath"];
        }

        if (jsonReader.contains("dnnModelWeightsPath"))
        {
            dnnModelWeightsPathJson = jsonReader["dnnModelWeightsPath"];
        }

        if (jsonReader.contains("dnnModelClassesPath"))
        {
            dnnModelClassesPathJson = jsonReader["dnnModelClassesPath"];
        }

        if (jsonReader.contains("appConfigPath"))
        {
            appConfigPathJson = jsonReader["appConfigPath"];
        }        

        if (jsonReader.contains("folderForRecordedVideosOriginal"))
        {            
            folderForRecordedVideosOriginal = jsonReader["folderForRecordedVideosOriginal"];
        }

        if (jsonReader.contains("folderForRecordedVideosAnnotated"))
        {
            folderForRecordedVideosAnnotated = jsonReader["folderForRecordedVideosAnnotated"];
        }

        if (jsonReader.contains("slowConnection"))
        {
            bool slowConnectionJson = jsonReader["slowConnection"];
            slowConnection = slowConnectionJson;
        }

        if (jsonReader.contains("showFrames"))
        {
            bool showFramesJson = jsonReader["showFrames"];
            showFrames = showFramesJson;
        }

        if (jsonReader.contains("writeVideoWithAnnotations"))
        {
            bool writeVideoWithAnnotationsJson = jsonReader["writeVideoWithAnnotations"];
            writeVideoWithAnnotations = writeVideoWithAnnotationsJson;
        }

        if (jsonReader.contains("writeVideoOriginal"))
        {
            bool writeVideoOriginalJson = jsonReader["writeVideoOriginal"];
            writeVideoOriginal = writeVideoOriginalJson;
        }

        if (jsonReader.contains("bufferAndSendVideo"))
        {
            bool bufferAndSendVideoJson = jsonReader["bufferAndSendVideo"];
            bufferAndSendVideo = bufferAndSendVideoJson;
        }

        if (jsonReader.contains("eventVideoDuration"))
        {
            eventVideoDuration = jsonReader["eventVideoDuration"];
            eventVideoDuration = (int)round(eventVideoDuration * cameraFps);
        }
            

        //if (jsonReader.contains("unpermittedDirectionTaken"))
        //    unpermittedDirectionTakenJson = jsonReader["unpermittedDirectionTaken"].get<std::vector<std::string>>();

        if (jsonReader.contains("unpermittedLaneTaken"))
            unpermittedLaneTakenJson = jsonReader["unpermittedLaneTaken"].get<std::vector<std::string>>();

        if (jsonReader.contains("redLightViolation"))
            redLightViolationJson = jsonReader["redLightViolation"].get<std::vector<std::string>>();

        if (jsonReader.contains("unpermittedStopping"))
            unpermittedStoppingJson = jsonReader["unpermittedStopping"].get<std::vector<std::string>>();

        if (jsonReader.contains("unpermittedVehiclePassage"))
            unpermittedVehiclePassageJson = jsonReader["unpermittedVehiclePassage"].get<std::vector<std::string>>();

        if (jsonReader.contains("vehiclePassage"))
            vehiclePassageJson = jsonReader["vehiclePassage"].get<std::vector<std::string>>();

        if (jsonReader.contains("counterLine"))
            counterLineJson = jsonReader["counterLine"].get<std::vector<std::string>>();
        
        if (jsonReader.contains("obstacleInCrossing"))
            obstacleInCrossingJson = jsonReader["obstacleInCrossing"];

        if (jsonReader.contains("warningDisplayTimeInSeconds"))
            warningDisplayTimeJson = jsonReader["warningDisplayTimeInSeconds"];

        if (jsonReader.contains("detectedObjectTTL"))
            detectedObjectTTL = jsonReader["detectedObjectTTL"];

        if (jsonReader.contains("initializeTimeForReferenceMatInSeconds"))
            initializeTimeForReferenceMatInSeconds = jsonReader["initializeTimeForReferenceMatInSeconds"];

        if (jsonReader.contains("obstacleDetectingTimeInSeconds"))
            obstacleDetectingTimeInSeconds = jsonReader["obstacleDetectingTimeInSeconds"];

        if (jsonReader.contains("minObstacleAreaInPixels"))
            minObstacleAreaInPixels = jsonReader["minObstacleAreaInPixels"];

        if (jsonReader.contains("maxObstacleAreaInPixels"))
            maxObstacleAreaInPixels = jsonReader["maxObstacleAreaInPixels"];

        if (jsonReader.contains("lastTrajectoryDistanceForObjectMovementInTrafficArea"))
            lastTrajectoryDistanceForObjectMovementInTrafficArea = jsonReader["lastTrajectoryDistanceForObjectMovementInTrafficArea"];

        if (jsonReader.contains("minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea"))
            minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea = jsonReader["minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea"];

        if (jsonReader.contains("trafficAreaObjectDirectionMovementTTL"))
            trafficAreaObjectDirectionMovementTTL = jsonReader["trafficAreaObjectDirectionMovementTTL"];

        if (jsonReader.contains("objectHistoryMovingSteps"))
            objectHistoryMovingSteps = jsonReader["objectHistoryMovingSteps"];

        if (jsonReader.contains("minLastObjectMovingDistanceForStopping"))
            minLastObjectMovingDistanceForStopping = jsonReader["minLastObjectMovingDistanceForStopping"];

        if (jsonReader.contains("monitoredAreaObjectMovingTTL"))
            monitoredAreaObjectMovingTTL = jsonReader["monitoredAreaObjectMovingTTL"];

        if (jsonReader.contains("frameBrightnessThresholdForTrafficLight"))
            frameBrightnessThresholdForTrafficLight = jsonReader["frameBrightnessThresholdForTrafficLight"];

        if (jsonReader.contains("checkPartOfDayIntervalInSeconds"))
            checkPartOfDayIntervalInSeconds = jsonReader["checkPartOfDayIntervalInSeconds"];

        monitoredArea.allowedDelayAfterRestricted = toleratedRedDelayJson;
        // Console output        
        std::cout << "APPLICATION [System configuration was successfully loaded]" << std::endl;
    }
    else {
        std::cout << "Can not open or read system configuration file: " + appSystemConfigName << std::endl;
    }
}