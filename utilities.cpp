#include "utilities.h"

// vector for store monitoredArea points for json
//std::vector<Point> monitoredAreaJson;
//MonitoredAreaStruct monitoredAreaJson;

// vector for store monitoredArea points for json
//kiac std::vector<int> monitoredAreaDirectionJson;

// variables for Json loading/saving
// vector for store traffic light area points for json
std::vector<Point> trafficLightAreaJson;

// vector for store no detecting area points for json
std::vector<Point> noDetectingAreaJson;

// vector for store unpermitted vehicle area points for json
std::vector<Point> unpermittedVehiclePassageAreaJson;

// vector for store control line points for json
std::vector<Point> controlLineJson;

// control line detected object indexes
// kiac kiac OPRAVENE
// std::vector<int> detectedIndexes;

// vector for store traffic area points for json
std::vector<TrafficAreaStruct> trafficAreaJson;

// kiac kiac nebude problem
TrafficLight trafficLight;
//extern int trafficLightStateDelay;
int trafficLightStateDelay = cameraFps / 2;
// kiac kiac nebude problem
Barriers barriers;

bool isDebugMode = false;

bool isCarDisplayed = false;

bool isDisplayedMonitoredAreaScenario = false;

int indexOfActivePointInMonitoringArea = -1;

int indexOfLastActivePointInMonitoredArea = -1;

bool isDisplayedTrafficAreaScenario = false;

// Monitored area
MonitoredArea monitoredArea;

std::vector<cv::Point> trafficLightArea;

bool isDisplayedTrafficLightAreaScenario = false;

int indexOfActivePointInTrafficLightArea = -1;

int indexOfLastActivePointInTrafficLightArea = -1;

// vector for traffic
std::vector<TrafficArea> trafficArea;

// index of active/current traffic line
int indexOfActiveTrafficArea = -1;

// current index of point in traffic area
int indexOfActivePointInTrafficArea = -1;

int indexOfLastActivePointInTrafficArea = -1;

std::vector<cv::Point> unpermittedVehiclePassageArea;

bool isNormalStateInUnpermittedVehiclePassageArea = true;

int indexOfActivePointInUnpermittedVehiclePassageArea = -1;

int indexOfLastActivePointInUnpermittedVehiclePassageArea = -1;

// Detecting area
vector<cv::Point> noDetectingArea;

bool isDisplayedNoDetectingAreaScenario = false;

int indexOfActivePointInDetectingArea = -1;

int indexOfLastActivePointInDetectingArea = -1;

// ControlLine display
bool isDisplayedControlLineScenario = false;

int indexOfActivePointInControlLine = -1;

int indexOfLastActivePointInControlLine = -1;

// control line, for vehicle counting
std::vector<cv::Point> controlLine;

// Access control area
bool isDisplayedUnpermittedVehiclePassageScenario = false;

bool isSelectingAccessControlArea = false;

bool isClickedMonitoredArea = false;

bool isNormalState = true;

bool isDisplayedMainMenu = false;

bool isPlaying = true;

bool isDrivingInOppositeSide = false;

bool isUnpermittedStopping = false;

bool isUnpermittedVehiclePassage = false;

bool isVehiclePassage = false;

// iterator for warnings
int iteratorForResetDrivingInOppositeSideWarning = 0;
int iteratorForResetDetectedObstacleInMonitoredAreaWarning = 0;
int iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning = 0;
int iteratorForResetNormalStateWarning = 0;
int iteratorForResetUnpermittedStopping = 0;
int iteratorForResetUnpermittedVehiclePassage = 0;
int iteratorForResetVehiclePassage = 0;

// app window
AppWindow appWindow;

cv::Mat fgMask;

int frameCounter = -cameraFps * 1;

cv::Mat referenceMat;

// kiac kiac, opravene, pouziva iba draw
std::vector<DetectedOtherObject> detectedOtherObjectInMonitoredArea;

bool isDetectedObstacleInMonitoredArea = false;

// kiac kiac iba draw
cv::Rect obstacleInMonitoredArea;
//int obstacleDurationTimeFrameCounting = 0;

// kiac kiac, pouziva len postprocess
// Controller for detected object indexes
IndexController indexController;

int frameBrightnessCounter = -1;

PartsOfDay partOfDay = PartsOfDay::day;
int frameBrightness = -1;

// Event type handler
EventType* evenTypeHandler;

bool stopGrabFrames = false;

cv::VideoCapture cap;
cv::VideoWriter videoWriterWithAnnotations;
cv::VideoWriter videoWriterOriginal;

std::chrono::high_resolution_clock::time_point lastFrameShowedTime;
std::chrono::high_resolution_clock::time_point actualFrameShowedTime;

std::thread t1;
std::thread statusSendingThread;
std::thread statisticSendingThread;

std::thread netForwardThread;
std::thread postProcessThread;
std::thread drawQueueThread;

std::thread videoEncodeThread;

int frameCounterForCheckObstacle = -1;

// Position of area for computing a average pixels value (is day or night)
// x, y, width, heigh
extern cv::Rect selectedAreaForDayOrNightComputing;

// Neural network classes
std::vector<std::string> classes;

// Vector for all detected objects
//std::vector<DetectedObject*> detectedObjectsInScene;

std::thread th1;

cv::Mat newFrame;

extern bool slowConnection;

extern bool isCuda;

extern int eventVideoDuration;

//extern std::vector<string> trafficAreaNames;

//std::vector<std::vector<uchar>> bufferedImages;
//std::vector<cv::Mat> bufferedImages;

cv::dnn::Net net;

std::vector<float> postProcessTime;

//bool bufferReady = false;

//control line counted objects [car, bus, truck, van]
//int controlLineNumOfObjects[] = {0, 0, 0, 0};
std::map<std::string, int> counterLineNumOfObjects{ {"car", 0}, {"bus", 0}, {"truck", 0}, {"van", 0} };

extern Piau piau;
//extern LPReader lPReader;

// Allow live player
extern bool showFrames;

extern bool showBgMask;

// Allow write video
extern bool writeVideoWithAnnotations;
extern bool writeVideoOriginal;

//Allow buffering and sending video with incident
extern bool bufferAndSendVideo;
//Event video duration in seconds
extern int eventVideoDuration;

const int alpha_slider_max = 360;
int alpha_slider = 0;
//double alpha;

static void on_trackbar(int, void*) {}

void onLoad(EventVideoThread& eventVideoThread, JsonSendThread& jsonSendThread)
{
    if(birdView)
    {
        //birdViewFrame = cv::imread("krizikova.png");
        birdViewFrame = cv::imread(birdViewImagePath);
        perspectiveMatrix = cv::getPerspectiveTransform(imgPoints, outputImgPoints);
    }

    loadSystemConfiguration();
    loadConfiguration();
    //std::cout << "unpermittedVehiclePassageJson: " << unpermittedVehiclePassageJson[0] << unpermittedVehiclePassageJson[1] << std::endl;
    //std::cout << "vehiclePassage: " << vehiclePassageJson[0] << std::endl;
    //std::cout << "obstacleInCrossing: " << obstacleInCrossingJson << std::endl;

    //std::cout << "targetIPaddress: " << targetIPaddress << std::endl;
    createFolder(pathToVideos + cameraIdJson);
    //eventVideoThread.cameraFps = 10;
    //std::cout << "eventVideoThread.cameraFps: " << eventVideoThread.cameraFps << std::endl;
    //EventType* newEvenTypeHandler;
    //JsonSendThread 
    //JsonSendThread* newJsonSendThread = &JsonSendThread(targetIPaddress, targetPort, sslAderosCertificatePath, sslCameraCertificatePath, sslCameraKeyPath, &stopGrabFrames);
    //*jsonSendThread = newJsonSendThread;
    //std::cout << "jsonSendThreadOnload: " << (*jsonSendThread)->stopGrabFrames << std::endl;

    jsonSendThread.setParams(targetIPaddress, targetPort, sslAderosCertificatePath, sslCameraCertificatePath, sslCameraKeyPath, &stopGrabFrames);
    
    evenTypeHandler = new EventType(trafficAreaNamesJson, eventVideoDuration, &eventVideoThread, &jsonSendThread);
    //jsonSendThread.objectNames = evenTypeHandler->getObjectNames();
    
    
    //std::cout << "evenTypeHandler.eventVideoThread.cameraFps: " << newEvenTypeHandler->eventVideoThread->cameraFps << std::endl;
    //std::cout << "evenTypeHandler.ipAddress: " << newEvenTypeHandler->ipAddress << std::endl;
    //evenTypeHandler = EventType(eventVideoThread);
    //evenTypeHandler.ipAddress = targetIPaddress;
    //evenTypeHandler.port = targetPort; //!< server port
    //evenTypeHandler.ca_pem = sslAderosCertificatePath; //!< SSL CA
    //evenTypeHandler.cert_pem = sslCameraCertificatePath; //!< SSL certificate
    //evenTypeHandler.key_pem = sslCameraKeyPath; //!< SSL key

    //evenTypeHandler.trafficAreaNames = trafficAreaNamesJson;  //!< All traffic area names
    //evenTypeHandler.eventVideoDuration = eventVideoDuration; //!< Time for lenght of event video

    //evenTypeHandler.eventVideoThread;
    //std::cout << "evenTypeHandler.targetIPaddress: " << evenTypeHandler.ipAddress << std::endl;
    piau = Piau();
    //lPReader = LPReader();

}

// nlohmann json reference to point structure
void to_json(nlohmann::json& j, const Point& p) {
    j = { {"x", p.x}, {"y", p.y} };
}

// nlohmann json reference to traffic area structure
void to_json(nlohmann::json& j, const TrafficAreaStruct& area) {
    j = { {"area", area.trafficArea}, {"index", area.trafficAreaIndex}, {"direction", area.trafficAreaDirection} };
}

// nlohmann json reference to monitored area structure
void to_json(nlohmann::json& j, const MonitoredAreaStruct& area) {
    j = { {"area", area.monitoredArea}, {"direction", area.monitoredAreaDirection} };
}

void onExit(int s, bool& stopGrabFrames, cv::VideoCapture& cap, cv::VideoWriter& videoWriterWithAnnotations, cv::VideoWriter& videoWriterOriginal, cv::dnn::Net& net) {
    stopGrabFrames = true;
#ifdef LINUX
    videoWriterWithAnnotations.release();
    videoWriterOriginal.release();
    printf("Caught signal %d\n", s);
#endif   
    videoWriterWithAnnotations.release();
    videoWriterOriginal.release();
    cap.release();
    //exit(s);
    //net = cv::dnn::Net();  
}

cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad)
{
    cv::Point2f outPoint;
    //CW rotation
    outPoint.x = std::cos(angRad) * inPoint.x - std::sin(angRad) * inPoint.y;
    outPoint.y = std::sin(angRad) * inPoint.x + std::cos(angRad) * inPoint.y;
    return outPoint;
}

cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad)
{
    return rotate2d(inPoint - center, angRad) + center;
}

// Find right-bottommost point
cv::Point findRightBottommostPoint(std::vector<cv::Point> contours) {

    cv::Point rightmostPoint = *max_element(contours.begin(), contours.end(),
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.x < rhs.x;
        });

    cv::Point bottommostPoint = *max_element(contours.begin(), contours.end(),
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.y < rhs.y;
        });

    cv::Point tempPoint;
    tempPoint.x = rightmostPoint.x;
    tempPoint.y = bottommostPoint.y;

    return tempPoint;
}

// Find left-bottommost point
cv::Point findLeftBottommostPoint(std::vector<cv::Point> contours) {

    cv::Point leftmostPoint = *max_element(contours.begin(), contours.end(),
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.x > rhs.x;
        });

    cv::Point bottommostPoint = *max_element(contours.begin(), contours.end(),
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.y < rhs.y;
        });

    cv::Point tempPoint;
    tempPoint.x = leftmostPoint.x;
    tempPoint.y = bottommostPoint.y;

    return tempPoint;
}

// Get center of rectangle
cv::Point getCenterOfRect(cv::Rect  rect) {
    return cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

// Check count of non zero pixels in frame
bool isSomethingInArea(cv::Mat frame) {
    if (countNonZero(frame) >= ((frame.rows * frame.cols) * 1 / 6))
        return true;
    return false;
}

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

// Find index of nearest point
int getIndexOfNearestPointInTrafficArea(cv::Point point) {
    if (trafficArea[indexOfActiveTrafficArea].getTrafficArea().size() < 100) {
        for (size_t i = 0; i < trafficArea[indexOfActiveTrafficArea].getTrafficArea().size(); i++)
        {
            if (trafficArea[indexOfActiveTrafficArea].getTrafficArea().at(i).x - circleClickRadiusOfSensitivity < point.x && point.x < trafficArea[indexOfActiveTrafficArea].getTrafficArea().at(i).x + circleClickRadiusOfSensitivity && trafficArea[indexOfActiveTrafficArea].getTrafficArea().at(i).y - circleClickRadiusOfSensitivity < point.y && point.y < trafficArea[indexOfActiveTrafficArea].getTrafficArea().at(i).y + circleClickRadiusOfSensitivity) {
                return i;
            }
        }
        return -1;
    }
    return -1;
}

int getIndexOfNearestPointInControlLine(cv::Point point) {
    for (int i = 0; i < controlLine.size(); i++)
    {
        if (controlLine.at(i).x - circleClickRadiusOfSensitivity < point.x && point.x < controlLine.at(i).x + circleClickRadiusOfSensitivity && controlLine.at(i).y - circleClickRadiusOfSensitivity < point.y && point.y < controlLine.at(i).y + circleClickRadiusOfSensitivity) {
            return i;
        }
    }
    return -1;
}

// Find index of nearest point
int getIndexOfNearestPointInTrafficLightArea(cv::Point point) {
    for (int i = 0; i < trafficLightArea.size(); i++)
    {
        if (trafficLightArea.at(i).x - circleClickRadiusOfSensitivity < point.x && point.x < trafficLightArea.at(i).x + circleClickRadiusOfSensitivity && trafficLightArea.at(i).y - circleClickRadiusOfSensitivity < point.y && point.y < trafficLightArea.at(i).y + circleClickRadiusOfSensitivity) {
            return i;
        }
    }
    return -1;
}

int getIndexOfNearestPointInNoDetectingArea(cv::Point point) {
    for (int i = 0; i < noDetectingArea.size(); i++)
    {
        if (noDetectingArea.at(i).x - circleClickRadiusOfSensitivity < point.x && point.x < noDetectingArea.at(i).x + circleClickRadiusOfSensitivity && noDetectingArea.at(i).y - circleClickRadiusOfSensitivity < point.y && point.y < noDetectingArea.at(i).y + circleClickRadiusOfSensitivity) {
            return i;
        }
    }
    return -1;
}

int getIndexOfNearestPointInUnpermittedVehiclePassage(cv::Point point) {
    for (int i = 0; i < unpermittedVehiclePassageArea.size(); i++)
    {
        if (unpermittedVehiclePassageArea.at(i).x - circleClickRadiusOfSensitivity < point.x && point.x < unpermittedVehiclePassageArea.at(i).x + circleClickRadiusOfSensitivity && unpermittedVehiclePassageArea.at(i).y - circleClickRadiusOfSensitivity < point.y && point.y < unpermittedVehiclePassageArea.at(i).y + circleClickRadiusOfSensitivity) {
            return i;
        }
    }
    return -1;
}

// Get available scalar color
cv::Scalar getScalarColor(int index) {
    switch (index)
    {
    case 0:
        return cv::Scalar(255, 75, 0);
        break;
    case 1:
        return cv::Scalar(150, 200, 50);
        break;
    case 2:
        return cv::Scalar(255, 255, 0);      
        break;
    case 3:
        return cv::Scalar(0, 255, 255);
        break;
    case 4:
        return cv::Scalar(255, 0, 255);
        break;
    case 5:
        return cv::Scalar(255, 125, 155);
        break;
    case 6:
        return cv::Scalar(0, 0, 100);
        break;
    case 7:
        return cv::Scalar(0, 100, 0);
        break;
    case 8:
        return cv::Scalar(100, 0, 0);
        break;
    default:
        return cv::Scalar(255, 255, 255);
        break;
    }
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(OutputFrame& outputFrame, int cameraFps, const std::vector<cv::Mat>& outs, std::vector<std::string>& classes, float& confThreshold, float& nmsThreshold, bool& isDebugMode)
{
    cv::Mat& frame = outputFrame.frameForDisplay;
    //std::vector<DetectedObject>& detectedObjectsInScene = outputFrame.detectedObjectsInScene;

    // Scale factor for font drawing
    scaleFactor = frame.cols / 2048.0;

    //std::cout << frame.rows << "/" << frame.cols << std::endl;
    auto start = chrono::steady_clock::now();

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<int> LPIds;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));

                if (classes[classIdPoint.x] == "licensePlate")
                {
                    LPIds.push_back(classIds.size() - 1);
                    //cv::Rect actualBBox = boxes[classIds.size() - 1];
                    //cv::Point highResTl = cv::Point(int(round((float(actualBBox.tl().x) / frame.rows) * outputFrame.frameOriginal.rows)), int(round((float(actualBBox.tl().y) / frame.cols) * outputFrame.frameOriginal.cols)));
                    //int highResWidth = int(round((float(actualBBox.width) / frame.rows) * outputFrame.frameOriginal.rows));
                    //int highResHeight = int(round((float(actualBBox.height) / frame.cols) * outputFrame.frameOriginal.cols));


                    //std::cout << "highResTlx: " << highResTl.x << "highResTly: " << highResTl.y << std::endl;
                    //std::cout << "highResWidth: " << highResWidth << "highResHeight: " << highResHeight << std::endl;
                    //cv::imshow("LP", outputFrame.frameOriginal(cv::Rect(highResTl.x, highResTl.y, highResWidth, highResHeight)));
                }                    
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Apply Kalman prediction
    if (outputFrame.detectedObjectsInScene.size() > 0) {
        for (size_t i = 0; i < outputFrame.detectedObjectsInScene.size(); i++)
        {
            outputFrame.detectedObjectsInScene.at(i).applyKalmanPrediction();
            //cv::circle(frame, outputFrame.detectedObjectsInScene.at(i)->getObjectPredRectCenter(), 12, cv::Scalar(0, 0, 255), 6);
        }
    }

    if (outputFrame.detectedObjectsInScene.size() != 0) {
        for (size_t i = 0; i < outputFrame.detectedObjectsInScene.size(); i++)
        {
            // decrement TTL in all old detected objects
            outputFrame.detectedObjectsInScene.at(i).decreaseTtl();

            // chceck TTL in all old detected objects
            if (outputFrame.detectedObjectsInScene.at(i).getTtl() <= 0) {

                if (outputFrame.detectedIndexes.size() > 0) {
                    if (std::count(outputFrame.detectedIndexes.begin(), outputFrame.detectedIndexes.end(), outputFrame.detectedObjectsInScene.at(i).getIndexOfObject())) {
                        outputFrame.detectedIndexes.erase(std::remove(outputFrame.detectedIndexes.begin(), outputFrame.detectedIndexes.end(), outputFrame.detectedObjectsInScene.at(i).getIndexOfObject()), outputFrame.detectedIndexes.end());
                    }
                }
                // Remove object from memory and in IndexController
                int index = outputFrame.detectedObjectsInScene.at(i).getIndexOfObject();
                evenTypeHandler->removeObject(index);
                //delete detectedObjectsInScene.at(i);
                outputFrame.detectedObjectsInScene.erase(outputFrame.detectedObjectsInScene.begin() + i);
                indexController.removeIndex(index);
            }
            // check if detected object is close to border of scene, if detected object is close to border of frame, so detected object will be removed
            else {
                float pointsDistance = cv::max(abs(outputFrame.detectedObjectsInScene.at(i).getObjectRoute()[0].x - outputFrame.detectedObjectsInScene.at(i).getObjectRoute()[outputFrame.detectedObjectsInScene.at(i).getObjectRouteSize() - 1].x), abs(outputFrame.detectedObjectsInScene.at(i).getObjectRoute()[0].y - outputFrame.detectedObjectsInScene.at(i).getObjectRoute()[outputFrame.detectedObjectsInScene.at(i).getObjectRouteSize() - 1].y));

                // chceck if detected object is moving in scene
                if (pointsDistance <= 6)
                {
                    if (outputFrame.detectedObjectsInScene.at(i).getObjectRect().x < 20 || (outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width) >(frame.cols - 20) || outputFrame.detectedObjectsInScene.at(i).getObjectRect().y < 20 || (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height) >(frame.rows - 20)) {
                        outputFrame.detectedObjectsInScene.at(i).setTtl(outputFrame.detectedObjectsInScene.at(i).getTtl() - 10);
                    }
                }                    
            }
        }
    }

    // Detected object handler, if previously detected objects is not NULL
    if (outputFrame.detectedObjectsInScene.size() != 0) {
        //std::cout << to_string(detectedObjectsInScene.size()) << std::endl;
        // indices store all new detected object in frame
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            cv::Rect box = boxes[idx];

            // one of new detected object
            cv::Point2f centerOfNewObjectRect = (box.br() + box.tl()) * 0.5;

            // Improve more sensitive detection with bigger origin box
            //cv::Rect biggerBox = box;
            float regionOfSensitivity = 0.45;
            cv::Rect2f biggerBox = cv::Rect(cv::Point(centerOfNewObjectRect.x - ((float)(box.width * regionOfSensitivity)), centerOfNewObjectRect.y - ((float)(box.height * regionOfSensitivity))), cv::Point(centerOfNewObjectRect.x + ((float)(box.width * regionOfSensitivity)), centerOfNewObjectRect.y + ((float)(box.height * regionOfSensitivity))));
            //cv::Rect biggerBox = cv::Rect(cv::Point(box.x - (box.width * 1 / 5), box.y - (box.height * 1 / 5)), cv::Point((box.x + box.width) + (box.width * 1 / 5), (box.y + box.height) + (box.height * 1 / 5)));

            int indexOfNearestPoint = 0;
            int lenghtBettwenTwoPoints = 0;

            int nearestRectIndex = -1;
            int nearestRectDistance = INT16_MAX;
            //int nearestRectDistance = 80;

            // Level crossing, barriers detecting ... Pavel prida do json, ci sa na priecesti realne vyskytuju bariery
            // Ak ano detekujeme ho ... 
            if (locationTypeJson == "roadCrossing" && (classes[classIds[idx]] == "barrierOpen" || classes[classIds[idx]] == "barrierClosed")) {
                if (classes[classIds[idx]] == "barrierOpen") {
                    barriers.setBarriersState(Barriers::BarriersState::open);
                }
                else {
                    barriers.setBarriersState(Barriers::BarriersState::down);
                }
            }
            else {
                barriers.setBarriersState(Barriers::BarriersState::none);
            }

            //if (classes[classIds[idx]] == "car" || classes[classIds[idx]] == "bus" || classes[classIds[idx]] == "truck" || classes[classIds[idx]] == "traffic light") {
            //if (classes[classIds[idx]] == "car" || classes[classIds[idx]] == "bus" || classes[classIds[idx]] == "truck" || classes[classIds[idx]] == "van" || classes[classIds[idx]] == "motorbike" || classes[classIds[idx]] == "bicycle") {
            if (std::find(allowedDetectingObjects.begin(), allowedDetectingObjects.end(), classes[classIds[idx]]) != allowedDetectingObjects.end()){

                // one current object (predicted rect) from all detected objects yet
                cv::Point centerOfCurrectObjectPredictedRect;

                // Loop go through every old detected objects
                for (size_t j = 0; j < outputFrame.detectedObjectsInScene.size(); j++)
                {
                    centerOfCurrectObjectPredictedRect = (outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().br() + outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().tl()) * 0.5;

                    if (true) {
                        // Draw new detected object rectangle (bigger rectangle)
                        if (biggerBox.x >= 0 && biggerBox.x <= frame.cols
                            && biggerBox.y >= 0 && biggerBox.y <= frame.rows) {
                            if ((biggerBox.x + biggerBox.width) <= frame.cols
                                && (biggerBox.y + biggerBox.height) <= frame.rows) {
                                //cv::rectangle(frame, biggerBox, CV_RGB(0, 255, 255), 1.5);
                            }
                        }

                        if (outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().x > 0) {

                            //cv::Rect tempRect = outputFrame.detectedObjectsInScene.at(j)->getObjectPredRect();

                            if (outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().x >= 0 && outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().x <= frame.cols
                                && outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().y >= 0 && outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().y <= frame.rows) {
                                if ((outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().x + outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().width) <= frame.cols
                                    && (outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().y + outputFrame.detectedObjectsInScene.at(j).getObjectPredRect().height) <= frame.rows) {
                                    //cv::rectangle(frame, detectedObjectsInScene.at(j)->getObjectPredRect(), CV_RGB(0, 0, 255), 1);
                                    //cv::drawMarker(frame, centerOfCurrectObjectPredictedRect, cv::Scalar(0, 0, 255), 0, 10, 5);
                                }
                            }
                        }                       
                        //cv::circle(frame, centerOfCurrectObjectPredictedRect, 12, cv::Scalar(0, 0, 255), 6);
                    }

                    // Check if biggerBox (new detected object) contains current object (current object from all old detected objects)
                    //if (box.contains(centerOfObjectRect) && classes[classIds[idx]] == detectedObjectsInScene.at(j)->getObjectName() || classes[classIds[idx]] == detectedObjectsInScene.at(j)->getObjectName() || classes[classIds[idx]] == detectedObjectsInScene.at(j)->getObjectName()) {
                    if (biggerBox.contains(centerOfCurrectObjectPredictedRect)) {
                        //cv::circle(frame, centerOfObjectRect, 5, cv::Scalar(0, 0, 255), 4);
                        //int distance = cv::max(abs(centerOfNewObjectRect.x - centerOfCurrectObjectPredictedRect.x), abs(centerOfNewObjectRect.y - centerOfCurrectObjectPredictedRect.y));
                        float distance = sqrt(pow(centerOfNewObjectRect.x - centerOfCurrectObjectPredictedRect.x, 2) + pow(centerOfNewObjectRect.y - centerOfCurrectObjectPredictedRect.y, 2));
                        
                        float lastAverageObjectDistance = 0;

                        //if (outputFrame.detectedObjectsInScene.at(j).getObjectRoute().size() > 2) {

                        if (false) {
                            //for (size_t k = 0; k < 4; k++)
                            //{
                                cv::Point currentPointN = outputFrame.detectedObjectsInScene.at(j).getObjectRoute().at(outputFrame.detectedObjectsInScene.at(j).getObjectRoute().size() - 1);
                                cv::Point currentPointN_1 = outputFrame.detectedObjectsInScene.at(j).getObjectRoute().at(outputFrame.detectedObjectsInScene.at(j).getObjectRoute().size() - 2);
                                lastAverageObjectDistance = lastAverageObjectDistance + sqrt(pow(currentPointN.x - currentPointN_1.x, 2) + pow(currentPointN.y - currentPointN_1.y, 2));
                            //}
                                cv::line(frame, centerOfCurrectObjectPredictedRect, centerOfNewObjectRect, cv::Scalar(0, 0, 255), 10);
                                cv::line(frame, currentPointN, currentPointN_1, cv::Scalar(255, 0, 0), 10);
                            //lastAverageObjectDistance = lastAverageObjectDistance / 2;

                            //std::cout << "\r" << distance << " / " << lastAverageObjectDistance << std::endl;
                            if(lastAverageObjectDistance)
                            if (distance < nearestRectDistance) {
                                //if (distance <= (lastAverageObjectDistance * 8) || lastAverageObjectDistance == 0) {
                                    nearestRectDistance = distance;
                                    nearestRectIndex = j;
                                //}
                            }
                        }
                        else {
                            if (distance < nearestRectDistance) {
                                nearestRectDistance = distance;
                                nearestRectIndex = j;
                            }
                        }
                    }
                }

                // If new detected object is near old detected object
                if (nearestRectIndex != -1) {
                    float maxHeightResize = 0.80;
                    float maxWidthResize = 0.80;
                    cv::Rect oldRect = outputFrame.detectedObjectsInScene.at(nearestRectIndex).getObjectRect();

                    float changedSize = box.width - oldRect.width;
                    if (abs(changedSize / box.width) > maxWidthResize)
                    {                        
                        //std::cout << "Percentage width resize: " << maxWidthResize << std::endl;
                        changedSize = 1.0 * box.width;
                        int widthToChange = (int)round(changedSize/2.0);
                        box.x += widthToChange;
                        box.width += widthToChange;
                    }
                    changedSize = box.height - oldRect.height;
                    if (abs(changedSize / box.width) > maxHeightResize)
                    {
                        //std::cout << "Percentage width resize: " << maxHeightResize << std::endl;
                        changedSize = 1.0 * box.height;
                        int heightToChange = (int)round(changedSize / 2.0);
                        box.y += heightToChange;
                        box.height += heightToChange;
                    }
                    outputFrame.detectedObjectsInScene.at(nearestRectIndex).setObjectRect(box); // update detected object rectangle
                    outputFrame.detectedObjectsInScene.at(nearestRectIndex).setObjectConfidence(confidences[idx]); // update detected object confidence
                    outputFrame.detectedObjectsInScene.at(nearestRectIndex).addObjectRoutePoint(cv::Point(box.br() + box.tl()) * 0.5); // detected object add new tracking router point
                                        
                    if (outputFrame.detectedObjectsInScene.at(nearestRectIndex).getOldObjectName() != outputFrame.detectedObjectsInScene.at(nearestRectIndex).getObjectName())
                    {
                        if (outputFrame.detectedObjectsInScene.at(nearestRectIndex).getOldObjectName() == classes[classIds[idx]])
                            outputFrame.detectedObjectsInScene.at(nearestRectIndex).setObjectName(classes[classIds[idx]]);
                    }

                    outputFrame.detectedObjectsInScene.at(nearestRectIndex).setOldObjectName(classes[classIds[idx]]);

                    // reset TTL
                    outputFrame.detectedObjectsInScene.at(nearestRectIndex).resetTtl();

                    cv::Rect intersectionBox;
                    cv::Mat LP;
                    for (size_t LPid = 0; LPid < LPIds.size(); LPid++)
                    {
                        int aktualLP = LPIds[LPid];
                        intersectionBox = boxes[aktualLP] & box;
                        if (intersectionBox.width > 0 && intersectionBox.height > 0)
                        {
                            cv::Rect actualBBox = boxes[aktualLP];
                            cv::Point highResTl = cv::Point(int(floor((float(actualBBox.tl().x) / frame.cols) * outputFrame.frameOriginal.cols)), int(floor((float(actualBBox.tl().y) / frame.rows) * outputFrame.frameOriginal.rows)));
                            //int highResWidth = int(floor((float(actualBBox.width) / frame.rows) * outputFrame.frameOriginal.rows));
                            //int highResHeight = int(floor((float(actualBBox.height) / frame.cols) * outputFrame.frameOriginal.cols));
                            int highResWidth = int(floor((float(actualBBox.width) / frame.cols) * outputFrame.frameOriginal.cols));
                            int highResHeight = int(floor((float(actualBBox.height) / frame.rows) * outputFrame.frameOriginal.rows));
                            /*if (highResTl.x > outputFrame.frameOriginal.cols)
                                highResTl.x = outputFrame.frameOriginal.cols - 1;
                            if (highResTl.y > outputFrame.frameOriginal.rows)
                                highResTl.y = outputFrame.frameOriginal.rows - 1;

                            if (highResWidth + highResTl.x > outputFrame.frameOriginal.cols)
                                highResWidth -= ((highResWidth + highResTl.x) - outputFrame.frameOriginal.cols);
                            if (highResHeight + highResTl.y > outputFrame.frameOriginal.rows)
                                highResHeight -= ((highResHeight + highResTl.y) - outputFrame.frameOriginal.rows);

                            if (highResTl.x < 0)
                                highResTl.x = 0;
                            if (highResTl.y < 0)
                                highResTl.y = 0;

                            if (highResWidth < 0)
                                highResWidth = 0;
                            if (highResHeight < 0)
                                highResHeight = 0;*/
                            //cv::Mat testMat;
                            //outputFrame.frameOriginal.copyTo(testMat);
                            //cv::Rect test = cv::Rect(highResTl.x, highResTl.y, highResWidth, highResHeight);
                            //cv::rectangle(testMat, test.tl(), test.br(), cv::Scalar(0,0,0), 2);
                            //resize(testMat, testMat, cv::Size(1024, 820), cv::INTER_LINEAR);

                            //cv::imshow("LPHighRes.jpg", testMat);
                            //cv::waitKey(1250); 

	                        //std::cout << "highResTlx: " << highResTl.x << "highResTly: " << highResTl.y << std::endl;
							//std::cout << "highResWidth: " << highResWidth << "highResHeight: " << highResHeight << std::endl;
                           

                            cv::Mat oldLP = outputFrame.detectedObjectsInScene.at(nearestRectIndex).getLP();
                            //cv::Mat oldLP = cv::Mat::zeros(2, 2, CV_8UC1);

                            if(oldLP.cols > 0 && oldLP.rows > 0)
                            {
                                //if (highResWidth > oldLP.cols && highResHeight > oldLP.rows)
                                if ((highResWidth+highResHeight) > (oldLP.cols + oldLP.rows))
                                {
                                    if (highResTl.x > outputFrame.frameOriginal.cols)
                                        highResTl.x = outputFrame.frameOriginal.cols - 1;
                                    if (highResTl.y > outputFrame.frameOriginal.rows)
                                        highResTl.y = outputFrame.frameOriginal.rows - 1;

                                    if (highResWidth + highResTl.x > outputFrame.frameOriginal.cols)                                        
                                        highResWidth -=((highResWidth + highResTl.x) - outputFrame.frameOriginal.cols);
                                    if (highResHeight + highResTl.y > outputFrame.frameOriginal.rows)
                                        highResHeight -= ((highResHeight + highResTl.y) - outputFrame.frameOriginal.rows);

                                    if (highResTl.x < 0)
                                        highResTl.x = 0;
                                    if (highResTl.y < 0)
                                        highResTl.y = 0;

                                    if (highResWidth < 0)
                                        highResWidth = 0;
                                    if (highResHeight < 0)
                                        highResHeight = 0;
                                    //std::cout << __LINE__ << " " << __func__ << std::endl;
                                    LP = outputFrame.frameOriginal(cv::Rect(highResTl.x, highResTl.y, highResWidth, highResHeight));
                                    //std::cout << __LINE__ << " " << __func__ << std::endl;
                                    outputFrame.detectedObjectsInScene.at(nearestRectIndex).setLP(LP);
                                    //cv::imshow("LP", LP);
                                    break;
                                }


                            }else
                            {
                                if (highResTl.x > outputFrame.frameOriginal.cols)
                                    highResTl.x = outputFrame.frameOriginal.cols - 1;
                                if (highResTl.y > outputFrame.frameOriginal.rows)
                                    highResTl.y = outputFrame.frameOriginal.rows - 1;

                                if (highResWidth + highResTl.x > outputFrame.frameOriginal.cols)
                                    highResWidth -= ((highResWidth + highResTl.x) - outputFrame.frameOriginal.cols);
                                if (highResHeight + highResTl.y > outputFrame.frameOriginal.rows)
                                    highResHeight -= ((highResHeight + highResTl.y) - outputFrame.frameOriginal.rows);

                                if (highResTl.x < 0)
                                    highResTl.x = 0;
                                if (highResTl.y < 0)
                                    highResTl.y = 0;

                                if (highResWidth < 0)
                                    highResWidth = 0;
                                if (highResHeight < 0)
                                    highResHeight = 0;

                                //std::cout << __LINE__ << " " << __func__ << std::endl;

                                //std::cout << "frameOriginal.cols" << outputFrame.frameOriginal.cols << std::endl;
                                //std::cout << "frameOriginal.rows" << outputFrame.frameOriginal.rows << std::endl;

                                //std::cout << "highResTl.x" << highResTl.x << std::endl;
                                //std::cout << "highResTl.y" << highResTl.y << std::endl;
                                //std::cout << "highResWidth" << highResWidth << std::endl;
                                //std::cout << "highResHeight" << highResHeight << std::endl;

                                //LP = outputFrame.frameOriginal(cv::Rect(highResTl.x, highResTl.y, highResWidth, highResHeight));

                                LP = outputFrame.frameOriginal(cv::Rect(highResTl.x, highResTl.y, highResWidth, highResHeight));
                                //std::cout << __LINE__ << " " << __func__ << std::endl;
                                outputFrame.detectedObjectsInScene.at(nearestRectIndex).setLP(LP);
                                //cv::imshow("LP", LP);
                                break;
                            } 
                        }
                    }
                }
                // If new detected object is NOT near old detected object -> add detected object as new object
                else {
                    // get first free index from indexController
                    int index = indexController.getIndex();

                    cv::Rect intersectionBox;

                    /*detectedObjectsInScene.push_back(new DetectedObject(classes[classIds[idx]], confidences[idx], box, index, cv::Point(box.br() + box.tl()) * 0.5));*/
                    cv::Mat LP;
                    for (size_t LPid = 0; LPid < LPIds.size(); LPid++)
                    {
                        int aktualLP = LPIds[LPid];
                        intersectionBox = boxes[aktualLP] & box;
                        if (intersectionBox.width > 0 && intersectionBox.height > 0)
                        {
                            cv::Rect actualBBox = boxes[aktualLP];
                            cv::Point highResTl = cv::Point(int(floor((float(actualBBox.tl().x) / frame.rows) * outputFrame.frameOriginal.rows)), int(floor((float(actualBBox.tl().y) / frame.cols) * outputFrame.frameOriginal.cols)));
                            int highResWidth = int(floor((float(actualBBox.width) / frame.rows) * outputFrame.frameOriginal.rows));
                            int highResHeight = int(floor((float(actualBBox.height) / frame.cols) * outputFrame.frameOriginal.cols));

                            if (highResTl.x > outputFrame.frameOriginal.cols)
                                highResTl.x = outputFrame.frameOriginal.cols - 1;
                            if (highResTl.y > outputFrame.frameOriginal.rows)
                                highResTl.y = outputFrame.frameOriginal.rows - 1;

                            if (highResWidth + highResTl.x > outputFrame.frameOriginal.cols)
                                highResWidth -= ((highResWidth + highResTl.x) - outputFrame.frameOriginal.cols);
                            if (highResHeight + highResTl.y > outputFrame.frameOriginal.rows)
                                highResHeight -= ((highResHeight + highResTl.y) - outputFrame.frameOriginal.rows);

                            if (highResTl.x < 0)
                                highResTl.x = 0;
                            if (highResTl.y < 0)
                                highResTl.y = 0;

                            if (highResWidth < 0)
                                highResWidth = 0;
                            if (highResHeight < 0)
                                highResHeight = 0;


                            //std::cout << "highResTlx: " << highResTl.x << "highResTly: " << highResTl.y << std::endl;
                            //std::cout << "highResWidth: " << highResWidth << "highResHeight: " << highResHeight << std::endl;
                            //std::cout << __LINE__ << " " << __func__ << std::endl;
                            LP = outputFrame.frameOriginal(cv::Rect(highResTl.x, highResTl.y, highResWidth, highResHeight));
                            //std::cout << __LINE__ << " " << __func__ << std::endl;
                            //cv::imshow("LP", LP);
                            break;
                        }
                    }

                    if(LP.rows != 0)
                        outputFrame.detectedObjectsInScene.push_back(DetectedObject(classes[classIds[idx]], confidences[idx], box, index, cv::Point(box.br() + box.tl()) * 0.5, LP));
                    else
                        outputFrame.detectedObjectsInScene.push_back(DetectedObject(classes[classIds[idx]], confidences[idx], box, index, cv::Point(box.br() + box.tl()) * 0.5));

                    
                    //int lastIndex = (detectedObjectsInScene.size() - 1);
                }
            }
            // draw all others detected object
            else {
                drawOtherDetectedObjects(outputFrame, classes[classIds[idx]], confidences[idx], box);
            }
        }
    }
    // Detected object handler, if previously detected objects is NULL
    else {
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            cv::Rect  box = boxes[idx];
            //DetectedObject detectedObject;

            // classIds name of class, confidences is percentage
            if (classes[classIds[idx]] == "car" || classes[classIds[idx]] == "bus" || classes[classIds[idx]] == "truck" || classes[classIds[idx]] == "van" ||  classes[classIds[idx]] == "motorbike" || classes[classIds[idx]] == "bicycle") {

                int index = indexController.getIndex();

                outputFrame.detectedObjectsInScene.push_back(DetectedObject(classes[classIds[idx]], confidences[idx], box, index, cv::Point(box.br() + box.tl()) * 0.5));
            }
            // draw all others detected object
            else {
                drawOtherDetectedObjects(outputFrame, classes[classIds[idx]], confidences[idx], box);
            }
        }
    }


    checkObjectMovementInTrafficArea(frame, outputFrame.detectedObjectsInScene);

    checkUnpermittedStoppingInMonitoredArea(frame, outputFrame.detectedObjectsInScene);

    if (isCarDisplayed) {
        drawCar(outputFrame.frameForProcessing);
        drawCar(frame);
    }

    // check obstacle in monitored area
    checkObstacleInMonitoredArea(frame, outputFrame.frameForProcessing, outputFrame.detectedObjectsInScene);

    // check enter to access control area
    checkUnpermittedVehiclePassage(frame, outputFrame.detectedObjectsInScene);

    // check and draw control line
    checkAndDrawControlLine(frame, outputFrame.detectedObjectsInScene, outputFrame.detectedIndexes);

    // Draw all detected objects
    if (outputFrame.detectedObjectsInScene.size() > 0) {

        // refresh bird view frame
        if (birdView) {
			birdViewFrame = cv::imread(birdViewImagePath);
        }
        
        for (int i = 0; i < outputFrame.detectedObjectsInScene.size(); i++)
        {
            // Draw detected objects trajectory
            if (outputFrame.detectedObjectsInScene.at(i).getObjectName() == "car" || outputFrame.detectedObjectsInScene.at(i).getObjectName() == "bus" || outputFrame.detectedObjectsInScene.at(i).getObjectName() == "truck" || outputFrame.detectedObjectsInScene.at(i).getObjectName() == "van") {
                drawObjectTrajectory(frame, outputFrame.detectedObjectsInScene.at(i));
            }

            if (outputFrame.detectedObjectsInScene.at(i).getObjectRect().x >= 0 && outputFrame.detectedObjectsInScene.at(i).getObjectRect().x <= frame.cols
                && outputFrame.detectedObjectsInScene.at(i).getObjectRect().y >= 0 && outputFrame.detectedObjectsInScene.at(i).getObjectRect().y <= frame.rows) {
                if ((outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width) <= frame.cols
                    && (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height) <= frame.rows) {
						drawPred(outputFrame.detectedObjectsInScene.at(i).getIndexOfObject(), outputFrame.detectedObjectsInScene.at(i).getObjectName(), outputFrame.detectedObjectsInScene.at(i).getObjectConfidence(), outputFrame.detectedObjectsInScene.at(i).getObjectSpeed(distanceConstant, (float)1 / cameraFps, perspectiveMatrix), outputFrame.detectedObjectsInScene.at(i).getObjectRect().x, outputFrame.detectedObjectsInScene.at(i).getObjectRect().y, outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width, outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height, frame, classes);
                }
            }

            if (isDebugMode == true) {
                cv::Rect predRect = outputFrame.detectedObjectsInScene.at(i).getObjectPredRect();
                cv::Point center = outputFrame.detectedObjectsInScene.at(i).getObjectPredRectCenter();

                if (predRect.x >= 0 && predRect.x <= frame.cols
                    && predRect.y >= 0 && predRect.y <= frame.rows) {
                    if ((predRect.x + predRect.width) <= frame.cols
                        && (predRect.y + predRect.height) <= frame.rows) {
                        cv::circle(frame, center, 2, CV_RGB(255, 0, 0), -1);
                        cv::rectangle(frame, predRect, CV_RGB(255, 0, 0), 1);
                    }
                }

                if (outputFrame.detectedObjectsInScene.at(i).getObjectRect().x >= 0 && outputFrame.detectedObjectsInScene.at(i).getObjectRect().x <= frame.cols
                    && outputFrame.detectedObjectsInScene.at(i).getObjectRect().y >= 0 && outputFrame.detectedObjectsInScene.at(i).getObjectRect().y <= frame.rows) {
                    if ((outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width) <= frame.cols
                        && (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height) <= frame.rows) {
                        if (outputFrame.detectedObjectsInScene.at(i).getDirectionOfMovement() == TrafficArea::TrafficAreaDirection::leftTop) {
                            cv::Point rotatePointTemp = rotatePoint(cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x, outputFrame.detectedObjectsInScene.at(i).getObjectRect().y), outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), (alpha_slider)* (M_PI / 180));
                            //cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x, outputFrame.detectedObjectsInScene.at(i).getObjectRect().y), cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                            cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), rotatePointTemp, cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                        }
                        else if (outputFrame.detectedObjectsInScene.at(i).getDirectionOfMovement() == TrafficArea::TrafficAreaDirection::rightTop) {
                            cv::Point rotatePointTemp = rotatePoint(cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width, outputFrame.detectedObjectsInScene.at(i).getObjectRect().y), outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), (alpha_slider) * (M_PI / 180));
                            //cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width, outputFrame.detectedObjectsInScene.at(i).getObjectRect().y), cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                            cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), rotatePointTemp, cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                        }
                        else if (outputFrame.detectedObjectsInScene.at(i).getDirectionOfMovement() == TrafficArea::TrafficAreaDirection::leftBottom) {
                            cv::Point rotatePointTemp = rotatePoint(cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x, (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height)), outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), (alpha_slider) * (M_PI / 180));
                            //cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x, (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height)), cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                            cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), rotatePointTemp, cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                        }
                        else if (outputFrame.detectedObjectsInScene.at(i).getDirectionOfMovement() == TrafficArea::TrafficAreaDirection::rightBottom) {
                            cv::Point rotatePointTemp = rotatePoint(cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width, (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height)), outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), (alpha_slider) * (M_PI / 180));
                            //cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), cv::Point(outputFrame.detectedObjectsInScene.at(i).getObjectRect().x + outputFrame.detectedObjectsInScene.at(i).getObjectRect().width, (outputFrame.detectedObjectsInScene.at(i).getObjectRect().y + outputFrame.detectedObjectsInScene.at(i).getObjectRect().height)), cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                            cv::arrowedLine(frame, outputFrame.detectedObjectsInScene.at(i).getObjectCenter(), rotatePointTemp, cv::Scalar(0, 255, 0), 2, 4, 0, 0.25);
                        }
                        //else {
                        //    cv::putText(frame, "X", outputFrame.detectedObjectsInScene.at(i)->getObjectCenter(), cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(255, 0, 0), 1.2);
                        //}
                    }
                }
            }
        }
    }
    else {
        indexController.resetController();
    }

    if (monitoredArea.getMonitoredArea().size() > 0) {
        if (isNormalState == true) {
            cv::putText(frame, "Situation in monitored area: normal", cv::Point(15, frame.rows - 15), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 255, 0), 1.8, cv::LINE_AA);
        }
        else {
            cv::putText(frame, "Situation in monitored area: risk assessment", cv::Point(15, frame.rows - 15), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 0, 255), 1.8, cv::LINE_AA);
        }
    }

    if (unpermittedVehiclePassageArea.size())
    {
        /*if (isNormalStateInUnpermittedVehiclePassageArea == true) {
            cv::putText(frame, "Situation in unpermitted vehicle passage area: normal", cv::Point(15, frame.rows - 95), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 255, 0), 1.8, cv::LINE_AA);
        }
        else {
            cv::putText(frame, "Situation in unpermitted vehicle passage area: risk assessment", cv::Point(15, frame.rows - 95), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 0, 255), 1.8, cv::LINE_AA);
        }*/
        if (isNormalStateInUnpermittedVehiclePassageArea == false) {
            cv::putText(frame, "Unpermitted vehicle passage in area detected", cv::Point(15, frame.rows - 95), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 0, 255), 1.8, cv::LINE_AA);
        }
    }

    if (monitoredArea.getSizeOfMonitoredArea() > 0) {
        switch (monitoredArea.getStateInMonitoredArea())
        {
        case MonitoredArea::MonitoredAreaState::ready:
            cv::putText(frame, "Monitored area: ready", cv::Point(15, frame.rows - 35), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, CV_RGB(50, 255, 50), 1.8, cv::LINE_AA);
            break;
        case MonitoredArea::MonitoredAreaState::standBy:
            cv::putText(frame, "Monitored area: standby", cv::Point(15, frame.rows - 35), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, CV_RGB(50, 255, 50), 1.8, cv::LINE_AA);
            break;
        case MonitoredArea::MonitoredAreaState::restricted:
            cv::putText(frame, "Monitored area: restricted", cv::Point(15, frame.rows - 35), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, CV_RGB(255, 50, 50), 1.8, cv::LINE_AA);
            break;
        default:
            cv::putText(frame, "Monitored area:", cv::Point(15, frame.rows - 35), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, CV_RGB(50, 255, 50), 1.8, cv::LINE_AA);
            break;
        }
    }

    if (isDetectedObstacleInMonitoredArea == true) {
        cv::rectangle(frame, obstacleInMonitoredArea, cv::Scalar(0, 0, 255), 4 * scaleFactor);
        cv::putText(frame, "Obstacle in monitored area detected", cv::Point(15, frame.rows - 55), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 0, 255), 1.8, cv::LINE_AA);
    }

    if (isDrivingInOppositeSide == true) {
        cv::putText(frame, "Driving in the opposite direction", cv::Point(15, frame.rows - 75), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 0, 255), 1.8, cv::LINE_AA);
    }

    if (isUnpermittedStopping == true) {
        cv::putText(frame, "Unpermitted vehicle stopping", cv::Point(15, frame.rows - 115), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 0, 255), 1.8, cv::LINE_AA);
    }   

    if (isVehiclePassage == true) {
        cv::putText(frame, "Vehicle passage", cv::Point(15, frame.rows - 135), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(255, 0, 0), 1.8, cv::LINE_AA);
    }

    if (controlLine.size() > 0) {
        //jetsonImShow("Krizikova bird view", birdViewFrame);
        int textIndex = 0;
        for (const auto& countedObject : counterLineNumOfObjects) {
            std::string tempText = "Counted " + std::string(countedObject.first) + " : " + std::to_string(countedObject.second);
            cv::putText(frame, tempText, cv::Point(15, (frame.rows - 155) - (textIndex * 20)), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 255, 0), 1.8, cv::LINE_AA);
            textIndex++;
        }
        //std::string tempText = "Counted objects (" + objectNameForControlLine + "): ";
        //cv::putText(frame, tempText + std::to_string(controlLineCounter), cv::Point(25, frame.rows - 130), cv::FONT_HERSHEY_DUPLEX, 1.2 * scaleFactor, cv::Scalar(0, 255, 0), 4.5, cv::LINE_AA);
    }

    // Opposite direction driving
    if (iteratorForResetDrivingInOppositeSideWarning > warningDisplayTime) {
        isDrivingInOppositeSide = false;
        iteratorForResetDrivingInOppositeSideWarning = -1;
    }
    // Detected obstacle in monitored area
    //if (iteratorForResetDetectedObstacleInMonitoredAreaWarning > warningDisplayTime) {
    //    isDetectedObstacleInMonitoredArea = false;
    //    iteratorForResetDetectedObstacleInMonitoredAreaWarning = -1;
        //detectedOtherObjectInMonitoredArea.clear();
    //}
    // Unpermitted vehicle passage
    if (iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning > warningDisplayTime) {
        isNormalStateInUnpermittedVehiclePassageArea = true;
        iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning = -1;
        // iteratorForResetUnpermittedVehiclePassage
        // isUnpermittedVehiclePassage
    }
    // Monitored area state
    if (iteratorForResetNormalStateWarning > warningDisplayTime) {
        isNormalState = true;
        iteratorForResetNormalStateWarning = -1;
    }
    // Unpermitted stopping in monitored area
    if (iteratorForResetUnpermittedStopping > warningDisplayTime) {
        isUnpermittedStopping = false;
        iteratorForResetUnpermittedStopping = -1;
    }
    // Vehicle passage
    if (iteratorForResetVehiclePassage > warningDisplayTime) {
        isVehiclePassage = false;
        iteratorForResetVehiclePassage = -1;
    }

    // iterating for all warnings
    if(iteratorForResetDrivingInOppositeSideWarning != -1)
        iteratorForResetDrivingInOppositeSideWarning = iteratorForResetDrivingInOppositeSideWarning + 1;
    if(iteratorForResetDetectedObstacleInMonitoredAreaWarning != -1)
        iteratorForResetDetectedObstacleInMonitoredAreaWarning = iteratorForResetDetectedObstacleInMonitoredAreaWarning + 1;
    if(iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning != -1)
        iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning = iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning + 1;
    if(iteratorForResetNormalStateWarning != -1)
        iteratorForResetNormalStateWarning = iteratorForResetNormalStateWarning + 1;
    if (iteratorForResetUnpermittedStopping != -1)
        iteratorForResetUnpermittedStopping = iteratorForResetUnpermittedStopping + 1;
    if (iteratorForResetVehiclePassage != -1)
        iteratorForResetVehiclePassage = iteratorForResetVehiclePassage + 1;

    auto end = chrono::steady_clock::now();

    if (postProcessTime.size() > 10) {
        postProcessTime.erase(postProcessTime.begin());
    }
    //else {
        postProcessTime.push_back((float)(chrono::duration_cast<chrono::nanoseconds>(end - start).count()) / 1000000);
    //}

    float postProcessTimeAvg = 0;
    for (int i = 0; i < postProcessTime.size(); i++)
    {
        postProcessTimeAvg = postProcessTimeAvg + postProcessTime.at(i);
    }
    postProcessTimeAvg = postProcessTimeAvg / postProcessTime.size();

    //std::cout << "\r" << "postprocess: " << postProcessTimeAvg << " ms";
}

// Draw the predicted bounding box
void drawPred(int index, cv::String objectName, float conf, float speed, int left, int top, int right, int bottom, cv::Mat& frame, std::vector<std::string>& classes)
{
    //Draw a rectangle displaying the bounding box
    //rectangle(frame, Point(left, top), Point(right, bottom), Scalar(165, 85, 0), 3);
    //rectangle(frame, Point(left, top), Point(right, bottom), Scalar(180, 125, 0), 1);
    //scaleFactor = frame.cols / 2048.0;

    //cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(200, 200, 0), 2.0 * scaleFactor);

    //Get the label for the class name and its confidence
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        label = objectName + ": " + label;
        //if (objectName == "car" || objectName == "bus" || objectName == "truck" || objectName == "traffic light") {
        if (objectName == "car" || objectName == "bus" || objectName == "truck" || objectName == "van") {

            //***float alpha = 0.6;
            //***cv::Mat overlay;
            //***frame.copyTo(overlay);

            //Display the label at the top of the bounding box
            int baseLine;
            cv::Size labelSize = cv::getTextSize("xxx km/h", cv::FONT_HERSHEY_SIMPLEX, 1.0 * scaleFactor, 1, &baseLine);
            top = cv::max(top, labelSize.height);
            //rectangle(frame, cv::Point(left, top - round(1.2 * labelSize.height)), cv::Point(left + round(1 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 0), cv::FILLED);
            // detected object name and probability
            //cv::putText(frame, label, cv::Point(left, top - 5), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 0.45);
            //***rectangle(frame, cv::Rect((left + ((right - left) / 2)) - labelSize.width / 2, (top + ((bottom - top) / 2)) - labelSize.height, labelSize.width, labelSize.height * 2), cv::Scalar(255, 100, 0), 12 * scaleFactor);
            //******rectangle(frame, cv::Rect((left + ((right - left) / 2)) - labelSize.width / 2, (top + ((bottom - top) / 2)) - labelSize.height, labelSize.width, labelSize.height * 2.2), cv::Scalar(255, 100, 0), cv::FILLED);

            //***addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

            cv::putText(frame, label, cv::Point((left + ((right - left) / 2)) - labelSize.width / 2, top + ((bottom - top) / 2)), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.25, cv::LINE_AA);

            // Display speed of object
            //******cv::putText(frame, cv::format("%.0f", speed) + " km/h", cv::Point((left + ((right - left) / 2)) - labelSize.width / 2, (top + ((bottom - top) / 2)) + labelSize.height + 5), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.25, cv::LINE_AA);

            // detected object index
            cv::putText(frame, std::to_string(index), cv::Point(left, bottom), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 0.85, cv::LINE_AA);

            //for (size_t i = 0; i < imgPoints.size(); i++)
            //{
            //    cv::circle(frame, imgPoints[i], 10, cv::Scalar(0, 255, 0), 20);
            //}

            // realD = 19,4m
            // imgD = 470,2 px
            // 1px = 0,0412590387069332
            // v = s / t [m/s]
        }
    }
}

// Call back function for define monitored and traffic area
void mouseEvent(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        //cout << x * 2.39062 << "/" << y * 2.0 << endl;
        //cout << x << "/" << y << endl;

        // App window handler function
        if (appWindow.isDisplayAppWindow() == true && appWindow.isAppWindowClicked(cv::Point(x,y))) {
            if (isDisplayedTrafficAreaScenario == true) {
                appWindow.checkButtonClick(cv::Point(x, y));
                return;
            }
            else if (isDisplayedMonitoredAreaScenario == true) {
                appWindow.checkButtonClick(cv::Point(x, y));
                return;
            }
            else if (isDisplayedUnpermittedVehiclePassageScenario == true) {
                appWindow.checkButtonClick(cv::Point(x, y));
                return;
            }
            else if (isDisplayedTrafficLightAreaScenario == true) {
                appWindow.checkButtonClick(cv::Point(x, y));
                return;
            }
            else if (isDisplayedNoDetectingAreaScenario == true) {
                appWindow.checkButtonClick(cv::Point(x, y));
                return;
            }
            else if (isDisplayedControlLineScenario == true) {
                appWindow.checkButtonClick(cv::Point(x, y));
                return;
            }
        }

        if (isCarDisplayed == true) {
            setCarPosition(x, y);
        }

        /*if (isSelectingAccessControlArea == true) {
            accessControlArea.push_back(cv::Point(x, y));
        }*/

        // Monitored area
        if (isDisplayedMonitoredAreaScenario == true && (indexOfLastActivePointInMonitoredArea != -1 || monitoredArea.getMonitoredArea().size() > 0)) {
            //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
            if (monitoredArea.getMonitoredArea().size() > 0 && monitoredArea.isClickedDirectionRect(cv::Point(x, y)) == true) {
                monitoredArea.addTrafficAreaDirection(cv::Point(x, y), alpha_slider);
            }
            else if (monitoredArea.getMonitoredArea().size() > 0 && monitoredArea.getIndexOfNearestPointInMonitoredArea(cv::Point(x, y)) != -1) {
                indexOfActivePointInMonitoringArea = monitoredArea.getIndexOfNearestPointInMonitoredArea(cv::Point(x, y));
                indexOfLastActivePointInMonitoredArea = indexOfActivePointInMonitoringArea;
            }
            else {
                monitoredArea.addPointToMonitoredArea(cv::Point(x, y)); 
                indexOfActivePointInMonitoringArea = monitoredArea.getIndexOfNearestPointInMonitoredArea(cv::Point(x, y));
                indexOfLastActivePointInMonitoredArea = indexOfActivePointInMonitoringArea;
            }
        }

        // Control line
        else if (isDisplayedControlLineScenario == true && (indexOfActivePointInControlLine != -1 || controlLine.size() > 0)) {
            if (controlLine.size() > 0 && getIndexOfNearestPointInControlLine(cv::Point(x, y)) != -1) {
                indexOfActivePointInControlLine = getIndexOfNearestPointInControlLine(cv::Point(x, y));
                indexOfLastActivePointInControlLine = indexOfActivePointInControlLine;
            }
            else if (controlLine.size() < 2) {
                controlLine.push_back(cv::Point(x, y));
                indexOfActivePointInControlLine = controlLine.size() - 1;
                indexOfLastActivePointInControlLine = indexOfActivePointInControlLine;
            }
        }
        /*else if (isDisplayedControlLineScenario == true) {
            if (controlLine.size() <= 2) {
                controlLine.push_back(cv::Point(x, y));
            }
            else {
                controlLine.at(1) = cv::Point(x, y);
            }
        }*/

        // Unpermitted vehicle passage area
        else if (isDisplayedUnpermittedVehiclePassageScenario == true) {
            if (unpermittedVehiclePassageArea.size() > 0 && getIndexOfNearestPointInUnpermittedVehiclePassage(cv::Point(x, y)) != -1) {
                indexOfActivePointInUnpermittedVehiclePassageArea = getIndexOfNearestPointInUnpermittedVehiclePassage(cv::Point(x, y));
                indexOfLastActivePointInUnpermittedVehiclePassageArea = indexOfActivePointInUnpermittedVehiclePassageArea;
            }
            else if (indexOfLastActivePointInUnpermittedVehiclePassageArea != -1) {
                unpermittedVehiclePassageArea.push_back(cv::Point(x, y));
                indexOfActivePointInUnpermittedVehiclePassageArea = unpermittedVehiclePassageArea.size() - 1;
                indexOfLastActivePointInUnpermittedVehiclePassageArea = indexOfActivePointInUnpermittedVehiclePassageArea;
            }
        }
        // Traffic light area
        else if (isDisplayedTrafficLightAreaScenario == true && (indexOfActivePointInTrafficLightArea != -1 || trafficLightArea.size() > 0)) {
            if (trafficLightArea.size() > 0 && getIndexOfNearestPointInTrafficLightArea(cv::Point(x, y)) != -1) {
                indexOfActivePointInTrafficLightArea = getIndexOfNearestPointInTrafficLightArea(cv::Point(x, y));
                indexOfLastActivePointInTrafficLightArea = indexOfActivePointInTrafficLightArea;
            }
            else if (trafficLightArea.size() < 2) {
                trafficLightArea.push_back(cv::Point(x, y));
                indexOfActivePointInTrafficLightArea = trafficLightArea.size() - 1;
                indexOfLastActivePointInTrafficLightArea = indexOfActivePointInTrafficLightArea;
            }
        }
        // No detecting area
        else if (isDisplayedNoDetectingAreaScenario == true) {
            if (noDetectingArea.size() > 0 && getIndexOfNearestPointInNoDetectingArea(cv::Point(x, y)) != -1) {
                indexOfActivePointInDetectingArea = getIndexOfNearestPointInNoDetectingArea(cv::Point(x, y));
                indexOfLastActivePointInDetectingArea = indexOfActivePointInDetectingArea;
            }
            else if (indexOfLastActivePointInDetectingArea != -1) {
                noDetectingArea.push_back(cv::Point(x, y));
                indexOfActivePointInDetectingArea = noDetectingArea.size() - 1;
                indexOfLastActivePointInDetectingArea = indexOfActivePointInDetectingArea;
            }
        }
        // Traffic area
        else if (isDisplayedTrafficAreaScenario == true) {           
            // selecting traffic area direction
            if (trafficArea.size() > indexOfActiveTrafficArea && trafficArea[indexOfActiveTrafficArea].isClickedDirectionRect(cv::Point(x, y)) == true) {
                trafficArea[indexOfActiveTrafficArea].setTrafficAreaDirection(cv::Point(x, y), alpha_slider);
            }
            // selecting traffic area
            else if (trafficArea.size() > 0 && getIndexOfTrafficArea(cv::Point(x, y)) != -1) {
                indexOfActiveTrafficArea = getIndexOfTrafficArea(cv::Point(x, y));
                indexOfLastActivePointInTrafficArea = -1;
            }
            // selecting clicked nearest point in traffic area
            else if (trafficArea.size() > indexOfActiveTrafficArea && getIndexOfNearestPointInTrafficArea(cv::Point(x, y)) != -1) {
                indexOfActivePointInTrafficArea = getIndexOfNearestPointInTrafficArea(cv::Point(x, y));
                indexOfLastActivePointInTrafficArea = getIndexOfNearestPointInTrafficArea(cv::Point(x, y));
            }
            else if (indexOfActiveTrafficArea != -1) {
                if (trafficArea.size() > indexOfActiveTrafficArea) {
                    trafficArea[indexOfActiveTrafficArea].addPointToTrafficArea(cv::Point(x, y));
                    indexOfLastActivePointInTrafficArea = trafficArea[indexOfActiveTrafficArea].getSizeOfTrafficArea() - 1;
                }
                else {
                    TrafficArea newTrafficArea;
                    if (trafficArea.size() > 0) {
                        newTrafficArea.setTrafficAreaIndex(trafficArea.size());
                    }
                    else {
                        newTrafficArea.setTrafficAreaIndex(0);
                    }
                    newTrafficArea.addPointToTrafficArea(cv::Point(x, y));
                    trafficArea.push_back(newTrafficArea);
                    //indexOfLastActivePointInTrafficArea = trafficArea[indexOfActiveTrafficArea].getSizeOfTrafficArea() - 1;
                }
            }
        }
    }

    else if (event == cv::EVENT_MOUSEMOVE) {
        // Monitored area
        if (isDisplayedMonitoredAreaScenario == true && monitoredArea.getSizeOfMonitoredArea() > 0 && indexOfActivePointInMonitoringArea != -1) {
            monitoredArea.updatePointInMonitoredArea(indexOfActivePointInMonitoringArea, cv::Point(x, y));
        }
        // Traffic area
        else if (isDisplayedTrafficAreaScenario == true && trafficArea.size() > 0 && indexOfActivePointInTrafficArea != -1) {
            trafficArea[indexOfActiveTrafficArea].updatePointInTrafficArea(indexOfActivePointInTrafficArea, cv::Point(x, y));
        }
        // Traffic light area
        else if (isDisplayedTrafficLightAreaScenario == true && trafficLightArea.size() > 0 && indexOfActivePointInTrafficLightArea != -1) {
            trafficLightArea[indexOfActivePointInTrafficLightArea] = cv::Point(x, y);
        }
        // Control line
        else if (isDisplayedControlLineScenario == true && controlLine.size() > 0 && indexOfActivePointInControlLine != -1) {
            controlLine[indexOfActivePointInControlLine] = cv::Point(x, y);
        }
        // Nodetecting area
        else if (isDisplayedNoDetectingAreaScenario == true && noDetectingArea.size() > 0 && indexOfActivePointInDetectingArea != -1) {
            noDetectingArea[indexOfActivePointInDetectingArea] = cv::Point(x, y);
        }
        else if (isDisplayedUnpermittedVehiclePassageScenario == true && unpermittedVehiclePassageArea.size() > 0 && indexOfActivePointInUnpermittedVehiclePassageArea != -1) {
            unpermittedVehiclePassageArea[indexOfActivePointInUnpermittedVehiclePassageArea] = cv::Point(x, y);
        }
        //else if (isDisplayedControlLineScenario == true) {
        //    if (controlLine.size() == 1) {
        //        controlLine.push_back(cv::Point(x, y));
        //    }
       //     else if (controlLine.size() > 1 && controlLine[1].x > 0 && controlLine[1].y > 0) {
        //        controlLine.at(1) = cv::Point(x, y);
        //    }
       // }
    }

    if (event == cv::EVENT_LBUTTONUP) {
        if (isDisplayedMonitoredAreaScenario == true && monitoredArea.getSizeOfMonitoredArea() > 0 && indexOfActivePointInMonitoringArea != -1) {
            indexOfActivePointInMonitoringArea = -1;
        }
        else if (isDisplayedTrafficAreaScenario == true && trafficArea.size() > 0 && indexOfActivePointInTrafficArea != -1) {
            indexOfActivePointInTrafficArea = -1;
        }
        else if (isDisplayedTrafficLightAreaScenario == true && trafficLightArea.size() > 0 && indexOfActivePointInTrafficLightArea != -1) {
            indexOfActivePointInTrafficLightArea = -1;
        }
        else if (isDisplayedControlLineScenario == true && controlLine.size() > 0 && indexOfActivePointInControlLine != -1) {
            indexOfActivePointInControlLine = -1;
        }
        else if (isDisplayedNoDetectingAreaScenario == true && noDetectingArea.size() > 0 && indexOfActivePointInDetectingArea != -1) {
            indexOfActivePointInDetectingArea = -1;
        }
        else if (isDisplayedUnpermittedVehiclePassageScenario == true && unpermittedVehiclePassageArea.size() > 0 && indexOfActivePointInUnpermittedVehiclePassageArea != -1) {
            indexOfActivePointInUnpermittedVehiclePassageArea = -1;
        }
       // else if (controlLine.size() > 1) {
        //    isDisplayedControlLineScenario = false;
        //}
    }
}

// Mouse event function for demonstration car
void mouseEventForCarMoving(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (isCarDisplayed == true) {
            setCarPosition(x, y);
        }
    }
}

// Draw object trajectory
void drawObjectTrajectory(cv::Mat& frame, DetectedObject& detectedObjectsInScene) {

    std::vector<cv::Point2f> objectRouteTemp = detectedObjectsInScene.getObjectRoute();
    double positionOffset = (frame.rows / 100) * 2.5;
    double size = 0.1;

    for (size_t j = 0; j < detectedObjectsInScene.getObjectRouteSize(); j++) {
        if (j != 0) {
            size = 0.1 * j;
            cv::line(frame, objectRouteTemp.at(j), objectRouteTemp.at(j - 1), cv::Scalar(0, 255, 255), 2);
        }
        cv::drawMarker(frame, objectRouteTemp.at(j), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 6, 2);
    }

    if (birdView) {
        for (int i = 0; i < objectRouteTemp.size(); i++)
        {
            if (objectRouteTemp.at(i).y <= frame.rows / 2) {
                objectRouteTemp.at(i) = cv::Point2f(objectRouteTemp.at(i).x, objectRouteTemp.at(i).y + positionOffset);
            }
        }

        if (detectedObjectsInScene.getObjectRouteSize() > 1) {
            std::vector<cv::Point2f> outputPoints;

            //perspectiveMatrix = cv::getPerspectiveTransform(imgPoints, outputImgPoints);
            cv::perspectiveTransform(objectRouteTemp, outputPoints, perspectiveMatrix);

            if (outputPoints.size() > 0) {
                //cv::circle(birdViewFrame, outputPoints[outputPoints.size() - 1], 5, cv::Scalar(0, 0, 255), 5);
                if (birdView)
                    cv::rectangle(birdViewFrame, cv::Point(outputPoints[outputPoints.size() - 1].x - 10, outputPoints[outputPoints.size() - 1].y - 10), cv::Point(outputPoints[outputPoints.size() - 1].x + 10, outputPoints[outputPoints.size() - 1].y + 10), cv::Scalar(255, 0, 0), 4);

                for (size_t j = 0; j < outputPoints.size(); j++) {
                    if (j != 0) {
                        size = 0.1 * j;
                        if (birdView)
                            cv::line(birdViewFrame, outputPoints.at(j), outputPoints.at(j - 1), cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
            
            #if !defined(LINUX)
            if (birdView) {
                //cv::putText(birdViewFrame, "Counted vehicle: " + std::to_string(controlLineCounter), cv::Point(10, 25), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 0), 0.85, cv::LINE_AA);
                cv::imshow("Krizikova bird view", birdViewFrame);
                cv::waitKey(1);  
            }
            #endif
        }
    }  
}

// draw car for demo
void drawCar(cv::Mat frame) {
    cv::String pathToCar = "car.png";

    cv::Mat car;
    car = cv::imread(pathToCar, cv::IMREAD_UNCHANGED);

    cv::Size size = cv::Size(car.cols * 0.15, car.rows * 0.15);
    resize(car, car, size, 0.0, 0.0, cv::INTER_AREA);

    // The position where to draw the car
    cv::Point2i carPositionInFrame(carPosition.x + (carPosition.width / 2), carPosition.y + (carPosition.height / 2));

    // Blend the car with frame
    for (int y = 0; y < car.rows; ++y)
    {
        for (int x = 0; x < car.cols; ++x)
        {
            cv::Vec4b& pixel = car.at<cv::Vec4b>(y, x);
            cv::Vec3b& pixel_dst = frame.at<cv::Vec3b>(y + carPositionInFrame.y - size.height / 2, x + carPositionInFrame.x - size.width / 2); // destination image's (face) pixel

            pixel_dst = (pixel[3] / 255.0f) * cv::Vec3b(pixel[0], pixel[1], pixel[2]) + (1 - pixel[3] / 255.0f) * pixel_dst; // we blend the two pixels according to the transparency of the mustache's pixel.
        }
    }
    //rectangle(frame, carPosition, cv::Scalar(0, 255, 0), 2, 8, 0);
}

// Get index of traffic area
int getIndexOfTrafficArea(cv::Point point) {
    for (int i = 0; i < trafficArea.size(); i++)
    {
        if (pointPolygonTest(trafficArea[i].getTrafficArea(), point, false) > 0) {
            return i;
        }
    }
    return -1;
}


// Set demonstration car position
void setCarPosition(int x, int y) {
    carPosition = cv::Rect(cv::Point(x - 75, y - 50), cv::Point(x + 75, y + 50));
}

// check enter to access control area
void checkUnpermittedVehiclePassage(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene) {
    if (unpermittedVehiclePassageArea.size() > 1 && detectedObjectsInScene.size() > 0) {
        for (int i = 0; i < detectedObjectsInScene.size(); i++) {
            if (std::find(unpermittedVehiclePassageJson.begin(), unpermittedVehiclePassageJson.end(), detectedObjectsInScene[i].getObjectName()) != unpermittedVehiclePassageJson.end()) {
                cv::Point detectedObjectCenter;
                detectedObjectCenter.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width / 2);
                detectedObjectCenter.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height / 2);

                if (pointPolygonTest(unpermittedVehiclePassageArea, detectedObjectCenter, false) > 0) {
                    //if (detectedObjectsInScene.at(i).getObjectName() == "van" || detectedObjectsInScene.at(i).getObjectName() == "truck" || detectedObjectsInScene.at(i).getObjectName() == "bus") {
                    isNormalStateInUnpermittedVehiclePassageArea = false;
                    iteratorForResetNormalStateInUnpermittedVehiclePassageAreaWarning = 0;
                    evenTypeHandler->sendUnpermittedVehiclePassage(frame, detectedObjectsInScene.at(i), barriers, trafficLight);
                    //}
                }
            }
        }
    }
}

// draw access control area
void drawUnpermittedVehiclePassageArea(cv::Mat& frame) {
    if (unpermittedVehiclePassageArea.size() > 0) {
        cv::Mat overlay;
        overlay = frame.clone();

        float alpha = 0.75;

        for (size_t i = 0; i < unpermittedVehiclePassageArea.size(); i++)
        {
            cv::fillPoly(frame, unpermittedVehiclePassageArea, cv::Scalar(200, 0, 0), 0);
            addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
        }
    }
}

// check traffic light state
void trafficLightCheck(OutputFrame& outputFrame) {
    cv::Mat frame = outputFrame.frameForDisplay;
    std::vector<DetectedObject> detectedObjectsInScene = outputFrame.detectedObjectsInScene;
    std::vector<TrafficLight> trafficLights = outputFrame.trafficLights;

    if (frameBrightnessCounter >= (checkPartOfDayIntervalInSeconds * cameraFps) || frameBrightnessCounter == -1) {
        frameBrightnessCounter = 0;
        
        cv::Mat grayFrame;
        double avgPixelsValue = 0;
        
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
        
        grayFrame = grayFrame(selectedAreaForDayOrNightComputing);
        
        for (int i = 0; i < grayFrame.rows; i++)
        {
            for (int j = 0; j < grayFrame.cols; j++)
            {
                if ((int)grayFrame.at<uchar>(i, j) >= 250) {
                    avgPixelsValue = avgPixelsValue + 0;
                }
                else {
                    avgPixelsValue = avgPixelsValue + grayFrame.at<uchar>(i, j);
                }
            }
        }
        
        avgPixelsValue = avgPixelsValue / (grayFrame.rows * grayFrame.cols);
        //std::cout << "avgPixelsValue: " << avgPixelsValue << std::endl;
        if (avgPixelsValue <= frameBrightnessThresholdForTrafficLight) {
            partOfDay = PartsOfDay::night;
            //std::cout << "> Part of day: NIGHT detected in frame" << std::endl;
        }
        else {
            partOfDay = PartsOfDay::day;
            //std::cout << "> Part of day: DAY detected in frame" << std::endl;
        }     
    }
    //else if (frameBrightnessCounter < 50000) {
    else {
        frameBrightnessCounter++;
    }
    
    // Check if dnn detect a traffic light in selected area
    // if not, isDnnDetectedTrafficLightInSelectedArea is false and for evalue traffic light state will be use conventional method
    bool isDnnDetectedTrafficLightInSelectedArea = false;

    if (partOfDay == PartsOfDay::day) {
        if (trafficLights.size() > 0 && trafficLightArea.size() > 1) {
            //std::cout << sqrt(pow(trafficLightArea[0].x - trafficLightArea[1].x, 2) + pow(trafficLightArea[0].y - trafficLightArea[1].y, 2)) << std::endl;
            for (size_t i = 0; i < trafficLights.size(); i++)
            {
                //cv::rectangle(frame, cv::Rect(cv::Point(trafficLights[i].getTrafficLightPosition().x, trafficLights[i].getTrafficLightPosition().y), cv::Point((trafficLights[i].getTrafficLightPosition().x + trafficLights[i].getTrafficLightPosition().width), (trafficLights[i].getTrafficLightPosition().y + trafficLights[i].getTrafficLightPosition().height))), cv::Scalar(0, 0, 255), 2);
                if (cv::Rect(cv::Point(trafficLightArea[0].x, trafficLightArea[0].y), cv::Point(trafficLightArea[1].x, trafficLightArea[1].y)).contains(cv::Point((trafficLights[i].getTrafficLightPosition().x + (trafficLights[i].getTrafficLightPosition().width / 2)),
                    (trafficLights[i].getTrafficLightPosition().y + (trafficLights[i].getTrafficLightPosition().height / 2))))) {

                    isDnnDetectedTrafficLightInSelectedArea = true;

                    //std::cout << trafficLights[i].getTrafficLightColor() << std::endl;
                    if (trafficLightStateDelay < 0) {
                        if (trafficLights[i].getTrafficLightColor() == TrafficLight::TrafficLightColor::restricted) {
                            monitoredArea.setRestrictedStateInMonitoredArea();
                            trafficLight.setTrafficLightColorString("red");
                            //cv::rectangle(frame, cv::Rect(cv::Point(trafficLights[i].getTrafficLightPosition().x / frameResizeRationX, trafficLights[i].getTrafficLightPosition().y / frameResizeRationY), cv::Point((trafficLights[i].getTrafficLightPosition().x + trafficLights[i].getTrafficLightPosition().width) / frameResizeRationX, (trafficLights[i].getTrafficLightPosition().y + trafficLights[i].getTrafficLightPosition().height) / frameResizeRationY)), cv::Scalar(0, 0, 255), 2);
                        }
                        else if (trafficLights[i].getTrafficLightColor() == TrafficLight::TrafficLightColor::standBy) {
                            monitoredArea.setStandByStateInMonitoredArea();
                            trafficLight.setTrafficLightColorString("yellow");
                            //cv::rectangle(frame, cv::Rect(cv::Point(trafficLights[i].getTrafficLightPosition().x / frameResizeRationX, trafficLights[i].getTrafficLightPosition().y / frameResizeRationY), cv::Point((trafficLights[i].getTrafficLightPosition().x + trafficLights[i].getTrafficLightPosition().width) / frameResizeRationX, (trafficLights[i].getTrafficLightPosition().y + trafficLights[i].getTrafficLightPosition().height) / frameResizeRationY)), cv::Scalar(255, 0, 0), 2);
                        }
                        else if (trafficLights[i].getTrafficLightColor() == TrafficLight::TrafficLightColor::ready) {
                            monitoredArea.setReadyStateInMonitoredArea();
                            trafficLight.setTrafficLightColorString("green");
                            //cv::rectangle(frame, cv::Rect(cv::Point(trafficLights[i].getTrafficLightPosition().x / frameResizeRationX, trafficLights[i].getTrafficLightPosition().y / frameResizeRationY), cv::Point((trafficLights[i].getTrafficLightPosition().x + trafficLights[i].getTrafficLightPosition().width) / frameResizeRationX, (trafficLights[i].getTrafficLightPosition().y + trafficLights[i].getTrafficLightPosition().height) / frameResizeRationY)), cv::Scalar(0, 255, 0), 2);
                        }
                        trafficLightStateDelay = cameraFps / 2;
                    }
                    else {
                        trafficLightStateDelay = trafficLightStateDelay - 1;
                    }
                }
            }
        }
    }
    if (isDnnDetectedTrafficLightInSelectedArea == false || partOfDay == PartsOfDay::night) {
        
        if (trafficLightArea.size()) {
            
            if (locationTypeJson == "crossingWithTrafficLights") {
                
                cv::Mat redColor, yellowColor, greenColor;
                //std::cout << "jop" << std::endl;


                //std::cout << "trafficLightArea[0]: " << trafficLightArea[0] << std::endl;
                //std::cout << "trafficLightArea[1]: " << trafficLightArea[1] << std::endl;
                cv::Rect trafficLightRect = cv::Rect(trafficLightArea[0], trafficLightArea[1]);
                
                //std::cout << "trafficLightRect.x - ((trafficLightRect.width / 100.0) * 15.0): " << trafficLightRect.x - ((trafficLightRect.width / 100.0) * 15.0) << std::endl;
                //std::cout << "trafficLightRect.y - (trafficLightRect.height / 100.0) * 15.0): " << trafficLightRect.y - (trafficLightRect.height / 100.0) * 15.0 << std::endl;
                //std::cout << "trafficLightRect.x + trafficLightRect.width) + ((trafficLightRect.width / 100.0) * 15.0): " << (trafficLightRect.x + trafficLightRect.width) + ((trafficLightRect.width / 100.0) * 15.0) << std::endl;
                //std::cout << "(trafficLightRect.y + trafficLightRect.height) + (trafficLightRect.height / 100.0) * 15.0): " << (trafficLightRect.y + trafficLightRect.height) + (trafficLightRect.height / 100.0) * 15.0 << std::endl;

                cv::Rect biggerRect = cv::Rect(cv::Point(trafficLightRect.x - ((trafficLightRect.width / 100.0) * 15.0), trafficLightRect.y - (trafficLightRect.height / 100.0) * 15.0), cv::Point((trafficLightRect.x + trafficLightRect.width) + ((trafficLightRect.width / 100.0) * 15.0), (trafficLightRect.y + trafficLightRect.height) + (trafficLightRect.height / 100.0) * 15.0));
                
                cv::Mat trafficLightImg = frame(biggerRect);
                
                //std::cout << "trafficLightImg.cols, trafficLightImg.rows * 1 / 3: " << trafficLightImg.cols << "::::" << trafficLightImg.rows * 1 / 3 << std::endl;
                //std::cout << "trafficLightImg.rows * 1 / 3, trafficLightImg.cols, trafficLightImg.rows * 1 / 3: " << trafficLightImg.rows * 1 / 3 << "::::" <<  trafficLightImg.cols << "::::" << trafficLightImg.rows * 1 / 3 << std::endl;
                //std::cout << "trafficLightImg.rows - (trafficLightImg.rows * 1 / 3), trafficLightImg.cols, trafficLightImg.rows * 1 / 3: " << trafficLightImg.rows - (trafficLightImg.rows * 1 / 3) << "::::" << trafficLightImg.cols << "::::" << trafficLightImg.rows * 1 / 3 << std::endl;

                redColor = trafficLightImg(cv::Rect(0, 0, trafficLightImg.cols, trafficLightImg.rows * 1 / 3));
                yellowColor = trafficLightImg(cv::Rect(0, trafficLightImg.rows * 1 / 3, trafficLightImg.cols, trafficLightImg.rows * 1 / 3));
                greenColor = trafficLightImg(cv::Rect(0, trafficLightImg.rows - (trafficLightImg.rows * 1 / 3), trafficLightImg.cols, trafficLightImg.rows * 1 / 3));

                cv::cvtColor(redColor, redColor, cv::COLOR_BGR2GRAY);
                cv::cvtColor(yellowColor, yellowColor, cv::COLOR_BGR2GRAY);
                cv::cvtColor(greenColor, greenColor, cv::COLOR_BGR2GRAY);

                cv::medianBlur(redColor, redColor, 9);
                cv::medianBlur(yellowColor, yellowColor, 9);
                cv::medianBlur(greenColor, greenColor, 9);

                cv::threshold(redColor, redColor, 215, 255, cv::THRESH_BINARY);
                cv::threshold(yellowColor, yellowColor, 215, 255, cv::THRESH_BINARY);
                cv::threshold(greenColor, greenColor, 215, 255, cv::THRESH_BINARY);

                int nonZeroPixelsOfRed = cv::countNonZero(redColor);
                int nonZeroPixelsOfYellow = cv::countNonZero(yellowColor);
                int nonZeroPixelsOfGreen = cv::countNonZero(greenColor);

                if ((nonZeroPixelsOfRed > nonZeroPixelsOfYellow) && (nonZeroPixelsOfRed > nonZeroPixelsOfGreen))
                {
                    monitoredArea.setRestrictedStateInMonitoredArea();
                    trafficLight.setTrafficLightColorString("red");
                }
                else if (nonZeroPixelsOfYellow > nonZeroPixelsOfGreen)
                {
                    monitoredArea.setStandByStateInMonitoredArea();
                    trafficLight.setTrafficLightColorString("yellow");
                }
                else
                {
                    monitoredArea.setReadyStateInMonitoredArea();
                    trafficLight.setTrafficLightColorString("green");
                }
                //imshow("redColor", redColor);
                //imshow("yellowColor", yellowColor);
                //imshow("greenColor", greenColor);

                //imshow("trafficLightImg", trafficLightImg);
            }
            else if (locationTypeJson == "roadCrossing") {
                cv::Mat redColor, blueColor;

                cv::Rect trafficLightRect = cv::Rect(trafficLightArea[0], trafficLightArea[1]);
                cv::Rect biggerRect = cv::Rect(cv::Point(trafficLightRect.x - ((trafficLightRect.width / 100.0) * 15.0), trafficLightRect.y - (trafficLightRect.height / 100.0) * 15.0), cv::Point((trafficLightRect.x + trafficLightRect.width) + ((trafficLightRect.width / 100.0) * 15.0), (trafficLightRect.y + trafficLightRect.height) + (trafficLightRect.height / 100.0) * 15.0));
                cv::Mat trafficLightImg = frame(biggerRect);

                cv::cvtColor(redColor, redColor, cv::COLOR_BGR2GRAY);
                cv::cvtColor(blueColor, blueColor, cv::COLOR_BGR2GRAY);

                cv::medianBlur(redColor, redColor, 9);
                cv::medianBlur(blueColor, blueColor, 9);

                cv::threshold(redColor, redColor, 215, 255, cv::THRESH_BINARY);
                cv::threshold(blueColor, blueColor, 215, 255, cv::THRESH_BINARY);

                int nonZeroPixelsOfRed = countNonZero(redColor);
                int nonZeroPixelsOfBlue = countNonZero(blueColor);

                if ((nonZeroPixelsOfRed > nonZeroPixelsOfBlue))
                {
                    monitoredArea.setRestrictedStateInMonitoredArea();
                    trafficLight.setTrafficLightColorString("red");
                }
                else
                {
                    monitoredArea.setReadyStateInMonitoredArea();
                    trafficLight.setTrafficLightColorString("green");
                }
                //imshow("redColor", redColor);
                //imshow("blueColor", blueColor);
                //imshow("trafficLightImg", trafficLightImg);
            }
        }     
    } 
}

//void loadSystemConfiguration()
//{
//    // read a JSON file
//    std::ifstream inputStream(appSystemConfigName);
//    nlohmann::json jsonReader;
//
//    if (inputStream.is_open())
//    {
//        inputStream >> jsonReader;
//        inputStream.close();
//        if(jsonReader.contains("cameraId"))
//			cameraIdJson = jsonReader["cameraId"];
//        if (jsonReader.contains("name"))
//			nameJson = jsonReader["name"];
//        if (jsonReader.contains("city"))
//            cityJson = jsonReader["city"];
//        if (jsonReader.contains("street"))
//            streetJson = jsonReader["street"];
//        //if (jsonReader.contains("direction"))
//        //    directionJson = jsonReader["direction"];
//        if (jsonReader.contains("info"))
//            infoJson = jsonReader["info"];
//        if (jsonReader.contains("locationType"))
//            locationTypeJson = jsonReader["locationType"];
//        if (jsonReader.contains("laneNumber"))
//            laneNumberJson = jsonReader["laneNumber"];
//
//        if (jsonReader.contains("trafficAreaNames"))        
//            trafficAreaNamesJson = jsonReader["trafficAreaNames"].get<std::vector<std::string>>();
//            //std::cout << "jsonReader get: " << jsonReader["trafficAreaNames"].get<std::vector<std::string>>() << std::endl;
//        //trafficAreaNamesJson = jsonReader["trafficAreaNames"].get<std::vector<std::string>>();      
//        //std::cout << "trafficAreaNamesJson[0]: " << trafficAreaNamesJson[0] << std::endl;
//
//        if (jsonReader.contains("signalType"))
//            signalTypeJson = jsonReader["signalType"];
//
//        if (jsonReader.contains("barriers"))
//        {
//            barriersJson = jsonReader["barriers"];
//        }
//
//        if(jsonReader.contains("toleratedRedDelay"))
//            toleratedRedDelayJson = jsonReader["toleratedRedDelay"];
//
//        if (jsonReader.contains("sslAderosCertificatePath"))
//        {
//            sslAderosCertificatePathJson = jsonReader["sslAderosCertificatePath"];
//            sslAderosCertificatePath = sslAderosCertificatePathJson.c_str();
//        }
//            
//        if (jsonReader.contains("sslCameraCertificatePath"))
//        {
//            sslCameraCertificatePathJson = jsonReader["sslCameraCertificatePath"];
//            sslCameraCertificatePath = sslCameraCertificatePathJson.c_str();            
//        }
//        if (jsonReader.contains("sslCameraKeyPath"))
//        {
//            sslCameraKeyPathJson = jsonReader["sslCameraKeyPath"];
//            sslCameraKeyPath = sslCameraKeyPathJson.c_str();           
//        }
//
//        if (jsonReader.contains("serverIpAddress"))
//        {           
//            serverIpAddressJson = jsonReader["serverIpAddress"];            
//            targetIPaddress = serverIpAddressJson.data(); 
//        }
//        if (jsonReader.contains("serverPort"))
//        {
//            serverPortJson = jsonReader["serverPort"];
//            targetPort = serverPortJson.c_str();
//        }
//    
//        if (jsonReader.contains("dnnModelConfigurationPath"))
//        {
//            dnnModelConfigurationPathJson = jsonReader["dnnModelConfigurationPath"];
//            modelConfiguration = dnnModelConfigurationPathJson;
//        }
//        if (jsonReader.contains("dnnModelWeightsPath"))
//        {
//            dnnModelWeightsPathJson = jsonReader["dnnModelWeightsPath"];
//
//        }
//
//        if (jsonReader.contains("dnnModelClasses"))
//        {
//            dnnModelClassesJson = jsonReader["dnnModelClasses"];
//
//        }
//
//        if (jsonReader.contains("slowConnection"))
//        {
//            bool slowConnectionJson = jsonReader["slowConnection"];
//            slowConnection = slowConnectionJson;
//        }
//        
//
//        if (jsonReader.contains("showFrames"))
//        {
//            bool showFramesJson = jsonReader["showFrames"];
//            showFrames = showFramesJson;
//        }
//        if (jsonReader.contains("writeVideoWithAnnotations"))
//        {
//            bool writeVideoWithAnnotationsJson = jsonReader["writeVideoWithAnnotations"];
//            writeVideoWithAnnotations = writeVideoWithAnnotationsJson;
//        }
//        if (jsonReader.contains("writeVideoOriginal"))
//        {
//            bool writeVideoOriginalJson = jsonReader["writeVideoOriginal"];
//            writeVideoOriginal = writeVideoOriginalJson;
//        }
//            
//        if (jsonReader.contains("bufferAndSendVideo"))
//        {
//            bool bufferAndSendVideoJson = jsonReader["bufferAndSendVideo"];
//            bufferAndSendVideo = bufferAndSendVideoJson;
//        }  
//
//        if (jsonReader.contains("warningDisplayTime"))
//            warningDisplayTimeJson = jsonReader["warningDisplayTime"];
//
//        monitoredArea.allowedDelayAfterRestricted = toleratedRedDelayJson;
//        // Console output        
//        std::cout << "APPLICATION [System configuration was successfully loaded]" << std::endl;
//    }
//    else {
//        std::cout << "Can not open or read system configuration file: " + appSystemConfigName << std::endl;
//    }
//}


// save configurations to json configuration file
void saveConfiguration()
{
    nlohmann::json jsonWritter;    
    // monitored area
    if (monitoredArea.getMonitoredArea().size() > 0) {
        MonitoredAreaStruct area;
        for (int i = 0; i < monitoredArea.getMonitoredArea().size(); i++)
        {
            Point point;
            point.x = monitoredArea.getMonitoredArea().at(i).x;
            point.y = monitoredArea.getMonitoredArea().at(i).y;
            area.monitoredArea.push_back(point);
        }

        for (int i = 0; i < monitoredArea.getMonitoredAreaDirection().size(); i++)
        {
            // none = 0, leftTop = 1, rightTop = 2, leftBottom = 3, rightBottom = 4
            if (MonitoredArea::MonitoredAreaDirection::none == monitoredArea.getMonitoredAreaDirection().at(i)) {
                area.monitoredAreaDirection.push_back(0);
            }
            else if (MonitoredArea::MonitoredAreaDirection::leftTop == monitoredArea.getMonitoredAreaDirection().at(i)) {
                area.monitoredAreaDirection.push_back(1);
            }
            else if (MonitoredArea::MonitoredAreaDirection::rightTop == monitoredArea.getMonitoredAreaDirection().at(i)) {
                area.monitoredAreaDirection.push_back(2);
            }
            else if (MonitoredArea::MonitoredAreaDirection::leftBottom == monitoredArea.getMonitoredAreaDirection().at(i)) {
                area.monitoredAreaDirection.push_back(3);
            }
            else if (MonitoredArea::MonitoredAreaDirection::rightBottom == monitoredArea.getMonitoredAreaDirection().at(i)) {
                area.monitoredAreaDirection.push_back(4);
            }
        }

        MonitoredAreaStruct monitoredAreaJson = area;
        
        jsonWritter["monitoredArea"] = monitoredAreaJson;
    }
    
    // traffic light
    trafficLightAreaJson.clear();
    if (trafficLightArea.size() > 0) {
        for (int i = 0;  i < trafficLightArea.size();  i++)
        {
            Point point;
            point.x = trafficLightArea.at(i).x;
            point.y = trafficLightArea.at(i).y;
            trafficLightAreaJson.push_back(point);
        }
        jsonWritter["trafficLightArea"] = trafficLightAreaJson;
    }

    // no detecting area
    noDetectingAreaJson.clear();
    if (noDetectingArea.size() > 0) {
        for (int i = 0; i < noDetectingArea.size(); i++)
        {
            Point point;
            point.x = noDetectingArea.at(i).x;
            point.y = noDetectingArea.at(i).y;
            noDetectingAreaJson.push_back(point);
        }
        jsonWritter["noDetectingArea"] = noDetectingAreaJson;
    }

    // Unpermitted vehicle passage
    unpermittedVehiclePassageAreaJson.clear();
    if (unpermittedVehiclePassageArea.size() > 0) {
        for (int i = 0; i < unpermittedVehiclePassageArea.size(); i++)
        {
            Point point;
            point.x = unpermittedVehiclePassageArea.at(i).x;
            point.y = unpermittedVehiclePassageArea.at(i).y;
            unpermittedVehiclePassageAreaJson.push_back(point);
        }
        jsonWritter["unpermittedVehiclePassageArea"] = unpermittedVehiclePassageAreaJson;
    }
    
    // traffic area
    trafficAreaJson.clear();
    if (trafficArea.size() > 0) {
        std::cout << "saving: " << trafficArea.size() << std::endl;
        for (int i = 0; i < trafficArea.size(); i++)
        {
            TrafficAreaStruct area;
            for (size_t j = 0; j < trafficArea.at(i).getSizeOfTrafficArea(); j++)
            {
                Point point;
                point.x = trafficArea.at(i).getTrafficArea().at(j).x;
                point.y = trafficArea.at(i).getTrafficArea().at(j).y;
                area.trafficArea.push_back(point);

            }
            area.trafficAreaIndex = trafficArea.at(i).getTrafficAreaIndex();
            area.trafficAreaDirection = trafficArea.at(i).getTrafficAreaDirection();
            trafficAreaJson.push_back(area);
        }
        jsonWritter["trafficArea"] = trafficAreaJson;
    }
    
    // controll line
    controlLineJson.clear();
    for (int i = 0; i < controlLine.size(); i++)
    {
        Point point;
        point.x = controlLine.at(i).x;
        point.y = controlLine.at(i).y;
        controlLineJson.push_back(point);
    }
    jsonWritter["controlLine"] = controlLineJson;

    // Enviroment rotate
    jsonWritter["enviromentRotating"] = alpha_slider;

    // Delete a file
    //std::ofstream ofs;
    //ofs.open(appConfigPathJson, std::ofstream::out | std::ofstream::trunc);
    //ofs.close();
    
    // write prettified JSON to another file
    std::ofstream outputStream(appConfigPathJson, std::ofstream::out | std::ofstream::trunc);
    if (outputStream.is_open())
    {
        outputStream << jsonWritter << std::endl;
        outputStream.close();
        std::cout << "APPLICATION [Application configuration settings was successfully saved]";
    }
    else {
        std::cout << "Can not open or read application configuration file: " << appConfigPathJson << std::endl;
    }
}

// load configurations from json configuration file
void loadConfiguration()
{
    // read a JSON file
    std::ifstream inputStream(appConfigPathJson);
    nlohmann::json jsonReader;

    if (inputStream.is_open())
    {
        inputStream >> jsonReader;
        inputStream.close();

        // trafficArea (struct)
        // |_ area
        //      |_ x (float)
        //      |_ y (float)
        // |_ direction (vector<int>)

        // monitored area
        if (jsonReader["monitoredArea"].size() > 0) {
            for (size_t i = 0; i < jsonReader["monitoredArea"]["area"].size(); i++)
            {
                cv::Point point;
                point.x = jsonReader["monitoredArea"]["area"].at(i)["x"];
                point.y = jsonReader["monitoredArea"]["area"].at(i)["y"];

                monitoredArea.addPointToMonitoredArea(point);
            }

            for (size_t i = 0; i < jsonReader["monitoredArea"]["direction"].size(); i++)
            {
                monitoredArea.setMonitoredAreaDirection(jsonReader["monitoredArea"]["direction"].at(i));
            }
            setMonitoredAreaDirection();
        }

        // no detecting area
        if (jsonReader["noDetectingArea"].size() > 0) {
            for (auto inputStream : jsonReader["noDetectingArea"])
            {
                noDetectingArea.push_back(cv::Point(inputStream["x"], inputStream["y"]));
            }
        }

        // unpermitted vehicle passage
        if (jsonReader["unpermittedVehiclePassageArea"].size() > 0) {
            for (auto inputStream : jsonReader["unpermittedVehiclePassageArea"])
            {
                unpermittedVehiclePassageArea.push_back(cv::Point(inputStream["x"], inputStream["y"]));
            }
        }

        // traffic light aArea
        if (jsonReader["trafficLightArea"].size() > 0) {
            for (auto inputStream : jsonReader["trafficLightArea"])
            {
                trafficLightArea.push_back(cv::Point(inputStream["x"], inputStream["y"]));
            }
        }

        // trafficArea (vector<struct>)
        // |_ area  (vector<Point>)
        //      |_ x (float)
        //      |_ y (float)
        // |_ direction (vector<int>)

        // traffic area
        if (jsonReader["trafficArea"].size() > 0) {
            for (auto inputStream : jsonReader["trafficArea"])
            {
                TrafficArea trafficAreaTemp;
                for (size_t i = 0; i < inputStream["area"].size(); i++)
                {
                    cv::Point point;
                    point.x = inputStream["area"].at(i)["x"];
                    point.y = inputStream["area"].at(i)["y"];
                    
                    trafficAreaTemp.addPointToTrafficArea(point);
                }
                trafficAreaTemp.setTrafficAreaDirection(inputStream["direction"]);
                trafficAreaTemp.setTrafficAreaIndex((inputStream["index"]));
                trafficArea.push_back(trafficAreaTemp);
            }
            setMonitoredAreaDirection();
        }

        // Control line
        if (jsonReader["controlLine"].size() > 0) {
            for (auto inputStream : jsonReader["controlLine"])
            {
                controlLine.push_back(cv::Point(inputStream["x"], inputStream["y"]));
            }
        }

        // Enviroment rotating
        alpha_slider = jsonReader["enviromentRotating"];

        //cameraCalibration.showCalibrationParameters();

        // Console output
        std::cout << "APPLICATION [Configuration settings was successfully loaded]" << std::endl;
    }
    else {
        std::cout << "APPLICATION [Can not open or read application configuration file: " << appConfigPathJson << "]" << std::endl;
    }
}


// add access control area
void addAccessControlArea(cv::Mat& frame)
{
    isDisplayedMainMenu = false;
    //isDisplayedTrafficAreaScenario = false;

    if (isSelectingAccessControlArea == true)
        isSelectingAccessControlArea = false;
    else
        isSelectingAccessControlArea = true;

    //cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "Acess control area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "- define access control area by mouse clicks or mouse click and move", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER or ESC", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);

    float alpha = 0.5;
    cv::Mat overlay;
    frame.copyTo(overlay);

    unpermittedVehiclePassageArea.clear();

    while (isSelectingAccessControlArea)
    {
        int key = cv::waitKey(1);
        // c
        if (key == 99) {
            std::cout << "CLEAR" << std::endl;
            unpermittedVehiclePassageArea.clear();
        }
        // enter/esc
        else if (key == 27 || key == 13) {
            isDisplayedMainMenu = false;
            isSelectingAccessControlArea = false;
            break;
        }

        else if (unpermittedVehiclePassageArea.size() > 1) {
            cv::fillPoly(frame, unpermittedVehiclePassageArea, cv::Scalar(100, 255, 100));
            for (size_t i = 0; i < unpermittedVehiclePassageArea.size(); i++)
            {
                cv::circle(frame, unpermittedVehiclePassageArea.at(i), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }
            addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
        }
        else {
            addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
        }
        //isSelectedRoiArea = true; 

        if (showFrames)
        {           
            jetsonImShow(kWinName, frame);
        }
    }
}

void addNoDetectingArea(cv::Mat& frame) {
    // cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "No detecting area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for define no detecting area press SPACE and then by mouse clicks", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for redefine no detecting area by mouse with drag and drop clicks", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for remove no detecting area or no detecting point press C", cv::Point(20, 145), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER", cv::Point(20, 180), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);

    float alpha = 0.5;
    cv::Mat overlay;
    frame.copyTo(overlay);

    bool isRemoveAreaWindowVisible = false;
    bool isCreateAreaWindowVisible = false;

    while (isDisplayedNoDetectingAreaScenario)
    {
        // accept active traffic area and continue to next traffic area
        int key = cv::waitKey(1);

        // key space
        if (key == 32) {
            if (isCreateAreaWindowVisible) {
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isCreateAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }
        // accept selected areas and exit to main screen
        else if (key == 27 || key == 13 || key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaScenario = false;
            isDisplayedTrafficAreaScenario = false;
            isDisplayedTrafficLightAreaScenario = false;
            isDisplayedNoDetectingAreaScenario = false;

            break;
        }
        // test for c press key
        else if (key == 99) {
            if (isRemoveAreaWindowVisible) {
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isRemoveAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }

        if (noDetectingArea.size() > 0 && indexOfLastActivePointInDetectingArea >= 0) {
            cv::circle(frame, noDetectingArea.at(indexOfLastActivePointInDetectingArea), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
        }

        if (noDetectingArea.size() > 0) {
            cv::fillPoly(frame, noDetectingArea, cv::Scalar(255,25,25));
            for (int i = 0; i < noDetectingArea.size(); i++)
            {
                cv::circle(frame, noDetectingArea.at(i), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }
        }

        // Create area window
        if (isCreateAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Create a new area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            case 0:
                noDetectingArea.clear();
                indexOfLastActivePointInDetectingArea = 0;
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 1:
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        if (isRemoveAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Remove the selected point", "Remove the entire area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
                // remove point from traffic area
            case 0:
                if (indexOfLastActivePointInDetectingArea != -1) {
                    noDetectingArea.erase(noDetectingArea.begin() + indexOfLastActivePointInDetectingArea);
                    indexOfLastActivePointInDetectingArea = -1;
                }
                indexOfActivePointInDetectingArea = -1;
                indexOfLastActivePointInDetectingArea = -1;
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
                // remove traffic area with all points
            case 1:
                if (noDetectingArea.size() > 0) {
                    noDetectingArea.clear();
                    indexOfActivePointInDetectingArea = -1;
                    indexOfLastActivePointInDetectingArea = -1;
                }
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 2:
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        //if (isRemoveAreaWindowVisible) {
        //    appWindow.drawWindow(frame, "Select one of the options", "Remove the entire area", "Close this window");

        //    switch (appWindow.getLastClickedButtonId())
        //    {
        //        // remove point from traffic area
        //    case 0:
        //        noDetectingArea.clear();
        //        indexOfLastActivePointInDetectingArea = -1;
        //        isRemoveAreaWindowVisible = false;
        //        break;
        //        // remove traffic area with all points
        //    case 1:
        //        isRemoveAreaWindowVisible = false;
        //        break;
        //    default:
        //        break;
        //    }
        //}

        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

        if (showFrames) {
            jetsonImShow(kWinName, frame);
        }
    }
}

//add traffic light area
void addTrafficLightArea(cv::Mat& frame)
{
    //cv::putText(frame, "Traffic light area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
    //cv::putText(frame, "- define traffic light area by mouse clicks or mouse click and move", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
    //cv::putText(frame, "- for accept selection, press ENTER or ESC", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "Traffic light area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for define traffic light area press SPACE and then by mouse clicks", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for select existed traffic light area use mouse click", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for redefine traffic light area by mouse with drag and drop clicks", cv::Point(20, 145), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for remove traffic light area press C", cv::Point(20, 180), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER", cv::Point(20, 215), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);

    float alpha = 0.5;
    cv::Mat overlay;
    frame.copyTo(overlay);

    bool isRemoveAreaWindowVisible = false;
    bool isCreateAreaWindowVisible = false;

    while (isDisplayedTrafficLightAreaScenario)
    {
        // accept active traffic area and continue to next traffic area
        int key = cv::waitKey(1);

        // key space
        if (key == 32) {
            if (isCreateAreaWindowVisible) {
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isCreateAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }
        // accept selected areas and exit to main screen
        else if (key == 27 || key == 13 || key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaScenario = false;
            isDisplayedTrafficAreaScenario = false;
            isDisplayedTrafficLightAreaScenario = false;
            break;
        }
        // test for c press key
        else if (key == 99) {
            if (isRemoveAreaWindowVisible) {
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isRemoveAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }

        if (trafficLightArea.size() > 0 && indexOfActivePointInTrafficLightArea >= 0) {
            cv::circle(frame, trafficLightArea.at(indexOfActivePointInTrafficLightArea), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
        }
        else if (trafficLightArea.size() > 0 && indexOfLastActivePointInTrafficLightArea >= 0) {
            cv::circle(frame, trafficLightArea.at(indexOfLastActivePointInTrafficLightArea), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
        }

        if (trafficLightArea.size() > 0) {
            if (trafficLightArea.size() >= 2) {
                cv::rectangle(frame, cv::Rect(trafficLightArea.at(0), trafficLightArea.at(1)), cv::Scalar(25, 180, 25), 4);
                cv::circle(frame, trafficLightArea.at(0), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
                cv::circle(frame, trafficLightArea.at(1), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }
            else if (trafficLightArea.size() == 1) {
                cv::circle(frame, trafficLightArea.at(0), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }                   
        }
        // Create area window
        if (isCreateAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Create a new area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            case 0:
                trafficLightArea.clear();
                indexOfActivePointInTrafficLightArea = 0;
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 1:
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        if (isRemoveAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Remove the entire area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
                // remove point from traffic area
            case 0:
                trafficLightArea.clear();
                indexOfActivePointInTrafficLightArea = -1;
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
                // remove traffic area with all points
            case 1:
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

        if (showFrames) {
            jetsonImShow(kWinName, frame);
        }
    }   
}

// draw all traffic areas
void drawAllTrafficAreas(cv::Mat& frame)
{
    float alpha = 0.75;
    cv::Mat overlay;
    frame.copyTo(overlay);
    if (trafficArea.size() != 0) {
        for (size_t i = 0; i < trafficArea.size(); i++)
        {
            cv::Scalar scalarColor = getScalarColor(i);
            cv::fillPoly(frame, trafficArea[i].getTrafficArea(), scalarColor);

            std::string direction;

            if (trafficArea[i].getIsSelectedTrafficAreaDirection() == true) {
                switch (trafficArea[i].getTrafficAreaDirection())
                {
                case TrafficArea::TrafficAreaDirection::leftTop:
                    direction = "lt";
                    break;
                case TrafficArea::TrafficAreaDirection::rightTop:
                    direction = "rt";
                    break;
                case TrafficArea::TrafficAreaDirection::leftBottom:
                    direction = "lb";
                    break;
                case TrafficArea::TrafficAreaDirection::rightBottom:
                    direction = "rb";
                    break;
                default:
                    direction = "";
                    break;
                }
                //cv::putText(frame, direction, trafficArea[i].getCenterOfTrafficArea(), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 255, 255), 2);
            }
            //cv::Point indexPoint = cv::Point(trafficArea[i].getCenterOfTrafficArea().x + 12, trafficArea[i].getCenterOfTrafficArea().y + 12);
            //cv::putText(frame, to_string(trafficArea[i].getTrafficAreaIndex()), indexPoint, cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 0.85, cv::LINE_AA);
        }
        //cv::drawContours(frame, trafficArea[indexOfActiveTrafficArea].getTrafficArea(), -1, cv::Scalar(0, 255, 0), 4, 1, cv::noArray());
        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
    }
}

void addControlLine(cv::Mat& frame)
{
    //////////controlLine.clear();

    //////////float alpha = 0.75;
    //////////cv::Mat overlay;
    //////////frame.copyTo(overlay);

    //////////while (isDisplayedControlLineScenario)
    //////////{
    //////////    int key = cv::waitKey(1);
    //////////    // c
    //////////    if (key == 99) {
    //////////        std::cout << "CLEAR" << std::endl;
    //////////        controlLine.clear();
    //////////    }
    //////////    // enter/esc
    //////////    if (key == 27 || key == 13) {
    //////////        isDisplayedMainMenu = false;
    //////////        isDisplayedControlLineScenario = false;
    //////////        break;
    //////////    }
    //////////    if (controlLine.size() == 1) {
    //////////        //std::cout << controlLine.size() << std::endl;
    //////////        cv::circle(frame, controlLine[0], 5, cv::Scalar(255, 0, 0), 5);
    //////////        //addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
    //////////    }
    //////////    else if (controlLine.size() == 2) {
    //////////        //std::cout << controlLine.size() << std::endl;
    //////////        cv::circle(frame, controlLine[1], 5, cv::Scalar(255, 0, 0), 5);
    //////////        cv::line(frame, controlLine[0], controlLine[1], cv::Scalar(255, 0, 0), 5);
    //////////        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
    //////////    }

    //////////    addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

    //////////    if (showFrames) {
    //////////        jetsonImShow(kWinName, frame);
    //////////    }
    //////////}

    // cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "Control line:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- define control line by mouse clicks or mouse click and move", cv::Point(10, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER or ESC", cv::Point(10, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);

    float alpha = 0.5;
    cv::Mat overlay;
    frame.copyTo(overlay);

    bool isRemoveAreaWindowVisible = false;
    bool isCreateAreaWindowVisible = false;

    while (isDisplayedControlLineScenario)
    {
        // accept active traffic area and continue to next traffic area
        int key = cv::waitKey(1);

        // key space
        if (key == 32) {
            if (isCreateAreaWindowVisible) {
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isCreateAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }
        // accept selected areas and exit to main screen
        else if (key == 27 || key == 13 || key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaScenario = false;
            isDisplayedTrafficAreaScenario = false;
            isDisplayedTrafficLightAreaScenario = false;
            isDisplayedControlLineScenario = false;
            break;
        }
        // test for c press key
        else if (key == 99) {
            if (isRemoveAreaWindowVisible) {
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isRemoveAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }

        if (controlLine.size() > 0 && indexOfActivePointInControlLine >= 0) {
            cv::circle(frame, controlLine.at(indexOfActivePointInControlLine), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
        }
        else if (controlLine.size() > 0 && indexOfLastActivePointInControlLine >= 0) {
            cv::circle(frame, controlLine.at(indexOfLastActivePointInControlLine), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
        }

        if (controlLine.size() == 1) {
            //std::cout << controlLine.size() << std::endl;
            cv::circle(frame, controlLine.at(0), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            //addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
        }
        else if (controlLine.size() == 2) {
            //std::cout << controlLine.size() << std::endl;
            cv::line(frame, controlLine[0], controlLine[1], cv::Scalar(255, 0, 0), circleRadiusInPixels / 2);
            cv::circle(frame, controlLine.at(0), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            cv::circle(frame, controlLine.at(1), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
        }

        /*if (controlLine.size() > 0) {
            if (controlLine.size() >= 2) {
                cv::rectangle(frame, cv::Rect(controlLine.at(0), controlLine.at(1)), cv::Scalar(25, 180, 25), 4);
                cv::circle(frame, controlLine.at(0), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
                cv::circle(frame, controlLine.at(1), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }
            else if (controlLine.size() == 1) {
                cv::circle(frame, controlLine.at(0), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }
        }*/
        // Create area window
        if (isCreateAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Create a new control line", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            case 0:
                controlLine.clear();
                indexOfActivePointInControlLine = 0;
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 1:
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        if (isRemoveAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Remove control line", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
                // remove point from traffic area
            case 0:
                controlLine.clear();
                indexOfActivePointInControlLine = -1;
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
                // remove traffic area with all points
            case 1:
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

        if (showFrames) {
            jetsonImShow(kWinName, frame);
        }
    }
}
bool isLinesInterected(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2){
//Point* intersection(Point o1, Point p1, Point p3, Point p4) {
    // Store the values for fast access and easy
    // equations-to-code conversion
    float x1 = o1.x, x2 = p1.x, x3 = o2.x, x4 = p2.x;
    float y1 = o1.y, y2 = p1.y, y3 = o2.y, y4 = p2.y;

    float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    std::cout << "d" << d << std::endl;
    // If d is zero, there is no intersection
    if (d == 0) 
        return false;
    else
        return true;
}

//bool isLinesInterected(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2)
//{
//    std::cout << "o1: " << o1 << std::endl;
//    std::cout << "o2: " << o2 << std::endl;
//    std::cout << "p1: " << p1 << std::endl;
//    std::cout << "p2: " << p2 << std::endl;
//    cv::Point2f x = o2 - o1;
//    cv::Point2f d1 = p1 - o1;
//    cv::Point2f d2 = p2 - o2;
//
//    std::cout << "x: " << x << std::endl;
//    std::cout << "d1: " << d1 << std::endl;
//    std::cout << "d2: " << d2 << std::endl;
//
//    float cross = d1.x * d2.y - d1.y * d2.x;
//    std::cout << "abs(cross): " << abs(cross) << std::endl;
//    if (abs(cross) < /*EPS*/1e-8)
//        return false;
//    return true;
//
//    //double t1 = (x.x * d2.y - x.y * d2.x) / cross;
//    //r = o1 + d1 * t1;
//    //return true;
//}
//
//void checkAndDrawControlLine(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene) {
//    if (controlLine.size() > 0) {
//
//        //vector<vector<cv::Point>> contours;
//        //cv::Mat binaryMask = cv::Mat(cv::Size(2048, 2448), CV_8UC1);
//        //cv::Mat binaryMask = cv::Mat(cv::Size(frame.cols, frame.rows), CV_8UC1);
//        bool isControlLineCrossed = false;
//
//        //cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(255, 0, 0), 12);
//        //cv::line(binaryMask, cv::Point(controlLine[0].x * 2.39062, controlLine[0].y * 2), cv::Point(controlLine[1].x * 2.39062, controlLine[1].y * 2), cv::Scalar(255, 0, 0), 60);
//        //cv::line(binaryMask, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(255, 0, 0), 60);
//        //cv::findContours(binaryMask, contours, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//
//        for (int i = 0; i < detectedObjectsInScene.size(); i++)
//        {            
//            // Defined which object name will be detected by control line
//            if (detectedObjectsInScene.at(i).getObjectName() == objectNameForControlLine) {
//
//                cv::circle(frame, detectedObjectsInScene.at(i).getObjectRoute().at(detectedObjectsInScene.at(i).getObjectRoute().size() - 1), 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
//            }
//            if (detectedObjectsInScene.at(i).getObjectName() == objectNameForControlLine && !std::count(detectedIndexes.begin(), detectedIndexes.end(), detectedObjectsInScene.at(i).getIndexOfObject())) {
//
//                std::vector<cv::Point2f> actualObjectRoute = detectedObjectsInScene.at(i).getObjectRoute();
//                for (int indexOfPoint = 0; indexOfPoint < detectedObjectsInScene.size()-1; indexOfPoint++)
//                {
//                    cv::Point2f firstPoint = actualObjectRoute.at(indexOfPoint);
//                    cv::Point2f secondPoint = actualObjectRoute.at(indexOfPoint+1);
//
//                    if (isLinesInterected(firstPoint, secondPoint, controlLine[0], controlLine[1]))
//                    {
//                        isControlLineCrossed = true;
//                        detectedIndexes.push_back(detectedObjectsInScene.at(i).getIndexOfObject());
//                        controlLineCounter++;
//                        break;
//                    }
//                }
//                //cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(0, 255, 0), 20);
//                //cv::Point tempPoint = detectedObjectsInScene.at(i).getObjectRoute().at(detectedObjectsInScene.at(i).getObjectRoute().size() - 1);
//
//                //if (contours.size() > 0) {
//                //    if (cv::pointPolygonTest(contours[contours.size() - 1], tempPoint, false) >= 0 && !std::count(detectedIndexes.begin(), detectedIndexes.end(), detectedObjectsInScene.at(i).getIndexOfObject())) {
//                //        isControlLineCrossed = true;
//                //        detectedIndexes.push_back(detectedObjectsInScene.at(i).getIndexOfObject());
//                //        controlLineCounter++;
//                //    }
//                //}
//            }
//        }
//
//        if (isControlLineCrossed == true) {
//            //cv::polylines(frame, contours, true, cv::Scalar(0, 255, 0), 4, 8);
//            cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(0, 255, 0), 20);
//        }
//        else {
//            //cv::polylines(frame, contours, true, cv::Scalar(0, 0, 255), 4, 8);
//            cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(255, 0, 0), 20);
//        }
//
//        //cv::resize(binaryMask, binaryMask, cv::Size(binaryMask.cols / 2, binaryMask.rows / 2));
//        //jetsonImShow("BG Mask", binaryMask);
//    }
//}

//Martinova verze
void checkAndDrawControlLine(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene, std::vector<int>& detectedIndexes) {
    if (controlLine.size() > 0) {
        vector<vector<cv::Point>> contours;
        //cv::Mat binaryMask = cv::Mat(cv::Size(2048, 2448), CV_8UC1);
        //cv::Mat binaryMask = cv::Mat(cv::Size(frame.cols, frame.rows), CV_8UC1);
        cv::Mat binaryMask(frame.cols, frame.rows, CV_8UC1, cv::Scalar(0));
        bool isControlLineCrossed = false;
        
        //cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(255, 0, 0), 12);
        //cv::line(binaryMask, cv::Point(controlLine[0].x * 2.39062, controlLine[0].y * 2), cv::Point(controlLine[1].x * 2.39062, controlLine[1].y * 2), cv::Scalar(255, 0, 0), 60);
        cv::line(binaryMask, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(255, 0, 0), 60);
        cv::findContours(binaryMask, contours, cv::noArray() , cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < detectedObjectsInScene.size(); i++)
        {
            // Defined which object name will be detected by control line
            if(std::find(counterLineJson.begin(), counterLineJson.end(), detectedObjectsInScene.at(i).getObjectName()) != counterLineJson.end()){
            //if (detectedObjectsInScene.at(i).getObjectName() == objectNameForControlLine) {
                cv::Point tempPoint = detectedObjectsInScene.at(i).getObjectRoute().at(detectedObjectsInScene.at(i).getObjectRoute().size() - 1);
            
                if (contours.size() > 0) { 
                    if (cv::pointPolygonTest(contours[contours.size() - 1], tempPoint, false) >= 0 && !std::count(detectedIndexes.begin(), detectedIndexes.end(), detectedObjectsInScene.at(i).getIndexOfObject())) {
                        isControlLineCrossed = true;
                        detectedIndexes.push_back(detectedObjectsInScene.at(i).getIndexOfObject());
                        counterLineNumOfObjects[detectedObjectsInScene.at(i).getObjectName()] += 1;
                    }
                }
            }
        }

        if (isControlLineCrossed == true) {
            //cv::polylines(frame, contours, true, cv::Scalar(0, 255, 0), 4, 8);
            cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(0, 255, 0), 20);
        }
        else {
            //cv::polylines(frame, contours, true, cv::Scalar(0, 0, 255), 4, 8);
            cv::line(frame, cv::Point(controlLine[0].x, controlLine[0].y), cv::Point(controlLine[1].x, controlLine[1].y), cv::Scalar(255, 0, 0), 20);
        }

        //cv::resize(binaryMask, binaryMask, cv::Size(binaryMask.cols / 4, binaryMask.rows / 4));
        //jetsonImShow("BG Mask", binaryMask);
    }
}
//predelane na vsechny objekty s filtrem
void checkIfBusIsLocatedInFrame(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene) {
    bool showBusRectangle = false;

    cv::Rect busDetectingArea(frame.cols * 1 / 6, frame.rows * 1 / 6, frame.cols - frame.cols * 1 / 3, frame.rows - frame.rows * 1 / 3);

    if (showBusRectangle) {
        cv::rectangle(frame, busDetectingArea, cv::Scalar(0, 255, 0), 4, 1, 0);
    }
    
    //cv::Point centerOfbox = (box.br() + box.tl()) * 0.5;
    //cv::circle(frame, centerOfbox, 5, cv::Scalar(0, 0, 255), 4);
    for (int i = 0; i < detectedObjectsInScene.size(); i++)
    {
        //if (detectedObjectsInScene.at(i).getObjectName() == "bus"){
        if (std::find(vehiclePassageJson.begin(), vehiclePassageJson.end(), detectedObjectsInScene[i].getObjectName()) != vehiclePassageJson.end()) {
            cv::Point detectedBusCenter;
            detectedBusCenter.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width / 2);
            detectedBusCenter.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height / 2);

            if (busDetectingArea.contains(detectedBusCenter)) {
                //auto utcTimeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                //std::stringstream utcTimeStampStringStream;
                //utcTimeStampStringStream << utcTimeStamp;
                //std::string pathToImage = std::string("/mnt/nvme/imagesToUpload/") + utcTimeStampStringStream.str();
                //cv::imwrite(pathToImage + ".jpg", frame);
                //string imageName = evenTypeHandler.saveImage(frame);
                isVehiclePassage = true;
                iteratorForResetVehiclePassage = 0;
                evenTypeHandler->sendVehiclePassage(frame, detectedObjectsInScene.at(i), barriers, trafficLight);
            }
        }
    }
}

// set monitored area direction
void setMonitoredAreaDirection() {
    // set monitored direction from traffic area
    if (trafficArea.size() > 0) {

        bool lt = false, rt = false, lb = false, rb = false;
        for (size_t j = 0; j < monitoredArea.getMonitoredAreaDirection().size(); j++)
        {
            if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::leftTop)
                lt = true;
            else if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::rightTop)
                rt = true;
            else if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::leftBottom)
                lb = true;
            else if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::rightBottom)
                rb = true;
        }

        for (size_t j = 0; j < trafficArea.size(); j++)
        {
            if (lt == true && trafficArea.at(j).getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::leftTop) {
                monitoredArea.setMonitoredAreaTrafficAreaDirectionIndexes(trafficArea.at(j).getTrafficAreaIndex());
            }
            else if (rt == true && trafficArea.at(j).getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::rightTop) {
                monitoredArea.setMonitoredAreaTrafficAreaDirectionIndexes(trafficArea.at(j).getTrafficAreaIndex());
            }
            else if (lb == true && trafficArea.at(j).getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::leftBottom) {
                monitoredArea.setMonitoredAreaTrafficAreaDirectionIndexes(trafficArea.at(j).getTrafficAreaIndex());
            }
            else if (rb == true && trafficArea.at(j).getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::rightBottom) {
                monitoredArea.setMonitoredAreaTrafficAreaDirectionIndexes(trafficArea.at(j).getTrafficAreaIndex());
            }
        }
    }
}

void drawOtherDetectedObjects(OutputFrame& outputFrame, string object, float confidence, cv::Rect box) {
    cv::Scalar objectRectColor;
    cv::Mat frame = outputFrame.frameForDisplay;
    if (object == "licensePlate")
    {
        cv::rectangle(frame, box, cv::Scalar(0, 255, 0));
    }

    if (object == "person") {
        objectRectColor = cv::Scalar(255, 145, 25);
    }
    else {
        objectRectColor = cv::Scalar(25, 225, 225);
    }

    std::string label = cv::format("%.2f", confidence);
    // traffic lights settings
    cv::Scalar trafficLightColor = cv::Scalar(255, 255, 255);
    if (object == "trafficLightRed" || object == "trafficLightRedYellow" || object == "trainTrafficLightRed") {
        trafficLightColor = cv::Scalar(0, 0, 255);

        TrafficLight trafficLightTemp;
        trafficLightTemp.setTrafficLightPosition(box);
        trafficLightTemp.setTrafficsLightColor(TrafficLight::restricted);
        outputFrame.trafficLights.push_back(trafficLightTemp);
    }
    else if (object == "trafficLightYellow" || object == "trafficLightRedYellow") {
        trafficLightColor = cv::Scalar(0, 125, 255);

        TrafficLight trafficLightTemp;
        trafficLightTemp.setTrafficLightPosition(box);
        trafficLightTemp.setTrafficsLightColor(TrafficLight::standBy);
        outputFrame.trafficLights.push_back(trafficLightTemp);
    }
    else if (object == "trafficLightGreen" || object == "trafficLightOff" || object == "trainTrafficLightWhite" || object == "trainTrafficLightOff") {
        trafficLightColor = cv::Scalar(0, 255, 0);

        TrafficLight trafficLightTemp;
        trafficLightTemp.setTrafficLightPosition(box);
        trafficLightTemp.setTrafficsLightColor(TrafficLight::ready);
        outputFrame.trafficLights.push_back(trafficLightTemp);
    }
    label = object + ": " + label;
    cv::putText(frame, label, cv::Point(box.x - 5, box.y - 5), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, trafficLightColor, 1.25, cv::LINE_AA);
    //cv::putText(frame, label, cv::Point(box.x - 5, box.y - 5), cv::FONT_HERSHEY_DUPLEX, 1, trafficLightColor, 1, cv::LINE_AA);
}

// add monitored area
void addMonitoredArea(cv::Mat& frame)
{
    //cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "Monitored area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for define monitored area press SPACE and then by mouse clicks", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for redefine monitored area by mouse with drag and drop clicks", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for remove monitored area or monitored area point press C", cv::Point(20, 145), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER", cv::Point(20, 180), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);

    float alpha = 0.5;
    cv::Mat overlay;
    frame.copyTo(overlay);
    //roiRectangle = selectROI(frame, true, false);

    bool isRemoveAreaWindowVisible = false;
    bool isCreateAreaWindowVisible = false;

    char TrackbarName[50];
    std::string trackBarWindow = "Track bar";

    cv::namedWindow(trackBarWindow, cv::WINDOW_NORMAL);

    sprintf(TrackbarName, "Rotation:", alpha_slider_max);
    //cv::createTrackbar(TrackbarName, kWinName, &alpha_slider, alpha_slider_max, on_trackbar);

    cv::createTrackbar(TrackbarName, trackBarWindow, &alpha_slider, alpha_slider_max, on_trackbar);

    while (isDisplayedMonitoredAreaScenario)
    {
        alpha_slider = cv::getTrackbarPos(TrackbarName, trackBarWindow);

        // accept active traffic area and continue to next traffic area
        int key = cv::waitKey(1);

        // key space
        if (key == 32) {
            if (isCreateAreaWindowVisible) {
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isCreateAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }
        // accept selected areas and exit to main screen
        else if (key == 27 || key == 13 || key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaScenario = false;
            isDisplayedTrafficAreaScenario = false;
            break;
        }
        // test for c press key
        else if (key == 99) {
            if (isRemoveAreaWindowVisible) {
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isRemoveAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }

        /*if (showFrames)
            cv::setMouseCallback(kWinName, mouseEvent, NULL);*/
        if (monitoredArea.getSizeOfMonitoredArea() != 0) {
            cv::fillPoly(frame, monitoredArea.getMonitoredArea(), cv::Scalar(100, 255, 100));
            cv::polylines(frame, monitoredArea.getMonitoredArea(), true, cv::Scalar(50, 200, 50), 4);

            if (monitoredArea.getMonitoredArea().size() > 0 && indexOfLastActivePointInMonitoredArea >= 0) {
                cv::circle(frame, monitoredArea.getMonitoredArea().at(indexOfLastActivePointInMonitoredArea), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
            }

            for (size_t i = 0; i < monitoredArea.getSizeOfMonitoredArea(); i++)
            {
                cv::circle(frame, monitoredArea.getMonitoredArea().at(i), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);

                if (monitoredArea.getMonitoredArea().size() >= 4) {
                    // get moments
                    cv::Moments mu = moments(monitoredArea.getMonitoredArea(), false);
                    // get center of moments
                    cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

                    cv::Rect boundingRect = cv::boundingRect(monitoredArea.getMonitoredArea());

                    cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05)), cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05)));
                    //cv::rectangle(frame, tmpRect, cv::Scalar(255, 0, 0), 8);
                    bool lt = false, rt = false, lb = false, rb = false;
                    for (size_t j = 0; j < monitoredArea.getMonitoredAreaDirection().size(); j++)
                    {
                        if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::MonitoredAreaDirection::leftTop) {
                            lt = true;
                        }
                        else if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::MonitoredAreaDirection::rightTop) {
                            rt = true;
                        }
                        else if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::MonitoredAreaDirection::leftBottom) {
                            lb = true;
                        }
                        else if (monitoredArea.getMonitoredAreaDirection().at(j) == MonitoredArea::MonitoredAreaDirection::rightBottom) {
                            rb = true;
                        }
                    }

                    // LT
                    cv::Point2f ltPoint = rotatePoint(cv::Point(tmpRect.x, tmpRect.y), cMu, (alpha_slider) * (M_PI / 180));
                    if (lt)
                        cv::arrowedLine(frame, cMu, ltPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, ltPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);

                    // RT
                    cv::Point2f rtPoint = rotatePoint(cv::Point(tmpRect.x + tmpRect.width, tmpRect.y), cMu, (alpha_slider) * (M_PI / 180));
                    if (rt)
                        cv::arrowedLine(frame, cMu, rtPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, rtPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);

                    // LB
                    cv::Point2f lbPoint = rotatePoint(cv::Point(tmpRect.x, (tmpRect.y + tmpRect.height)), cMu, (alpha_slider) * (M_PI / 180));
                    if (lb)
                        cv::arrowedLine(frame, cMu, lbPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, lbPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);

                    // RB
                    cv::Point2f rbPoint = rotatePoint(cv::Point(tmpRect.x + tmpRect.width, (tmpRect.y + tmpRect.height)), cMu, (alpha_slider) * (M_PI / 180));
                    if (rb)
                        cv::arrowedLine(frame, cMu, rbPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, rbPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);
                }
            }
        }

        // Create area window
        if (isCreateAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Create a new area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            case 0:
                monitoredArea.removeMonitoredArea();
                indexOfLastActivePointInMonitoredArea = 0;
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 1:
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        if (isRemoveAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Remove the selected point", "Remove the entire area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            // remove point from traffic area
            case 0:
                if (indexOfLastActivePointInMonitoredArea != -1) {
                    monitoredArea.removePointInMonitoredArea(indexOfLastActivePointInMonitoredArea);
                    indexOfLastActivePointInMonitoredArea = -1;
                }
                indexOfActivePointInMonitoringArea = -1;
                indexOfLastActivePointInMonitoredArea = -1;
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            // remove traffic area with all points
            case 1:
                if (monitoredArea.getSizeOfMonitoredArea() > 0) {
                    monitoredArea.removeMonitoredArea();
                    indexOfActivePointInMonitoringArea = -1;
                    indexOfLastActivePointInMonitoredArea = -1;
                }
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 2:
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

        if (showFrames) {
            jetsonImShow(kWinName, frame);
        }     
    }
    setMonitoredAreaDirection();
    //isDisplayedMonitoredAreaScenario = false;
}

// add unpermitted passage area
void addUnpermittedVehiclePassage(cv::Mat& frame)
{
    //cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 2.0, cv::LINE_AA);
    cv::putText(frame, "Unpermitted vehicle passage area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for define unpermitted vehicle paasage area press SPACE and then by mouse clicks", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for redefine unpermitted vehicle paasage area by mouse with drag and drop clicks", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for remove unpermitted vehicle paasage area or unpermitted vehicle paasage area point press C", cv::Point(20, 145), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER", cv::Point(20, 180), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);

    float alpha = 0.5;
    cv::Mat overlay;
    frame.copyTo(overlay);
    //roiRectangle = selectROI(frame, true, false);

    bool isRemoveAreaWindowVisible = false;
    bool isCreateAreaWindowVisible = false;

    while (isDisplayedUnpermittedVehiclePassageScenario)
    {
        // accept active traffic area and continue to next traffic area
        int key = cv::waitKey(1);

        // key space
        if (key == 32) {
            if (isCreateAreaWindowVisible) {
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isCreateAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }
        // accept selected areas and exit to main screen
        else if (key == 27 || key == 13 || key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaScenario = false;
            isDisplayedUnpermittedVehiclePassageScenario = false;
            isDisplayedTrafficAreaScenario = false;
            break;
        }
        // test for c press key
        else if (key == 99) {
            if (isRemoveAreaWindowVisible) {
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isRemoveAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }

        /*if (showFrames)
            cv::setMouseCallback(kWinName, mouseEvent, NULL);*/
        if (unpermittedVehiclePassageArea.size() != 0) {
            cv::fillPoly(frame, unpermittedVehiclePassageArea, cv::Scalar(100, 255, 100));
            cv::polylines(frame, unpermittedVehiclePassageArea, true, cv::Scalar(50, 200, 50), 4);

            if (unpermittedVehiclePassageArea.size() > 0 && indexOfLastActivePointInUnpermittedVehiclePassageArea >= 0) {
                cv::circle(frame, unpermittedVehiclePassageArea.at(indexOfLastActivePointInUnpermittedVehiclePassageArea), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
            }

            for (size_t i = 0; i < unpermittedVehiclePassageArea.size(); i++)
            {
                cv::circle(frame, unpermittedVehiclePassageArea.at(i), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);
            }
        }

        // Create area window
        if (isCreateAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Create a new area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            case 0:
                unpermittedVehiclePassageArea.clear();
                indexOfLastActivePointInUnpermittedVehiclePassageArea = 0;
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 1:
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        if (isRemoveAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Remove the selected point", "Remove the entire area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
                // remove point from traffic area
            case 0:
                if (indexOfLastActivePointInUnpermittedVehiclePassageArea != -1) {
                    unpermittedVehiclePassageArea.erase(unpermittedVehiclePassageArea.begin() + indexOfLastActivePointInUnpermittedVehiclePassageArea);
                    indexOfLastActivePointInUnpermittedVehiclePassageArea = -1;
                }
                indexOfActivePointInMonitoringArea = -1;
                indexOfLastActivePointInUnpermittedVehiclePassageArea = -1;
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
                // remove traffic area with all points
            case 1:
                if (unpermittedVehiclePassageArea.size() > 0) {
                    unpermittedVehiclePassageArea.clear();
                    indexOfActivePointInUnpermittedVehiclePassageArea = -1;
                    indexOfLastActivePointInUnpermittedVehiclePassageArea = -1;
                }
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 2:
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);

        if (showFrames) {
            jetsonImShow(kWinName, frame);
        }
    }
}

// draw monitored area
void drawMonitoredArea(cv::Mat& frame) {
    
    //// get moments
    //cv::Moments mu = cv::moments(monitoredArea.getMonitoredArea(), false);
    //// get center of moments
    //cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

    //cv::Rect boundingRect = cv::boundingRect(monitoredArea.getMonitoredArea());

    //cv::Point2f ltPoint = cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05));
    //cv::Point2f rtPoint = cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05));
    //cv::Point2f lbPoint = cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05));
    //cv::Point2f rbPoint = cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05));

    //ltPoint = rotatePoint(ltPoint, cMu, (alpha_slider) * (M_PI / 180));
    //rtPoint = rotatePoint(rtPoint, cMu, (alpha_slider) * (M_PI / 180));
    //lbPoint = rotatePoint(lbPoint, cMu, (alpha_slider) * (M_PI / 180));
    //rbPoint = rotatePoint(rbPoint, cMu, (alpha_slider) * (M_PI / 180));

    //cv::circle(frame, ltPoint, 10, cv::Scalar(0, 0, 255), 4);
    //cv::circle(frame, rtPoint, 10, cv::Scalar(0, 0, 255), 4);
    //cv::circle(frame, lbPoint, 10, cv::Scalar(0, 0, 255), 4);
    //cv::circle(frame, rbPoint, 10, cv::Scalar(0, 0, 255), 4);

    ////cv::Rect tmpRect(ltPoint, rbPoint);
    //std::vector<cv::Point> rotatedRectPoints = { ltPoint, rtPoint, rbPoint, lbPoint };
    //cv::polylines(frame, rotatedRectPoints, true, cv::Scalar(255, 0, 0), 4);

    ////cv::rectangle(frame, tmpRect, cv::Scalar(255, 255, 0), 4, 8);

    //////////////////////////////////////

    // get moments
    //cv::Moments mu = cv::moments(monitoredArea.getMonitoredArea(), false);
    //// get center of moments
    //cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

    //cv::Rect boundingRect = cv::boundingRect(monitoredArea.getMonitoredArea());

    //cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.1), cMu.y - (boundingRect.width * 0.1)), cv::Point(cMu.x + (boundingRect.width * 0.1), cMu.y + (boundingRect.width * 0.1)));

    //// LT
    //cv::Rect lt(cv::Point(tmpRect.x, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
    //// RT
    //cv::Rect rt(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
    //// LB
    //cv::Rect lb(cv::Point(tmpRect.x, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
    //// RB
    //cv::Rect rb(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));

    //std::vector<cv::Point> ltRect = { cv::Point(lt.x, lt.y), cv::Point(lt.x + lt.width, lt.y), cv::Point(lt.x + lt.width, lt.y + lt.height), cv::Point(lt.x, lt.y + lt.height) };
    //std::vector<cv::Point> rtRect = { cv::Point(rt.x, rt.y), cv::Point(rt.x + rt.width, rt.y), cv::Point(rt.x + rt.width, rt.y + rt.height), cv::Point(rt.x, rt.y + rt.height) };
    //std::vector<cv::Point> rbRect = { cv::Point(rb.x, rb.y), cv::Point(rb.x + rb.width, rb.y), cv::Point(rb.x + rb.width, rb.y + rb.height), cv::Point(rb.x, rb.y + rb.height) };
    //std::vector<cv::Point> lbRect = { cv::Point(lb.x, lb.y), cv::Point(lb.x + lb.width, lb.y), cv::Point(lb.x + lb.width, lb.y + lb.height), cv::Point(lb.x, lb.y + lb.height) };

    //// LT
    //ltRect[0] = rotatePoint(ltRect[0], cMu, (alpha_slider) * (M_PI / 180));
    //ltRect[1] = rotatePoint(ltRect[1], cMu, (alpha_slider) * (M_PI / 180));
    //ltRect[2] = rotatePoint(ltRect[2], cMu, (alpha_slider) * (M_PI / 180));
    //ltRect[3] = rotatePoint(ltRect[3], cMu, (alpha_slider) * (M_PI / 180));

    //// RT
    //rtRect[0] = rotatePoint(rtRect[0], cMu, (alpha_slider) * (M_PI / 180));
    //rtRect[1] = rotatePoint(rtRect[1], cMu, (alpha_slider) * (M_PI / 180));
    //rtRect[2] = rotatePoint(rtRect[2], cMu, (alpha_slider) * (M_PI / 180));
    //rtRect[3] = rotatePoint(rtRect[3], cMu, (alpha_slider) * (M_PI / 180));

    //// RB
    //rbRect[0] = rotatePoint(rbRect[0], cMu, (alpha_slider) * (M_PI / 180));
    //rbRect[1] = rotatePoint(rbRect[1], cMu, (alpha_slider) * (M_PI / 180));
    //rbRect[2] = rotatePoint(rbRect[2], cMu, (alpha_slider) * (M_PI / 180));
    //rbRect[3] = rotatePoint(rbRect[3], cMu, (alpha_slider) * (M_PI / 180));

    //// LB
    //lbRect[0] = rotatePoint(lbRect[0], cMu, (alpha_slider) * (M_PI / 180));
    //lbRect[1] = rotatePoint(lbRect[1], cMu, (alpha_slider) * (M_PI / 180));
    //lbRect[2] = rotatePoint(lbRect[2], cMu, (alpha_slider) * (M_PI / 180));
    //lbRect[3] = rotatePoint(lbRect[3], cMu, (alpha_slider) * (M_PI / 180));

    //cv::polylines(frame, ltRect, true, cv::Scalar(255, 0, 0), 4);
    //cv::polylines(frame, rtRect, true, cv::Scalar(255, 0, 0), 4);
    //cv::polylines(frame, rbRect, true, cv::Scalar(255, 0, 0), 4);
    //cv::polylines(frame, lbRect, true, cv::Scalar(255, 0, 0), 4);

    if (monitoredArea.getSizeOfMonitoredArea() > 0) {

        cv::Mat overlay;
        overlay = frame.clone();

        float alpha = 0.75;

        //alpha = 0.65;
        // display monitored area
        if (monitoredArea.isMonitoredAreaRestricted() == true) {
            cv::fillPoly(frame, monitoredArea.getMonitoredArea(), cv::Scalar(0, 0, 255));
            cv::polylines(frame, monitoredArea.getMonitoredArea(), true, cv::Scalar(25, 25, 180), 4);
            addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
        }
        else {
            cv::fillPoly(frame, monitoredArea.getMonitoredArea(), cv::Scalar(0, 255, 0));
            cv::polylines(frame, monitoredArea.getMonitoredArea(), true, cv::Scalar(25, 180, 25), 4);
            addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
        }
    }
}

// add traffic area
void addTrafficArea(cv::Mat& frame)
{
    cv::putText(frame, "Traffic area:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for define traffic area press SPACE and then by mouse clicks", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for select existed traffic area use mouse click", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for redefine traffic area by mouse with drag and drop clicks", cv::Point(20, 145), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for remove traffic area or traffic area point press C", cv::Point(20, 180), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    cv::putText(frame, "- for accept selection, press ENTER", cv::Point(20, 215), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
    //cv::putText(frame, "- for exit from help menu, press H", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);

    // main info message in the bottom of window
    //cv::putText(frame, "Press space for accept selection", cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);

    float alpha = 0.2;
    cv::Mat overlay;
    frame.copyTo(overlay);
    // //roiRectangle = selectROI(frame, true, false);

    bool isRemoveAreaWindowVisible = false;
    bool isCreateAreaWindowVisible = false;

    char TrackbarName[50];
    std::string trackBarWindow = "Track bar";

    cv::namedWindow(trackBarWindow, cv::WINDOW_NORMAL);

    sprintf(TrackbarName, "Rotation:", alpha_slider_max);
    //cv::createTrackbar(TrackbarName, kWinName, &alpha_slider, alpha_slider_max, on_trackbar);

    cv::createTrackbar(TrackbarName, trackBarWindow, &alpha_slider, alpha_slider_max, on_trackbar);
    //cv::setTrackbarPos(TrackbarName, kWinName, alpha_slider);

    //cv::resizeWindow(trackBarWindow, 1024, 20);
    
    while (isDisplayedTrafficAreaScenario)
    {       
        alpha_slider = cv::getTrackbarPos(TrackbarName, trackBarWindow);

        // 1 stupen = pi / 180
        // 90 stupnov = 90 * (pi / 180)
        //cv::Rect arec = cv::Rect(150, 150, 200, 200);
        //cv::rectangle(frame, arec, cv::Scalar(255, 0, 0), 4);
        //cv::arrowedLine(frame, (arec.br() + arec.tl()) * 0.5, cv::Point(arec.x + arec.width, arec.y), cv::Scalar(0, 0, 255), 4);
        //cv::Point2f aaaaRot = rotatePoint(cv::Point(arec.x + arec.width, arec.y), (arec.br() + arec.tl()) * 0.5, alpha_slider * (M_PI / 180));
        //cv::arrowedLine(frame, (arec.br() + arec.tl()) * 0.5, aaaaRot, cv::Scalar(0, 0, 255), 4);

        // accept active traffic area and continue to next traffic area
        int key = cv::waitKey(1);

        // key space
        if (key == 32) {
            if (isCreateAreaWindowVisible) {
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isCreateAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
            indexOfActiveTrafficArea++;
        }
        // accept selected areas and exit to main screen
        else if (key == 27 || key == 13 || key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaScenario = false;
            isDisplayedTrafficAreaScenario = false;
            break;
        }
        // test for c press key
        else if (key == 99) {
            if (isRemoveAreaWindowVisible) {
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
            }
            else {
                isRemoveAreaWindowVisible = true;
                appWindow.showAppWindow();
            }
        }

        if (indexOfActiveTrafficArea >= 0 && trafficArea.size() > indexOfActiveTrafficArea) {
            cv::polylines(frame, trafficArea[indexOfActiveTrafficArea].getTrafficArea(), true, cv::Scalar(64, 255, 64), 4);
            if (indexOfLastActivePointInTrafficArea >= 0) {
                cv::circle(frame, trafficArea[indexOfActiveTrafficArea].getTrafficArea().at(indexOfLastActivePointInTrafficArea), circleRadiusInPixels + 4, cv::Scalar(0, 255, 0), -1);
            }
        }

        //if (trafficArea.size() > indexOfActiveTrafficArea) {
        for (size_t i = 0; i < trafficArea.size(); i++)
        {
            cv::Scalar scalarColor = getScalarColor(i);
            cv::fillPoly(frame, trafficArea[i].getTrafficArea(), scalarColor);
            for (size_t j = 0; j < trafficArea[i].getTrafficArea().size(); j++)
            {
                cv::circle(frame, trafficArea[i].getTrafficArea().at(j), circleRadiusInPixels, cv::Scalar(255, 0, 0), -1);

                if (trafficArea[i].getTrafficArea().size() >= 4) {
                    // get moments
                    cv::Moments mu = moments(trafficArea[i].getTrafficArea(), false);
                    // get center of moments
                    cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

                    cv::Rect boundingRect = cv::boundingRect(trafficArea[i].getTrafficArea());

                    cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05)), cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05)));
                    //cv::rectangle(frame, tmpRect, cv::Scalar(255, 0, 0), 8);

                    // LT
                    cv::Point2f ltPoint = rotatePoint(cv::Point(tmpRect.x, tmpRect.y), cMu, (alpha_slider) * (M_PI / 180));
                    if(trafficArea[i].getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::leftTop)
                        cv::arrowedLine(frame, cMu, ltPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, ltPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);
                    cv::Rect lt(cv::Point(tmpRect.x, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
                    //cv::rectangle(frame, lt, cv::Scalar(255, 0, 0), 2);
                    // RT
                    // cv::Point2f aaaaRot = rotatePoint(cv::Point(arec.x + arec.width, arec.y), (arec.br() + arec.tl()) * 0.5, alpha_slider * (M_PI / 180));
                    cv::Point2f rtPoint = rotatePoint(cv::Point(tmpRect.x + tmpRect.width, tmpRect.y), cMu, (alpha_slider) * (M_PI / 180));
                    if (trafficArea[i].getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::rightTop)
                        cv::arrowedLine(frame, cMu, rtPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, rtPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);
                    cv::Rect rt(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
                    //cv::rectangle(frame, rt, cv::Scalar(255, 0, 0), 2);
                    // LB
                    cv::Point2f lbPoint = rotatePoint(cv::Point(tmpRect.x, (tmpRect.y + tmpRect.height)), cMu, (alpha_slider) * (M_PI / 180));
                    if (trafficArea[i].getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::leftBottom)
                        cv::arrowedLine(frame, cMu, lbPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, lbPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);
                    cv::Rect lb(cv::Point(tmpRect.x, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
                    //cv::rectangle(frame, lb, cv::Scalar(255, 0, 0), 2);
                    // RB
                    cv::Point2f rbPoint = rotatePoint(cv::Point(tmpRect.x + tmpRect.width, (tmpRect.y + tmpRect.height)), cMu, (alpha_slider) * (M_PI / 180));
                    if (trafficArea[i].getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::rightBottom)
                        cv::arrowedLine(frame, cMu, rbPoint, cv::Scalar(0, 200, 0), 8, 4, 0, 0.5);
                    else
                        cv::arrowedLine(frame, cMu, rbPoint, cv::Scalar(255, 255, 255), 8, 4, 0, 0.5);
                    cv::Rect rb(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
                    //cv::rectangle(frame, rb, cv::Scalar(255, 0, 0), 2);  

                    // show traffic area index
                    cv::Point indexPoint = cv::Point((tmpRect.x + tmpRect.width) + 12, (tmpRect.y + tmpRect.height) + 12);
                    cv::putText(frame, to_string(trafficArea[i].getTrafficAreaIndex()), indexPoint, cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 0.85, cv::LINE_AA);
                }
            }

            // get moments
            cv::Moments mu = moments(trafficArea[i].getTrafficArea(), false);
            // get center of moments
            cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

            cv::Rect boundingRect = cv::boundingRect(trafficArea[i].getTrafficArea());

            cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05)), cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05)));
            
            // show traffic area index
            //cv::putText(frame, to_string(trafficArea[i].getTrafficAreaIndex()), cv::Point((tmpRect.x + tmpRect.width) + ((tmpRect.x + tmpRect.width) / 100) * 0.1, (tmpRect.y + tmpRect.height) + ((tmpRect.y + tmpRect.height) / 100) * 0.1), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 0.85);
            //cv::Point indexPoint = cv::Point(trafficArea[i].getCenterOfTrafficArea().x + 12, trafficArea[i].getCenterOfTrafficArea().y + 12);
            //cv::putText(frame, to_string(trafficArea[i].getTrafficAreaIndex()), indexPoint, cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 0.85, cv::LINE_AA);
        }

        // Create area window
        if (isCreateAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Create a new area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            case 0:
                if (trafficArea.size() == 0) {
                    indexOfActiveTrafficArea = 0;
                }
                else {
                    indexOfActiveTrafficArea = trafficArea.size();
                }               
                indexOfLastActivePointInTrafficArea = -1;
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 1:
                isCreateAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        // Remove area window
        if (isRemoveAreaWindowVisible) {
            appWindow.drawWindow(frame, "Select one of the options", "Remove the selected point", "Remove the entire area", "Close this window");

            switch (appWindow.getLastClickedButtonId())
            {
            // remove point from traffic area
            case 0:
                if (indexOfLastActivePointInTrafficArea != -1) {
                    if (trafficArea[indexOfActiveTrafficArea].getTrafficArea().size() > 0) {

                        trafficArea[indexOfActiveTrafficArea].removePointInTrafficArea(indexOfLastActivePointInTrafficArea);

                        if (trafficArea[indexOfActiveTrafficArea].getTrafficArea().size() == 0) {
                            trafficArea[indexOfActiveTrafficArea].removeTrafficArea();
                            trafficArea.erase(trafficArea.begin() + indexOfActiveTrafficArea);
                            indexOfActiveTrafficArea = -1;
                        }

                        indexOfLastActivePointInTrafficArea = -1;
                    }
                }
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            // remove traffic area with all points
            case 1:
                // if is selected active traffic area
                if (indexOfActiveTrafficArea != -1) {
                    if (trafficArea[indexOfActiveTrafficArea].getTrafficArea().size() > 0) {
                        //trafficArea[indexOfActiveTrafficArea].getTrafficArea().clear();
                        trafficArea[indexOfActiveTrafficArea].removeTrafficArea();
                        trafficArea.erase(trafficArea.begin() + indexOfActiveTrafficArea);

                        indexOfActiveTrafficArea = -1;
                        indexOfLastActivePointInTrafficArea = -1;
                    }
                }

                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            case 2:
                isRemoveAreaWindowVisible = false;
                appWindow.hideAppWindow();
                break;
            default:
                break;
            }
        }

        addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
            
        if (showFrames) {
            jetsonImShow(kWinName, frame);
        }
    }
    cv::destroyWindow(trackBarWindow);
    setMonitoredAreaDirection();
}

// check if monitored area contains any object
bool isContainsMonitoredAreaAnyDetectedObject(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene) {
    if (monitoredArea.getSizeOfMonitoredArea() > 1 && detectedObjectsInScene.size() > 0) {
        for (int i = 0; i < detectedObjectsInScene.size(); i++)
        {
            //left top corner of detected object
            cv::Point detectedObjectLT;
            //right top corner of detected object
            cv::Point detectedObjectRT;
            //left bottom corner of detected object
            cv::Point detectedObjectLB;
            //right bottom corner of detected object
            cv::Point detectedObjectRB;

            detectedObjectLT.x = detectedObjectsInScene.at(i).getObjectRect().x;
            detectedObjectLT.y = detectedObjectsInScene.at(i).getObjectRect().y;

            detectedObjectRT.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width);
            detectedObjectRT.y = detectedObjectsInScene.at(i).getObjectRect().y;

            detectedObjectLB.x = detectedObjectsInScene.at(i).getObjectRect().x;
            detectedObjectLB.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height);

            detectedObjectRB.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width);
            detectedObjectRB.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height);

            //yyyyyyyyyy
            //cv::circle(frame, detectedObjectLT, 40, cv::Scalar(255, 0, 0), 4);
            //cv::circle(frame, detectedObjectRT, 40, cv::Scalar(255, 0, 0), 4);
            //cv::circle(frame, detectedObjectLB, 40, cv::Scalar(255, 0, 0), 4);
            //cv::circle(frame, detectedObjectRB, 40, cv::Scalar(255, 0, 0), 4);

            if (pointPolygonTest(monitoredArea.getMonitoredArea(), detectedObjectLT, false) > 0)
                return true;
            if (pointPolygonTest(monitoredArea.getMonitoredArea(), detectedObjectRT, false) > 0)
                return true;
            if (pointPolygonTest(monitoredArea.getMonitoredArea(), detectedObjectLB, false) > 0)
                return true;
            if (pointPolygonTest(monitoredArea.getMonitoredArea(), detectedObjectRB, false) > 0)
                return true;
        }
        return false;
    }
}

// check monitored area 
void monitoredAreaCheck(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene) {
    if (monitoredArea.getMonitoredArea().size() > 1 && detectedObjectsInScene.size() > 0 && monitoredArea.getStateInMonitoredArea() == MonitoredArea::MonitoredAreaState::restricted) {

        //for (size_t i = 0; i < detectedObjectsInScene.size(); i++)
        //{
        //    cv::Point detectedObjectCenter;
        //    detectedObjectCenter.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width / 2);
        //    detectedObjectCenter.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height / 2);

        //    cv::circle(frame, detectedObjectCenter, 50, cv::Scalar(0, 0, 255), 30);
        //}

        for (int i = 0; i < detectedObjectsInScene.size(); i++)
        {
            if (std::find(redLightViolationJson.begin(), redLightViolationJson.end(), detectedObjectsInScene[i].getObjectName()) != redLightViolationJson.end()) {
                //std::cout << "redLightViolationJson: " << detectedObjectsInScene[i].getObjectName() << std::endl;
                cv::Point detectedObjectCenter;
                detectedObjectCenter.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width / 2);
                detectedObjectCenter.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height / 2);

                detectedObjectCenter.y = detectedObjectCenter.y + (((detectedObjectsInScene.at(i).getObjectRect().height / 3)));

                for (size_t j = 0; j < monitoredArea.getMonitoredAreaTrafficAreaDirectionIndexes().size(); j++)
                {
                    if (pointPolygonTest(monitoredArea.getMonitoredArea(), detectedObjectCenter, false) > 0) {

                        if (detectedObjectsInScene.at(i).getIndexOfTrafficArea() == monitoredArea.getMonitoredAreaTrafficAreaDirectionIndexes().at(j)) {
                            if (monitoredArea.getDeleyAfterRestricted() > monitoredArea.allowedDelayAfterRestricted)
                            {
                                evenTypeHandler->sendEnterToMonitoredArea(frame, detectedObjectsInScene.at(i), detectedObjectsInScene.at(i).getIndexOfTrafficArea(), monitoredArea.getDeleyAfterRestricted(), barriers, trafficLight);
                                isNormalState = false;
                                iteratorForResetNormalStateWarning = 0;
                            }
                        }

                    }
                }
            }
        }
    }
}

// check object movement in traffic area
void checkObjectMovementInTrafficArea(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene)
{    
    //int indexOfObjectDrivingInOppositeDirection = -1;
    if (detectedObjectsInScene.size() > 0) {        
        for (size_t i = 0; i < detectedObjectsInScene.size(); i++)
        {            
            TrafficArea::TrafficAreaDirection objectDirection = detectedObjectsInScene.at(i).getDirectionOfMovement();
            cv::Point detectedObjectCenter;
            detectedObjectCenter.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width / 2);
            detectedObjectCenter.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height / 2);
            // centroid offset in y direction
            //detectedObjectCenter.y = detectedObjectCenter.y + ((detectedObjectsInScene.at(i).getObjectRect().height / 2) * 1 / 3);
            detectedObjectCenter.y = detectedObjectCenter.y + (((detectedObjectsInScene.at(i).getObjectRect().height / 3)));
            //cv::circle(frame, detectedObjectCenter, 10, cv::Scalar(255, 255, 0), 2);

            if (trafficArea.size() > 0) {
                for (size_t j = 0; j < trafficArea.size(); j++)
                {
                    // set object traffic area id
                    if (pointPolygonTest(trafficArea.at(j).getTrafficArea(), detectedObjectCenter, false) > 0) {
                        detectedObjectsInScene.at(i).setIndexOfTrafficArea(trafficArea.at(j).getTrafficAreaIndex());

                        // check object movement in traffic area
                        if (trafficArea[j].getIsSelectedTrafficAreaDirection() == true) {                          
                            if (trafficArea[j].isDirectionOfMovementAllowed(objectDirection) == true || objectDirection == TrafficArea::TrafficAreaDirection::none) {
                                detectedObjectsInScene.at(i).resetMovementDirectionTtl();
                                //if(isDebugMode == true)
                                //    cv::circle(frame, detectedObjectCenter, 10, cv::Scalar(0, 255, 0), 5);
                            }
                            else {
                                detectedObjectsInScene.at(i).decreaseMovementDirectionTtl();
                                //if (isDebugMode == true)
                                //    cv::circle(frame, detectedObjectCenter, 10, cv::Scalar(0, 0, 255), 5);
                            }
                        }
                    }                   
                }
                //if (detectedObjectsInScene.at(i).getMovementDirectionTtl() <= 0 && indexOfObjectDrivingInOppositeDirection != detectedObjectsInScene.at(i).getIndexOfObject()) {
                if (detectedObjectsInScene.at(i).getMovementDirectionTtl() <= 0) {
                    if (std::find(unpermittedLaneTakenJson.begin(), unpermittedLaneTakenJson.end(), detectedObjectsInScene[i].getObjectName()) != unpermittedLaneTakenJson.end()) {
                        evenTypeHandler->sendIllegalOppositeDirection(frame, detectedObjectsInScene.at(i), detectedObjectsInScene.at(i).getIndexOfTrafficArea(), barriers, trafficLight);
                        //std::cout << "id: " << detectedObjectsInScene.at(i).getIndexOfTrafficArea() << std::endl;
                        isDrivingInOppositeSide = true;
                        iteratorForResetDrivingInOppositeSideWarning = 0;
                        cv::rectangle(frame, cv::Rect((detectedObjectsInScene.at(i).getObjectRect().x), (detectedObjectsInScene.at(i).getObjectRect().y), ((detectedObjectsInScene.at(i).getObjectRect().width)), ((detectedObjectsInScene.at(i).getObjectRect().height))), cv::Scalar(25, 25, 255), 2);
                    }
                }
            }
        }
    }
}

void checkUnpermittedStoppingInMonitoredArea(cv::Mat& frame, std::vector<DetectedObject>& detectedObjectsInScene) {
    if (monitoredArea.getMonitoredArea().size() > 0) {
        for (size_t i = 0; i < detectedObjectsInScene.size(); i++)
        {
            if (std::find(unpermittedStoppingJson.begin(), unpermittedStoppingJson.end(), detectedObjectsInScene[i].getObjectName()) != unpermittedStoppingJson.end()) {            
                cv::Point detectedObjectCenter;
                detectedObjectCenter.x = (detectedObjectsInScene.at(i).getObjectRect().x + detectedObjectsInScene.at(i).getObjectRect().width / 2);
                detectedObjectCenter.y = (detectedObjectsInScene.at(i).getObjectRect().y + detectedObjectsInScene.at(i).getObjectRect().height / 2);
                // centroid offset in y direction
                //detectedObjectCenter.y = detectedObjectCenter.y + ((detectedObjectsInScene.at(i).getObjectRect().height / 2) * 1 / 3);
                detectedObjectCenter.y = detectedObjectCenter.y + (((detectedObjectsInScene.at(i).getObjectRect().height / 3)));
                //cv::circle(frame, detectedObjectCenter, 10, cv::Scalar(255, 255, 0), 2);

                if (pointPolygonTest(monitoredArea.getMonitoredArea(), detectedObjectCenter, false) > 0) {
                    
                    if (detectedObjectsInScene.at(i).isObjectMoving() == false) {
                        isUnpermittedStopping = true;
                        iteratorForResetUnpermittedStopping = 0;
                        cv::rectangle(frame, detectedObjectsInScene.at(i).getObjectRect(), cv::Scalar(0, 0, 255), 4 * scaleFactor);
                        evenTypeHandler->sendUnpermittedStoppingBegin(frame, detectedObjectsInScene.at(i), barriers, trafficLight);
                    }
                    else if (detectedObjectsInScene.at(i).isObjectMoving() == true && detectedObjectsInScene.at(i).isObjectStopped() == true) {
                        cv::rectangle(frame, detectedObjectsInScene.at(i).getObjectRect(), cv::Scalar(0, 255, 0), 4 * scaleFactor);
                        evenTypeHandler->sendUnpermittedStoppingEnd(frame, detectedObjectsInScene.at(i), barriers, trafficLight);
                        detectedObjectsInScene.at(i).setObjectIsMovingAgain();
                    }                   
                }
                else if (detectedObjectsInScene.at(i).isObjectStopped() == true) {
                    cv::rectangle(frame, detectedObjectsInScene.at(i).getObjectRect(), cv::Scalar(0, 255, 0), 4 * scaleFactor);
                    evenTypeHandler->sendUnpermittedStoppingEnd(frame, detectedObjectsInScene.at(i), barriers, trafficLight);
                    detectedObjectsInScene.at(i).setObjectIsMovingAgain();
                }
            }
        }
    }
}

// check if any obstacle is detected in monitored area
void checkObstacleInMonitoredArea(cv::Mat& frame, cv::Mat& frameForProcessing, std::vector<DetectedObject>& detectedObjectsInScene)
{
    if (iteratorForResetDetectedObstacleInMonitoredAreaWarning > warningDisplayTime) {
        isDetectedObstacleInMonitoredArea = false;
        iteratorForResetDetectedObstacleInMonitoredAreaWarning = -1;
        detectedOtherObjectInMonitoredArea.clear();
    }

    if (obstacleInCrossingJson == true) {
        if (monitoredArea.getMonitoredArea().size() > 0) {
            if (frameCounterForCheckObstacle == -1 || frameCounterForCheckObstacle > (cameraFps * initializeTimeForReferenceMatInSeconds)) {

                int isObjectIsDetectedInMonitoredArea = false;

                // Check if is no detected object in monitored area
                for (int i = 0; i < detectedObjectsInScene.size(); i++)
                {
                    //cv::rectangle(frame, detectedObjectsInScene.at(i).getObjectRect(), cv::Scalar(255, 0, 0), 2);
                    cv::Rect objectRect = detectedObjectsInScene.at(i).getObjectRect();
                    if (cv::pointPolygonTest(monitoredArea.getMonitoredArea(), cv::Point(objectRect.x, objectRect.y), false) > 0 ||
                        cv::pointPolygonTest(monitoredArea.getMonitoredArea(), cv::Point(objectRect.x + objectRect.width, objectRect.y), false) > 0 ||
                        cv::pointPolygonTest(monitoredArea.getMonitoredArea(), cv::Point(objectRect.x, objectRect.y + objectRect.height), false) > 0 ||
                        cv::pointPolygonTest(monitoredArea.getMonitoredArea(), cv::Point(objectRect.x + objectRect.width, objectRect.y + objectRect.height), false) > 0) {
                        isObjectIsDetectedInMonitoredArea = true;
                    }
                }

                if (isObjectIsDetectedInMonitoredArea == false) {
                    frameForProcessing.copyTo(referenceMat);
                    cv::GaussianBlur(referenceMat, referenceMat, cv::Size(7, 7), 0, 0);

                    frameCounterForCheckObstacle = 0;
                }
            }
            else {
                frameCounterForCheckObstacle++;
            }

            if (referenceMat.cols > 0 || referenceMat.rows > 0) {
                cv::absdiff(frameForProcessing, referenceMat, fgMask);
                cv::cvtColor(fgMask, fgMask, cv::COLOR_BGR2GRAY);
                cv::threshold(fgMask, fgMask, 10, 255, cv::THRESH_BINARY);

                int morph_type = cv::MORPH_RECT;
                int erosion_size = 6;

                // apply erosion for eliminate small artefacts in scene
                cv::Mat element = cv::getStructuringElement(morph_type,
                    cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                    cv::Point(erosion_size, erosion_size));

                cv::erode(fgMask, fgMask, element);

                int dilatation_size = 22;
                element = cv::getStructuringElement(morph_type,
                    cv::Size(2 * dilatation_size + 1, 2 * dilatation_size + 1),
                    cv::Point(dilatation_size, dilatation_size));

                cv::dilate(fgMask, fgMask, element);

                // contours detections in scene
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(fgMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                if (contours.size() > 0 && monitoredArea.getMonitoredArea().size() > 0) {
                    for (size_t i = 0; i < contours.size(); i++)
                    {
                        if (cv::contourArea(contours.at(i)) > minObstacleAreaInPixels && cv::contourArea(contours.at(i)) < maxObstacleAreaInPixels) {

                            // get moments
                            cv::Moments mu = moments(contours[i], false);
                            // get center of moments
                            cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

                            if (pointPolygonTest(monitoredArea.getMonitoredArea(), cMu, false) > 0) {
                                
                                if (detectedOtherObjectInMonitoredArea.size() > 0) {

                                    //cv::drawContours(frame, contours, i, cv::Scalar(0, 0, 255), 6);
                                    //std::cout << detectedOtherObjectInMonitoredArea.size() << std::endl;
                                    
                                    int indexOfDetectedObject = -1;
                                    for (size_t j = 0; j < detectedOtherObjectInMonitoredArea.size(); j++)
                                    {
                                        cv::Rect rectIntersection = cv::boundingRect(contours[i]) & detectedOtherObjectInMonitoredArea.at(j).getObjectRect();
                                        int rectArea1 = cv::boundingRect(contours[i]).area();
                                        int rectArea2 = detectedOtherObjectInMonitoredArea.at(j).getObjectRect().area();

                                        float rectIntersectionPercentage = 0.0;

                                        if (rectArea1 < rectArea2) {
                                            rectIntersectionPercentage = (float)rectIntersection.area() / rectArea1;
                                        }
                                        else {
                                            rectIntersectionPercentage = (float)rectIntersection.area() / rectArea2;
                                        }

                                        if (rectIntersectionPercentage > 0.45) {
                                            indexOfDetectedObject = j;

                                            detectedOtherObjectInMonitoredArea.at(j).increaseTtl();
                                            detectedOtherObjectInMonitoredArea.at(j).updateObjectRect(boundingRect(contours[i]));
                                        }
                                    }

                                    if (indexOfDetectedObject == -1) {
                                        detectedOtherObjectInMonitoredArea.push_back(DetectedOtherObject(cv::boundingRect(contours[i]), (cv::boundingRect(contours[i]).br() + cv::boundingRect(contours[i]).tl()) * 0.5, chrono::high_resolution_clock::now()));
                                    }
                                }
                                else {
                                    detectedOtherObjectInMonitoredArea.push_back(DetectedOtherObject(cv::boundingRect(contours[i]), (cv::boundingRect(contours[i]).br() + cv::boundingRect(contours[i]).tl()) * 0.5, chrono::high_resolution_clock::now()));
                                }
                            }
                        }
                    }
                }
                
                // remove unuseless detected objects
                for (size_t j = 0; j < detectedOtherObjectInMonitoredArea.size(); j++)
                {
                    if (chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - detectedOtherObjectInMonitoredArea.at(j).getStartTime()).count() >= obstacleDetectingTimeInSeconds) {

                        isDetectedObstacleInMonitoredArea = true;
                        iteratorForResetDetectedObstacleInMonitoredAreaWarning = 0;

                        obstacleInMonitoredArea = cv::Rect(detectedOtherObjectInMonitoredArea.at(j).getObjectRect());

                        evenTypeHandler->sendObstacleInMonitoredArea(frame, j, barriers, trafficLight);

                        detectedOtherObjectInMonitoredArea.clear();
                        break;
                    }

                    detectedOtherObjectInMonitoredArea.at(j).decreaseTtl();
                    if (detectedOtherObjectInMonitoredArea.at(j).getTtl() <= 0 || (chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - detectedOtherObjectInMonitoredArea.at(j).getStartTime()).count() >= (obstacleDetectingTimeInSeconds + 15.0))) {
                        detectedOtherObjectInMonitoredArea.erase(detectedOtherObjectInMonitoredArea.begin() + j);
                    }
                }

                // Show binary output from background subtractor
                if (false) {
                    jetsonImShow("BG Mask", fgMask);
                    cv::waitKey(1);
                }
            }
        }
    }
}

// Function chceck if file is exists in project directiory
bool isFileExist(std::string& name) {
    std::ifstream file(name);
    if (!file)
        return false;
    else
        return true;
}

void onExitSignal(int s) {
    onExit(s, stopGrabFrames, cap, videoWriterWithAnnotations, videoWriterOriginal, net);
}


//void bufferImage(cv::Mat frame)
//{
//    bufferedImages.push_back(frame);
//    //if (bufferedImages.size() > 100
//    if (bufferedImages.size() > eventVideoDuration)
//    {        
//        bufferedImages.erase(bufferedImages.begin());
//        //evenTypeHandler.sendVehiclePassage(frame, 0, "Bus");
//    }
//    if(bufferReady == false)
//    {
//        if (bufferedImages.size() > cameraFps)
//            bufferReady = true;
//    }
//}
//
//std::vector<cv::Mat> getBufferedImages()
//{
//    return bufferedImages;
//}

//void writeEventVideo(string filePathName)
//{
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    std::string originalPath = filePathName.substr(0, filePathName.find_last_of("//") + 1);
//    std::string pathToVideos = "/mnt/nvme/" + originalPath;
//
//    createFolder(pathToVideos);
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//
//    std::string fileName = filePathName.substr(filePathName.find_last_of("//") + 1);
//    std::string beforeUploadName = "v" + fileName;
//    std::string afterUploadName = fileName;
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    /*std::vector<cv::Mat> bufferedImagesToWrite = getBufferedImages();*/
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //int indexToCheckChanges = bufferedImagesToWrite.size() / 2;
//    //uchar imageToCheck = bufferedImagesToWrite.at(indexToCheckChanges)[0];
//    //std::cout << "cameraFps: " << cameraFps << std::endl;
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //std::cout << "bufferedImagesToWrite.size(): " << bufferedImagesToWrite.size() << std::endl;
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //std::cout << "bufferedImagesToWrite.size(): " << bufferedImagesToWrite.size() << std::endl;
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //int msToWait = (int)round((1.0 / (double)cameraFps) * (double)bufferedImagesToWrite.size() / 2.0) * 1000;
//    int msToWait = (int)round(eventVideoDuration / 2.0)*1000;
//    
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    /*while(bufferedImagesToWrite.at(indexToCheckChanges)[0]==imageToCheck)
//    {*/
//    //std::cout << imageToCheck << std::endl;
//    //bufferedImagesToWrite = getBufferedImages();
//    //std::cout << "waiting: " << msToWait << std::endl;
//    sleep(msToWait);
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //bufferedImagesToWrite = getBufferedImages();
//    std::vector<cv::Mat> bufferedImagesToWrite = getBufferedImages();
//    //std::cout << "msToWait: " << msToWait << std::endl;
//    //std::cout << "bufferedImagesToWrite.size(): " << bufferedImagesToWrite.size() << std::endl;
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //}
//    //cv::Mat actualFrame = cv::imdecode(bufferedImagesToWrite.front(), cv::IMREAD_COLOR
//    cv::Mat actualFrame = bufferedImagesToWrite.front();
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    bufferedImagesToWrite.erase(bufferedImagesToWrite.begin());
//    cv::VideoWriter actualEventVideoWriter = createJetsonVideoWriter(pathToVideos, beforeUploadName, actualFrame.cols, actualFrame.rows);
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //cv::VideoWriter actualEventVideoWriter = createWindowsVideoWriter("",actualFrame.cols, actualFrame.rows);
//
//    actualEventVideoWriter.write(actualFrame);
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //std::cout << "saving" << std::endl;
//    for (auto& bufferedImageToWrite : bufferedImagesToWrite) // access by reference to avoid copying
//    {
//        actualEventVideoWriter.write(bufferedImageToWrite);
//        //std::cout << "frame from buffer" << std::endl;
//    }
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    //std::cout << "all frame from buffer writed " << std::endl;
//    actualEventVideoWriter.release();
//    //std::cout << "Renaming video: " + pathToVideos + beforeUploadName + ".mp4" << " to: " << pathToVideos + afterUploadName + ".mp4" << std::endl;
//    rename(std::string(pathToVideos + beforeUploadName + ".mp4").c_str(), std::string(pathToVideos + afterUploadName + ".mp4").c_str());
//
//    //std::cout << __LINE__ << " " << __func__ << std::endl;
//    evenTypeHandler.numOfActualGeneratedVideos -= 1;
//    //std::cout << "Actual generated videos: " << evenTypeHandler.numOfActualGeneratedVideos << std::endl;
//}

void jetsonImShow(std::string kWinName, cv::Mat frame)
{
    if (slowConnection)
    {
        actualFrameShowedTime = std::chrono::high_resolution_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(actualFrameShowedTime - lastFrameShowedTime).count() > 1000)
        {
            lastFrameShowedTime = std::chrono::high_resolution_clock::now();
            imshow(kWinName, frame);
        }
    }
    else
    {
        imshow(kWinName, frame);
    }		
}
#if __has_include("cameraGrabModule.h")

nlohmann::json getStatisticJson()
{
    //{
    //    sourceId: c9931362 - 1ed9 - 4c76 - 97cc - 43a71b49fb61,
    //    time : "2022-03-28T16:21:14+00",
    //    car : 2,
    //    bus : 3,
    //    truck : 0,
    //    van : 6
    //}

    json statisticJson;

    char timeInChars[100];
    std::string timInString;
    time_t nowTime = time(0);
    struct tm localTime = { 0 };
    localtime_r(&nowTime, &localTime);

    strftime(timeInChars, 100, "%Y-%m-%dT%H:%M:%S", localtime(&nowTime));
    timInString = timeInChars;
    std::stringstream timeOff;
    timeOff << std::setw(2) << std::setfill('0') << 0;
    timInString = timInString + std::string("+") + timeOff.str();

    statisticJson["sourceId"] = cameraIdJson;
    statisticJson["time"] = timInString.c_str();
    statisticJson["car"] = counterLineNumOfObjects["car"];
    statisticJson["bus"] = counterLineNumOfObjects["bus"];
    statisticJson["truck"] = counterLineNumOfObjects["truck"];
    statisticJson["van"] = counterLineNumOfObjects["van"];

    counterLineNumOfObjects["car"] = 0;
    counterLineNumOfObjects["bus"] = 0;
    counterLineNumOfObjects["truck"] = 0;
    counterLineNumOfObjects["van"] = 0;
    
    //std::cout << "event status: " << statisticJson.dump() << std::endl;
    return statisticJson;
}

void statisticSending()
{
    while (!stopGrabFrames)
    {
        try
        {  
            sleep(3600000);
            //mutualJsonClientSendStatistic(targetIPaddress, targetPort, sslAderosCertificatePath, sslCameraCertificatePath, sslCameraKeyPath, getStatisticJson());            
            //sleep(60000);
        }
        catch (const std::exception& e)
        {
            std::cout << "Exception was caught. Message: '" << e.what() << std::endl;
            std::cout << __LINE__ << " " << __func__ << std::endl;

            std::ofstream fileToWrite;
            fileToWrite.open("exceptions.log");
            fileToWrite << "Exception was caught. Message: '" << e.what() << "'\n";
            fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
            fileToWrite.close();
        }
    }
}

std::thread startStatisticSending()
{
    std::thread statisticSendingThread = std::thread(statisticSending);
    return statisticSendingThread;
}
#else
void statisticSending() {}
std::thread startStatisticSending()
{
    std::thread statisticSendingThread = std::thread(statisticSending);
    return statisticSendingThread;
}
#endif


std::string generateGUID()
{
#if __has_include("cameraGrabModule.h")    
    uuid_t uuid;
    uuid_generate_random(uuid);
    char generatedGUID[37];
    uuid_unparse(uuid, generatedGUID);
    //std::cout << "UUID: " << generatedGUID << std::endl;
#else
    UUID uuid;
    UuidCreate(&uuid);

    unsigned char* str;
    UuidToStringA(&uuid, &str);

    std::string generatedGUID((char*)str);

    RpcStringFreeA(&str);
#endif

    return generatedGUID;
}

std::string getContentPath()
{
    string fileName = cameraIdJson;

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%d%m%Y");        

    fileName += "/" + ss.str() + "/";
    fileName += generateGUID();
    //std::cout << "fileName: " << fileName << std::endl;
    return fileName;    
}

void createFolder(std::string folderName)
{    
#if __has_include("cameraGrabModule.h")    
    struct stat st = { 0 };    
    if (stat(folderName.c_str(), &st) == -1) {        
        mkdir(folderName.c_str(), 0777);        
    }
#endif
}
