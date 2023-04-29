
#ifndef EVENTTYPE_H_
#define EVENTTYPE_H_
#include "Barriers.h"
#include <tuple>
#include <random>
#include "json.hpp"
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <iostream>
#include "config.h"
#include <map>
#include "TrafficLight.h"
#include "JsonSendThread.h"
#include "EventVideoThread.h"
#include <fstream>
#include "LPReader.h"


using namespace std;
using namespace nlohmann;

extern string swVersion;

/** Subset of this events are supported on the server
*/
enum eventTypes { vehiclePassage, unpermittedDirectionTaken, unpermittedLaneTaken, unpermittedOvertaking, redLightViolation, potentialRedLightViolation, unpermittedStoppingBegin, unpermittedStoppingEnd, unpermittedVehiclePassage, suspiciousBehaviorBegin, suspiciousBehaviorEnd, obstacleInCrossingAppeared, obstacleInCrossingRemoved, other };

/** Subset of this severity are supported on the server
*/
enum severity { information, warning, mediumHigh, high, highest, notClassified };

/** Subset of this locataions are supported on the server
*/ 
enum locationType { roadCrossing, footCrossing, bikeCrossing, roadNecking, alternatingDirectionSite, tunnel, bridge, twoRoadsCrossing, multipleRoadsCrossing, tTypeCrossing, branchingInCurve, ringRoadSite, crossingWithTrafficLights, otherLocation };

/** Subset of this image/video type are supported on the server
*/
enum imageType { preview, image, video };

/** Subset of this vehicles type are supported on the server
*/
enum objectType { singleTrackVehicle, doubleTrackVehicle, person, generalObject };

/** Subset of this signal types types/devices are supported on the server
*/
enum signalType { czTrafficLightS1, czTrafficLightS2, czTrafficLightS3, czTrafficLightS4, czTrafficLightS5, czTrafficLightS6, czTrafficLightS7, czTrafficLightS8a, czTrafficLightS8b, czTrafficLightS8c, czTrafficLightS8d, czTrafficLightS9, czTrafficLightS10, czTrafficLightS11, czTrafficLightS12, czTrafficLightS13, czTrafficLightS14, czTrafficLightS15 };


//#if __has_include("cameraGrabModule.h")    
//#include "mutualJsonClient.h"
//#else
//int mutualJsonClientSend(const char* ipAddress, const char* port, const char* ca_pem, const char* cert_pem, const char* key_pem, json eventJson, const char* imageName);
//#endif

/*! EventType class contains handlinh with all events */
class EventType {
private:
	//const char* ipAddress; //!< server IP address
	//const char* port; //!< server port
	//const char* ca_pem; //!< SSL CA
	//const char* cert_pem; //!< SSL certificate
	//const char* key_pem; //!< SSL key

	//std::vector<std::string> trafficAreaNames;  //!< All traffic area names
	//int eventVideoDuration; //!< Time for lenght of event video
	//EventVideoThread defaultEventVideoThread;
	//EventVideoThread& eventVideoThread;
	/**
	 * Function for generate event JSON, send it to server and upload all included files (image/video)
	 *
	 * @param eventTypes eventType is occured event type
	 * @param severity severity is occured event severity
	 * @param imageType imageType is type of image
	 * @param std::string imageName is image name for upload to server
	 * @param std::string videoName is video name for upload to server
	 * @param std::string objectClass is object class which generates event
	 * @param Barriers barriers is Barriers object, which contains actual necessary informations about barriers in location
	 * @param TrafficLight trafficLight is TrafficLight object, which contains actual necessary informations about traffic light in location
	 * @param std::string lP = std::string() is readed license plate
	 * @param int lPProbability = -1 is probability from OCR license plate reading
	 * @param int indexOfTrafficArea = -1 is index of traffic area where event occured
	 * @param int deleyAfterRestricted = -1 is time after which after monitored area was restricted
	 */
	void generateAndSendJson(eventTypes eventType, severity severity, imageType imageType, std::string imageName, std::string videoName, std::string objectClass, Barriers barriers, TrafficLight trafficLight, std::string lP = std::string(), int lPProbability = -1, int indexOfTrafficArea = -1, int deleyAfterRestricted = -1);

	/**
	 * Function for generate actual time and date in format for server
	 * @return string is actual tima end date
	 */
	string getTimeToJson();

	unsigned int sequenceNumber; //!< variable for store actual sequence number

	/**
	 * Function for generate random char, inspirated by https://lowrey.me/guid-generation-in-c-11/
	 * @return string is actual tima end date
	 */
	unsigned int random_char();

	/**
	 * Function for iterate and get actual event sequence number
	 * @return unsigned int is actual sequence number
	 */
	unsigned int getSequenceNumber();

	/**
	 * Function for generate event JSON template
	 * @return json is template of event JSON
	 */
	json getEventJsonTemplate();

	int maxGeneratedVideosInSameTime; //!< variable for specify maximum video encoded videos in same time

	vector<int> illegalMovementDirectionObjectIndexes; /*!< vector for store all objects indexes with illegal movement direction */
	vector<int> illegalOppositeDirectionObjectIndexes; /*!< vector for store all objects indexes with illegal opposite direction */
	vector<int> illegalOvertakingObjectIndexes; /*!< vector for store all objects indexes with illegal overtaking */
	vector<int> enterToMonitoredAreaObjectIndexes; /*!< vector for store all objects indexes with enter to monitored area */
	vector<int> potentiallyEnterToMonitoredAreaObjectIndexes; /*!< vector for store all objects indexes with potential enter to monitored area */
	vector<int> unpermittedStoppingBeginObjectIndexes; /*!< vector for store all objects indexes with unpermitted stopping begin */
	vector<int> unpermittedStoppingEndObjectIndexes; /*!< vector for store all objects indexes with unpermitted stopping end */
	vector<int> obstacleInCrossing; /*!< vector for store all objects indexes with obstacle in crossing */
	vector<int> unpermittedVehiclePassageIndexes; /*!< vector for store all objects indexes with unpermitted vehicle passage */
	vector<int> ilegalEnterToAreaObjectIndexes; /*!< vector for store all objects indexes with illegal enter to area */
	vector<int> illegalPassingTheAreaObjectIndexes;/*!< vector for store all objects indexes with illegal vehicle passage */

	vector<int> suspiciousBehaviorObjectIndexes; /*!< vector for store all objects indexes with suspicious behavior */
	vector<int> busDetectedObjectIndexes; /*!< vector for store all objects indexes with vehicle passage */

	LPReader* lPReader;

	/**
	 * Function for generate event JSON template
	 * @param eventTypes eventType is occured event type
	 * @param severity severity is occured event severity
	 * @param imageType imageType is type of image
	 * @param cv::Mat frame is frame on which event occured
	 * @param std::string objectName is object class which generates event
	 * @param Barriers barriers is Barriers object, which contains actual necessary informations about barriers in location
	 * @param TrafficLight trafficLight is TrafficLight object, which contains actual necessary informations about traffic light in location
	 * @param cv::Mat LP is matrix with license plate to read with OCR
	 * @param int indexOfTrafficArea = -1 is index of traffic area where event occured
	 * @param int deleyAfterRestricted = -1 is time after which after monitored area was restricted
	 */
	void writeAndSend(eventTypes eventType, severity severity, imageType imageType, cv::Mat frame, std::string objectName, Barriers barriers, TrafficLight TrafficLight, cv::Mat LP = cv::Mat(), int indexOfTrafficArea = -1, int deleyAfterRestricted = -1);

public:
	//const char* ipAddress; //!< server IP address
	//const char* port; //!< server port
	//const char* ca_pem; //!< SSL CA
	//const char* cert_pem; //!< SSL certificate
	//const char* key_pem; //!< SSL key

	std::vector<std::string> trafficAreaNames;  //!< All traffic area names
	int eventVideoDuration; //!< Time for lenght of event video
	//EventVideoThread defaultEventVideoThread;
	EventVideoThread* eventVideoThread;
	JsonSendThread* jsonSendThread;


	//EventType& operator=(const EventType& mc) { 
	//	EventType newEventType = EventType(mc.ipAddress, mc.port, mc.ca_pem, mc.cert_pem, mc.key_pem, mc.trafficAreaNames, mc.eventVideoDuration, mc.eventVideoThread)
	//	std::cout << "test: " << mc.eventVideoThread->cameraFps << std::endl;
	//	std::cout << "test: " << *this. eventVideoThread->cameraFps << std::endl;
	//	return *this; 
	//}
	//EventType& operator=(const EventType& mc);

	int numOfActualGeneratedVideos; //!< int stored actual encoded videos

	/**
	* Default constructor for EventType class
	*/
	EventType();

	/**
	* Constructor for AppWindow class with additional parameters
	* @param const char* ipAddress is IPAddress of ADEROS server
	* @param const char* port is port of ADEROS server
	* @param const char* Ca_pem is SSL CA
	* @param const char* Cert_pem is SSL certificate
	* @param const char* Key_pem is SSL key
	* @param std::vector<std::string> newTrafficAreaNames is vector of traffic area names
	*/
	//EventType(const char* ipAddress, const char* port, const char* Ca_pem, const char* Cert_pem, const char* Key_pem, std::vector<std::string> newTrafficAreaNames, int newEventVideoDuration, EventVideoThread* eventVideoThreadNew);
	EventType(std::vector<std::string> newTrafficAreaNames, int newEventVideoDuration, EventVideoThread* eventVideoThreadNew, JsonSendThread* newJsonSendThread);


	EventType(EventVideoThread* eventVideoThreadNew);
	/**
	 * Function for generate GUID, inspirated by https://lowrey.me/guid-generation-in-c-11/
	 * @return string is generated GUID
	 */
	std::string generate_hex(const unsigned int len);

	/**
	 * Function for generate illegal movement directon event
	 *
	 * @param DetectedObject& detectedObject is detected object of event
	 * @param int indexOfTrafficArea is traffic area in which event was detected
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	//void sendIllegalMovementDirection(DetectedObject& detectedObject, int indexOfTrafficArea, Barriers barriers, TrafficLight trafficLight);

	/**
	 * Function for generate illegal opposite direction event
	 *
	 * @param cv::Mat frame is frame captured when event occured
	 * @param int indexOfTrafficArea is traffic area in which event was detected
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	void sendIllegalOppositeDirection(cv::Mat frame, DetectedObject& detectedObject, int indexOfTrafficArea, Barriers barriers, TrafficLight trafficLight);

	//nedovolene predjizdeni
	//void sendIllegalOvertaking(DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight);

	/**
	 * Function for generate red light violation event
	 *
	 * @param cv::Mat frame is frame captured when event occured
	 * @param DetectedObject& detectedObject is detected object of event
	 * @param int indexOfTrafficArea is traffic area in which event was detected
	 * @param int deleyAfterRestricted is time after red light turn on
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	void sendEnterToMonitoredArea(cv::Mat frame, DetectedObject& detectedObject, int indexOfTrafficArea, int deleyAfterRestricted, Barriers barriers, TrafficLight trafficLight);	

	/**
	 * Function for generate unpermitted stopping begin event
	 * @param DetectedObject& detectedObject is detected object of event
	 * @param cv::Mat frame is frame captured when event occured	 
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	void sendUnpermittedStoppingBegin(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight);

	/**
	 * Function for generate unpermitted stopping end event
	 * @param DetectedObject& detectedObject is detected object of event
	 * @param cv::Mat frame is frame captured when event occured
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	void sendUnpermittedStoppingEnd(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight);

	//prujezd vozidla
	//void sendIllegalPassingTheArea(DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight);	

	/**
	 * Function for generate obstacle in monitored area	 event
	 * @param cv::Mat frame is frame captured when event occured
	 * @param int indexOfDetectedObject is index of detected object
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	void sendObstacleInMonitoredArea(cv::Mat frame, int indexOfDetectedObject, Barriers barriers, TrafficLight trafficLight);

	/**
	 * Function for generate vehicle passage event
	 * @param cv::Mat frame is frame captured when event occured
	 * @param DetectedObject& detectedObject is detected object of event
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status when event occured
	 */
	void sendVehiclePassage(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight);

	/**
	 * Function for generate unpermitted vehicle passage event
	 * @param cv::Mat frame is frame captured when event occured
	 * @param DetectedObject& detectedObject is detected object of event
	 * @param Barriers barriers is barriers status when event occured
	 * @param TrafficLight trafficLight is traffic light status ehn event occured
	 */
	void sendUnpermittedVehiclePassage(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight);

	/**
	 * Function for write frame to file and get path of saved image
	 * @param cv::Mat frame for write to file
	 * @return std::string path of writed image
	 */
	std::string saveImage(cv::Mat frame);

	void removeObject(int objectIndex);

	std::string* getObjectNames();

	
	
	/**
	 * Function for add vehicle informations to event JSON
	 * @param json& eventJson is JSON to which vehicle will be added
	 * @param std::string objectClass is name of vehicle class to add
	 * @param std::string lP = std::string() is license plate of added vehicle
	 * @param int lPProbability = -1 is OCR probability of readed license plate
	 */
	void addVehicleToJson(json& eventJson, std::string objectClass, std::string lP = std::string(), int lPProbability = -1);

	/**
	 * Function for translate idnex of area to name
	 * @param int indexOfTrafficArea is index of traffic areay to translate
	 * @return std::string is area name
	 */
	std::string trafficAreaNumberToName(int indexOfTrafficArea);
};
#endif