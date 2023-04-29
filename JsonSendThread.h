#pragma once
#ifndef JSONSENDTHREAD_H_
#define JSONSENDTHREAD_H_


#include "Barriers.h"
#include "TrafficLight.h"
#include "json.hpp"
#include <thread>
#include "threadQueue.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/imgcodecs.hpp>
#include <fstream>

using namespace nlohmann;

#if __has_include("cameraGrabModule.h")    
#include "mutualJsonClient.h"
#else
// kiac koment
//int mutualJsonClientSend(const char* ipAddress, const char* port, const char* ca_pem, const char* cert_pem, const char* key_pem, json eventJson) { return 0; }
#endif

//typedef struct JsonValuesToSend {
//    std::string guid;
//    std::string eventTypeString;
//    std::string severityString;
//    std::string imageTypeString;
//    std::string imageName;
//    std::string videoName;
//    std::string objectClass;
//    Barriers barriers;
//    TrafficLight trafficLight;
//    std::string lP;
//    int lPProbability;
//    int indexOfTrafficArea;
//    int deleyAfterRestricted;
//    JsonValuesToSend(std::string guid, std::string eventTypeString, std::string severityString, std::string imageTypeString, std::string imageName, std::string videoName, std::string objectClass, Barriers barriers, TrafficLight trafficLight, std::string lP, int lPProbability, int indexOfTrafficArea, int deleyAfterRestricted) : guid(guid), eventTypeString(eventTypeString), severityString(severityString), imageTypeString(imageTypeString), imageName(imageName), videoName(videoName), objectClass(objectClass), barriers(barriers), trafficLight(trafficLight), lP(lP), lPProbability(lPProbability), indexOfTrafficArea(indexOfTrafficArea), deleyAfterRestricted(deleyAfterRestricted) {};
//
//}JsonValuesToSend;

class JsonSendThread
{
public:
	JsonSendThread();
    JsonSendThread(const char* newIpAddress, const char* newPort, const char* newCa_pem, const char* newCert_pem, const char* newKey_pem, bool* newStopGrabFrames);

	~JsonSendThread();
    std::thread runJsonSending();
    void setParams(const char* newIpAddress, const char* newPort, const char* newCa_pem, const char* newCert_pem, const char* newKey_pem, bool* newStopGrabFrames);
    void addJsonToSend(json jsonToSend);
    bool* stopGrabFrames;
	///**
 //    * Function for generate event JSON, send it to server and upload all included files (image/video)
 //    *
 //    * @param eventTypes eventType is occured event type
 //    * @param severity severity is occured event severity
 //    * @param imageType imageType is type of image
 //    * @param std::string imageName is image name for upload to server
 //    * @param std::string videoName is video name for upload to server
 //    * @param std::string objectClass is object class which generates event
 //    * @param Barriers barriers is Barriers object, which contains actual necessary informations about barriers in location
 //    * @param TrafficLight trafficLight is TrafficLight object, which contains actual necessary informations about traffic light in location
 //    * @param std::string lP = std::string() is readed license plate
 //    * @param int lPProbability = -1 is probability from OCR license plate reading
 //    * @param int indexOfTrafficArea = -1 is index of traffic area where event occured
 //    * @param int deleyAfterRestricted = -1 is time after which after monitored area was restricted
 //    */
     //void generateAndSendJson(std::string guid, std::string eventTypeString, std::string severityString, std::string imageTypeString, std::string imageName, std::string videoName, std::string objectClass, Barriers barriers, TrafficLight trafficLight, bool videoWillBeGenerated, std::string lP = std::string(), int lPProbability = -1, int indexOfTrafficArea = -1, int deleyAfterRestricted = -1);
     //std::string* objectNames;

private:
	
	threadQueue<json> jsonsToSendQueue;
    unsigned int sequenceNumber = 0;
    const char* ipAddress; //!< server IP address
    const char* port; //!< server port
    const char* ca_pem; //!< SSL CA
    const char* cert_pem; //!< SSL certificate
    const char* key_pem; //!< SSL key
    void jsonSending();
    //json getEventJsonTemplate();
    ///**
    // * Function for iterate and get actual event sequence number
    // * @return unsigned int is actual sequence number
    // */
    //unsigned int getSequenceNumber();
    ///**
    // * Function for translate idnex of area to name
    // * @param int indexOfTrafficArea is index of traffic areay to translate
    // * @return std::string is area name
    // */
    //std::string trafficAreaNumberToName(int indexOfTrafficArea);

    ///**
    // * Function for add vehicle informations to event JSON
    // * @param json& eventJson is JSON to which vehicle will be added
    // * @param std::string objectClass is name of vehicle class to add
    // * @param std::string lP = std::string() is license plate of added vehicle
    // * @param int lPProbability = -1 is OCR probability of readed license plate
    // */
    //void addVehicleToJson(json& eventJson, std::string objectClass, std::string lP = std::string(), int lPProbability = -1);


    ///**
    // * Function for generate actual time and date in format for server
    // * @return string is actual tima end date
    // */
    //std::string getTimeToJson();
    

};

#endif