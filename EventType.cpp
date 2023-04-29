#include "EventType.h"

#include "systemThreads.h"
#include "utilities.h"

//extern LPReader lPReader;

//unpermittedLaneTaken | jízda v protisměru(v nepovoleném pruhu) | high
//redLightViolation | vjezd do křížení na červenou | highest
//unpermittedStoppingBegin | nedovolené / neočekávané stání(začátek) (area se zpožděním) | mediumHigh
//unpermittedStoppingEnd | nedovolené / neočekávané stání(konec) (area se zpožděním) | information
//unpermittedVehiclePassage | vjezd nepovoleného vozidla | warning
//vehiclePassage | průjezd vozidla | information

//obstacleInCrossingAppeared | překážka v křížení(výskyt) | high
//obstacleInCrossingRemoved | překážka v křížení(odstranění) | information


//define static informations for JSON uploaded to server
std::string eventNames[] = { "vehiclePassage", "unpermittedDirectionTaken", "unpermittedLaneTaken", "unpermittedOvertaking", "redLightViolation", "potentialRedLightViolation", "unpermittedStoppingBegin", "unpermittedStoppingEnd", "unpermittedVehiclePassage", "suspiciousBehaviorBegin", "suspiciousBehaviorEnd", "obstacleInCrossingAppeared", "obstacleInCrossingRemoved", "other" };
std::string severityNames[] = { "information", "warning", "mediumHigh", "high", "highest", "notClassified" };
std::string locationNames[] = { "roadCrossing", "footCrossing", "bikeCrossing", "roadNecking", "alternatingDirectionSite", "tunnel", "bridge", "twoRoadsCrossing", "multipleRoadsCrossing", "tTypeCrossing", "branchingInCurve", "ringRoadSite", "crossingWithTrafficLights", "other" };
std::string imageNames[] = { "preview", "image", "video" };
std::string objectNames[] = { "singleTrackVehicle", "doubleTrackVehicle", "person", "generalObject" };
std::string signalTypeNames[] = { "czTrafficLightS1", "czTrafficLightS2", "czTrafficLightS3", "czTrafficLightS4", "czTrafficLightS5", "czTrafficLightS6", "czTrafficLightS7", "czTrafficLightS8a", "czTrafficLightS8b", "czTrafficLightS8c", "czTrafficLightS8d", "czTrafficLightS9", "czTrafficLightS10", "czTrafficLightS11", "czTrafficLightS12", "czTrafficLightS13", "czTrafficLightS14", "czTrafficLightS15" };

//default constructor for event type
//EventType::EventType() : eventVideoThread(defaultEventVideoThread){
EventType::EventType(){
    sequenceNumber = 0;
    numOfActualGeneratedVideos = 0;
    maxGeneratedVideosInSameTime = 5;
    eventVideoDuration = 140;
    lPReader = new LPReader();

}

//EventType::EventType(std::vector<std::string> newTrafficAreaNames, int newEventVideoDuration, EventVideoThread* eventVideoThreadNew, JsonSendThread* jsonSendThread) : trafficAreaNames(newTrafficAreaNames), eventVideoDuration(newEventVideoDuration), eventVideoThread(eventVideoThreadNew), jsonSendThread(jsonSendThread) {}

//constructor for event type
//EventType::EventType(const char* ipAddress, const char* port, const char* Ca_pem, const char* Cert_pem, const char* Key_pem, std::vector<std::string> newTrafficAreaNames, int eventVideoDuration, EventVideoThread& eventVideoThreadNew) : ipAddress(ipAddress), port(port), ca_pem(Ca_pem), cert_pem(Cert_pem), key_pem(Key_pem), trafficAreaNames(newTrafficAreaNames), eventVideoDuration(eventVideoDuration), eventVideoThread(eventVideoThreadNew)
//EventType::EventType(const char* ipAddress, const char* port, const char* Ca_pem, const char* Cert_pem, const char* Key_pem, std::vector<std::string> newTrafficAreaNames, int newEventVideoDuration, EventVideoThread* eventVideoThreadNew)
//{
//    EventType::ipAddress = ipAddress;
//    EventType::port = port;
//    ca_pem = Ca_pem;
//    cert_pem = Cert_pem;
//    key_pem = Key_pem;
//    //bufferReady = false;
//    sequenceNumber = 0;
//    numOfActualGeneratedVideos = 0;
//    maxGeneratedVideosInSameTime = 5;
//    trafficAreaNames = newTrafficAreaNames; 
//    eventVideoDuration = newEventVideoDuration;
//    eventVideoThread = eventVideoThreadNew;
//    //std::cout << "eventVideoThreadNew" << eventVideoThreadNew->cameraFps << std::endl;
//    //std::cout << "eventVideoThread" << eventVideoThread->cameraFps << std::endl;
//
//}

EventType::EventType(std::vector<std::string> newTrafficAreaNames, int newEventVideoDuration, EventVideoThread* eventVideoThreadNew, JsonSendThread* newJsonSendThread)
{    
    //bufferReady = false;
    sequenceNumber = 0;
    numOfActualGeneratedVideos = 0;
    maxGeneratedVideosInSameTime = 5;
    trafficAreaNames = newTrafficAreaNames;
    eventVideoDuration = newEventVideoDuration;
    eventVideoThread = eventVideoThreadNew;
    jsonSendThread = newJsonSendThread;
    //std::cout << "eventVideoThreadNew" << eventVideoThreadNew->cameraFps << std::endl;
    //std::cout << "eventVideoThread" << eventVideoThread->cameraFps << std::endl;
    lPReader = new LPReader();

}

void EventType::writeAndSend(eventTypes eventType, severity severity, imageType imageType, cv::Mat frame, std::string objectName, Barriers barriers, TrafficLight trafficLight, cv::Mat LP, int indexOfTrafficArea, int deleyAfterRestricted)
{
    try {
        string fileName = "";
        std::string imageName = saveImage(frame);
        std::string lP = std::string();
        int lPProbability = -1;
        //std::cout << __LINE__ << " " << __func__ << std::endl;
        if (bufferAndSendVideo && eventVideoThread->bufferReady)
        {
            //std::cout << __LINE__ << " " << __func__ << std::endl;
            if (eventVideoThread->getNumOfVideosInCounter() < maxGeneratedVideosInSameTime)
            {
                //std::cout << __LINE__ << " " << __func__ << std::endl;
                //numOfActualGeneratedVideos += 1;
                //fileName = getContentPath();
                VideoToEncode videoToEncode;
                //std::cout << __LINE__ << " " << __func__ << std::endl;
                videoToEncode.filePathName = getContentPath();
                fileName = videoToEncode.filePathName;
                //std::cout << __LINE__ << " " << __func__ << std::endl;
                videoToEncode.TTL = ((int)round(eventVideoDuration / 2.0) - 1);
                std::cout << "TTL of new video duration: " << videoToEncode.TTL << std::endl;
                //std::cout << __LINE__ << " " << __func__ << std::endl;
                eventVideoThread->addToCounterForVideosToEncode(videoToEncode);
                //std::cout << __LINE__ << " " << __func__ << std::endl;
                //std::thread writeEventVideoThread(writeEventVideo, fileName);
                //writeEventVideoThread.detach();
                //std::cout << __LINE__ << " " << __func__ << std::endl;
            }
            else
            {
                std::cout << "Maximum generated videos has been exceeded, sending only image! Maximum videos: " << maxGeneratedVideosInSameTime << std::endl;
            }
        }



        if (LP.rows > 15 && LP.cols > 15)
        {
            //std::cout << "LP.rows: " << LP.rows << std::endl;
            //std::cout << __LINE__ << " " << __func__ << std::endl;
            std::tuple<std::string, int> lPReaderOutput = lPReader->readLP(LP);
            //std::cout << __LINE__ << " " << __func__ << std::endl;
            lP = std::get<0>(lPReaderOutput);
            //std::cout << __LINE__ << " " << __func__ << std::endl;
            lPProbability = std::get<1>(lPReaderOutput);
            //std::cout << __LINE__ << " " << __func__ << std::endl;
            //std::cout << "lp: " << lP << std::endl;
            //std::cout << "lPProbability: " << lPProbability << std::endl;
        }
        //std::cout << __LINE__ << " " << __func__ << std::endl;
        //JsonValuesToSend jsonValuesToSend(generateGUID(), std::string(eventNames[eventType].c_str()), std::string(severityNames[severity].c_str()), std::string(imageNames[imageType].c_str()), imageName, fileName, objectName, barriers, trafficLight, lP, lPProbability, indexOfTrafficArea, deleyAfterRestricted);
        //jsonSendThread->addJsonToSend();
        generateAndSendJson(eventType, severity, imageType, imageName, fileName, objectName, barriers, trafficLight, lP, lPProbability, indexOfTrafficArea, deleyAfterRestricted);

        //thread generateAndSendJsonThread(&EventType::generateAndSendJson, this, eventType, severity, imageType, imageName, fileName, objectName, barriers, trafficLight, lP, lPProbability, indexOfTrafficArea, deleyAfterRestricted);
        //generateAndSendJsonThread.detach();
    }
    catch (const std::exception& e) // caught by reference to base
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

//jizda v protismeru
void EventType::sendIllegalOppositeDirection(cv::Mat frame, DetectedObject& detectedObject, int indexOfTrafficArea, Barriers barriers, TrafficLight trafficLight)
{ 
    if (!(std::find(illegalOppositeDirectionObjectIndexes.begin(), illegalOppositeDirectionObjectIndexes.end(), detectedObject.getIndexOfObject()) != illegalOppositeDirectionObjectIndexes.end())) {
	    //std::cout << "Illegal opposite direction by " + objectType << std::endl;
	    std::cout << "> EVENT DETECTED [DRIVING IN OPPOSITE DIRECTION]" << std::endl;
        const auto p1 = std::chrono::system_clock::now();

        writeAndSend(unpermittedLaneTaken, high, image, frame, detectedObject.getObjectName(), barriers, trafficLight, detectedObject.getLP(),indexOfTrafficArea);
        illegalOppositeDirectionObjectIndexes.push_back(detectedObject.getIndexOfObject());
    }
}

//vjezd do krizen� na cervenou
void EventType::sendEnterToMonitoredArea(cv::Mat frame, DetectedObject& detectedObject, int indexOfTrafficArea, int deleyAfterRestricted, Barriers barriers, TrafficLight trafficLight)
{
    if (!(std::find(enterToMonitoredAreaObjectIndexes.begin(), enterToMonitoredAreaObjectIndexes.end(), detectedObject.getIndexOfObject()) != enterToMonitoredAreaObjectIndexes.end())) {
	    //std::cout << "Enter to monitored area by " + objectType << std::endl;
	    std::cout << "> EVENT DETECTED [RESTRICTED ENTER TO MONITORED AREA]" << std::endl;
        
        writeAndSend(redLightViolation, highest, image, frame, detectedObject.getObjectName(), barriers, trafficLight, detectedObject.getLP(), indexOfTrafficArea);

        enterToMonitoredAreaObjectIndexes.push_back(detectedObject.getIndexOfObject());
    }
}

void EventType::sendUnpermittedStoppingBegin(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight)
{
    if (!(std::find(unpermittedStoppingBeginObjectIndexes.begin(), unpermittedStoppingBeginObjectIndexes.end(), detectedObject.getIndexOfObject()) != unpermittedStoppingBeginObjectIndexes.end())) {
        std::cout << "> EVENT DETECTED [UNPERMITTED STOPPING IN MONITORED AREA - BEGIN]" << std::endl;
        //std::cout << __LINE__ << " " << __func__ << std::endl;
       
        writeAndSend(unpermittedStoppingBegin, high, image, frame, detectedObject.getObjectName(), barriers, trafficLight, detectedObject.getLP());
        //std::cout << __LINE__ << " " << __func__ << std::endl;

        unpermittedStoppingBeginObjectIndexes.push_back(detectedObject.getIndexOfObject());
        //std::cout << __LINE__ << " " << __func__ << std::endl;
    }
}

void EventType::sendUnpermittedStoppingEnd(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight)
{
    if (!(std::find(unpermittedStoppingEndObjectIndexes.begin(), unpermittedStoppingEndObjectIndexes.end(), detectedObject.getIndexOfObject()) != unpermittedStoppingEndObjectIndexes.end())) {
        std::cout << "> EVENT DETECTED [UNPERMITTED STOPPING IN MONITORED AREA - END]" << std::endl;

        writeAndSend(unpermittedStoppingEnd, information, image, frame, detectedObject.getObjectName(), barriers, trafficLight, detectedObject.getLP());

        unpermittedStoppingEndObjectIndexes.push_back(detectedObject.getIndexOfObject());
    }
}

void EventType::sendObstacleInMonitoredArea(cv::Mat frame, int indexOfDetectedObject, Barriers barriers, TrafficLight trafficLight)
{
    if (!(std::find(obstacleInCrossing.begin(), obstacleInCrossing.end(), indexOfDetectedObject) != obstacleInCrossing.end())) {

        std::cout << "> EVENT DETECTED [OBSTACLE IN MONITORED AREA]" << std::endl;

        writeAndSend(obstacleInCrossingAppeared, high, image, frame, "otherObstacle", barriers, trafficLight);

        obstacleInCrossing.push_back(indexOfDetectedObject);
    }
}

void EventType::sendVehiclePassage(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight)
{
    if (!(std::find(busDetectedObjectIndexes.begin(), busDetectedObjectIndexes.end(), detectedObject.getIndexOfObject()) != busDetectedObjectIndexes.end())) {
	    std::cout << "> EVENT DETECTED [DETECTED " << detectedObject.getObjectName() << " IN FRAME]" << std::endl;
        
	    writeAndSend(vehiclePassage, information, image, frame, detectedObject.getObjectName(), barriers, trafficLight, detectedObject.getLP());
        
        busDetectedObjectIndexes.push_back(detectedObject.getIndexOfObject());
    }
}

void EventType::sendUnpermittedVehiclePassage(cv::Mat frame, DetectedObject& detectedObject, Barriers barriers, TrafficLight trafficLight)
{
    if (!(std::find(unpermittedVehiclePassageIndexes.begin(), unpermittedVehiclePassageIndexes.end(), detectedObject.getIndexOfObject()) != unpermittedVehiclePassageIndexes.end())) {
        std::cout << "> EVENT DETECTED [UNPERMITTED VEHICLE PASSAGE " << detectedObject.getObjectName() << " IN FRAME]" << std::endl;

        writeAndSend(unpermittedVehiclePassage, warning, image, frame, detectedObject.getObjectName(), barriers, trafficLight, detectedObject.getLP());

        unpermittedVehiclePassageIndexes.push_back(detectedObject.getIndexOfObject());
    }
}

//GUID generator https://lowrey.me/guid-generation-in-c-11/
unsigned int EventType::random_char() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    return dis(gen);
}

std::string EventType::generate_hex(const unsigned int len) {
    std::stringstream ss;
    for (auto i = 0; i < len; i++) {
        const auto rc = random_char();
        std::stringstream hexstream;
        hexstream << std::hex << rc;
        auto hex = hexstream.str();
        ss << (hex.length() < 2 ? '0' + hex : hex);
    }
    return ss.str();
}

json EventType::getEventJsonTemplate()
{ 
    // auto eventJson = R"({
    // "eventId": "e13",

    // "isAlert": true,
    // "severity": "warning",
    // "hash": "017f9db155fdb95a4c6d451fcd10f987",
    // "eventSeqNumber": 1,
    // "detectedTime": "2024-03-18T09:31:52+01",
    // "location": {
    //     "city": "Brno",
    //     "street": "�voz",
    //     "direction": "centrum",
    //     "info": " Dohled j�zdy na �ervenou, ORP Brno",
    //     "locationType": "footCrossing",
    //     "laneNumber": 5
    // },
    // "source": {
    //     "id": "Test Kamera online 1",
    //     "name": "lpr_sn0015",
    //     "swVersion": "1.0",
    //     "swName": "lpr_daemon_debug"
    // },
    // "eventType": "vehiclePassage",
    // "images": [
    //     {
    //         "id": "sit1",
    //         "imageType": "image"
    //     },
    //     {
    //         "id": "sample",
    //         "imageType": "video"
    //     }
    // ],
    // "objectType": "doubleTrackVehicle",
    // "objectClass": "bus",
    // "vehicle": {
    //     "registration": "4Z42318",
    //     "registrationProbability": 80,
    //     "adr": "33 1203",
    //     "info": "No other data found for 4Z42318 in CSDb",
    //     "adrProbability": 80,
    //     "mpz": "CZ",
    //     "color": "dark blue",
    //     "numberOfAxles": 2,
    //     "lightsEnabled": true,
    //     "manufacturer": "�koda",
    //     "model": "Octavia II",
    //     "preference": "ambulanceVehicle"
    // },
    // "speed": {
    //     "speedValue": 102.78,
    //     "roundedSpeed": 102,
    //     "limit": 90
    // },
    // "rideOnRed": {
    //     "redDelay": 200,
    //     "toleratedRedDelay": 80
    // },
    // "signalDevice": {
    //     "signalType": "czTrafficLightS1",
    //     "isBarrierDown": false
    // }
    // })"_json;
    auto eventJson = R"({
    "eventId": "e13",
    
    "isAlert": true,
    "severity": "warning",    
    "eventSeqNumber": 1,
    "detectedTime": "2024-03-18T09:31:52+01",
    "location": {
        "city": "Brno",
        "street": "Krizikova",
        "direction": "AZD",
        "info": "Dohled jizdy",
        "locationType": "footCrossing",
        "laneNumber": 4
    },
    "source": {
        "id": "Test Kamera online 1",
        "name": "lpr_sn0015",
        "swVersion": "1.0",
        "swName": "lpr_daemon_debug"
    },
    "eventType": "vehiclePassage",
    "images": [
        {
            "id": "sit1",
            "imageType": "image"
        }
    ],
    "objectType": "doubleTrackVehicle",
    "objectClass": "",
    "signalDevice": {
        "signalType": "czTrafficLightS1",
        "isBarrierDown": false
    }
    })"_json;    

    return eventJson;
}

string EventType::getTimeToJson()
{
    string timeToJson = "";
#if __has_include("cameraGrabModule.h")   
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
    timeToJson = timInString;
#endif
    return timeToJson;	
}

std::string EventType::trafficAreaNumberToName(int indexOfTrafficArea)
{    
    std::string name = "";    
    if(indexOfTrafficArea > -1 && indexOfTrafficArea < trafficAreaNames.size())
        name = trafficAreaNames[indexOfTrafficArea]; 
    return name;
}

void EventType::generateAndSendJson(eventTypes eventType, severity severity, imageType imageType, std::string imageName, std::string videoName, std::string objectClass, Barriers barriers, TrafficLight trafficLight, std::string lP, int lPProbability, int indexOfTrafficArea, int deleyAfterRestricted)
{
    try
    {        
        json eventJson = getEventJsonTemplate();
        
        eventJson["eventType"] = eventNames[eventType].c_str();
        eventJson["severity"] = severityNames[severity].c_str();   

        if (bufferAndSendVideo && eventVideoThread->bufferReady && videoName!="")
        {            
            eventJson["images"] = { {{"id", imageName}, {"imageType", imageNames[imageType]}}, {{"id", videoName}, {"imageType", "video"}} };
        }
        else
        {
            eventJson["images"] = { {{"id", imageName}, {"imageType", imageNames[imageType]}} };
        }
    
        eventJson["eventSeqNumber"] = getSequenceNumber();
        eventJson["eventId"] = generateGUID();

        eventJson["source"] = { {"id", cameraIdJson}, {"name", nameJson} , {"swVersion", swVersion} , {"swName", kWinName} };

        if (indexOfTrafficArea > -1)
            eventJson["location"] = { {"city", cityJson}, {"street", streetJson}, {"direction", trafficAreaNumberToName(indexOfTrafficArea)}, {"info", infoJson}, {"locationType", locationTypeJson}, {"laneNumber", indexOfTrafficArea} };
        else
            eventJson["location"] = { {"city", cityJson}, {"street", streetJson}, {"info", infoJson}, {"locationType", locationTypeJson}};
        
        addVehicleToJson(eventJson, objectClass, lP, lPProbability);
        
        
        if (deleyAfterRestricted > -1)
            eventJson["rideOnRed"] = { {"redDelay", deleyAfterRestricted}, {"toleratedRedDelay", monitoredArea.allowedDelayAfterRestricted} };
                
        eventJson["signalDevice"] = { {"signalType", signalTypeJson} };
        
        if (barriers.getBarriersState() != Barriers::BarriersState::none && barriers.isBarrierDown() == true) {
            eventJson["signalDevice"]["isBarrierDown"] = true;
        }
        else if (barriers.getBarriersState() != Barriers::BarriersState::none){
            eventJson["signalDevice"]["isBarrierDown"] = false;
        }

        eventJson["signalDevice"]["signalState"] = trafficLight.getTrafficLightColorString();

        eventJson["detectedTime"] = getTimeToJson().c_str();

        //std::cout << "detectedTime" << eventJson["detectedTime"] << std::endl;

        //mutualJsonClientSend(ipAddress, port, ca_pem, cert_pem, key_pem, eventJson);
        jsonSendThread->addJsonToSend(eventJson);
    }catch (const std::exception& e) // caught by reference to base
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

void EventType::addVehicleToJson(json &eventJson, std::string objectClass, std::string lP, int lPProbability)
{    
    eventJson["objectClass"] = objectClass;

    if(objectClass == "bicycle" || objectClass == "motorcycle")
    {
        eventJson["objectType"] = objectNames[singleTrackVehicle].c_str();
    }        
    else if(objectClass == "car" || objectClass == "bus" || objectClass == "truck" || objectClass == "van")
    {
        eventJson["objectType"] = objectNames[doubleTrackVehicle].c_str();
    }
    else if (objectClass == "person")
    {
        eventJson["objectType"] = objectNames[person].c_str();
    }else
    {
        eventJson["objectType"] = objectNames[generalObject].c_str();
    }  

    if (lP.length() > 0)
    {
        eventJson["vehicle"] = { {"registration", lP}, {"registrationProbability", lPProbability} };
    }    
}


string EventType::saveImage(cv::Mat frame)
{    
    std::string imagePath = getContentPath();
    string pathToImage = "/mnt/nvme/" + imagePath;   

    createFolder(pathToImage.substr(0, pathToImage.find_last_of("//") + 1));   
	const string pathWithExtension = pathToImage + ".jpg";    
    
    cv::imwrite(pathWithExtension, frame);    
    
    return imagePath;
}

void EventType::removeObject(int objectIndex)
{
    if (std::find(illegalMovementDirectionObjectIndexes.begin(), illegalMovementDirectionObjectIndexes.end(), objectIndex) != illegalMovementDirectionObjectIndexes.end())
        illegalMovementDirectionObjectIndexes.erase(std::remove(illegalMovementDirectionObjectIndexes.begin(), illegalMovementDirectionObjectIndexes.end(), objectIndex), illegalMovementDirectionObjectIndexes.end());

    if (std::find(illegalOppositeDirectionObjectIndexes.begin(), illegalOppositeDirectionObjectIndexes.end(), objectIndex) != illegalOppositeDirectionObjectIndexes.end())
        illegalOppositeDirectionObjectIndexes.erase(std::remove(illegalOppositeDirectionObjectIndexes.begin(), illegalOppositeDirectionObjectIndexes.end(), objectIndex), illegalOppositeDirectionObjectIndexes.end());
    
    if (std::find(illegalOvertakingObjectIndexes.begin(), illegalOvertakingObjectIndexes.end(), objectIndex) != illegalOvertakingObjectIndexes.end())
        illegalOvertakingObjectIndexes.erase(std::remove(illegalOvertakingObjectIndexes.begin(), illegalOvertakingObjectIndexes.end(), objectIndex), illegalOvertakingObjectIndexes.end());

    if (std::find(enterToMonitoredAreaObjectIndexes.begin(), enterToMonitoredAreaObjectIndexes.end(), objectIndex) != enterToMonitoredAreaObjectIndexes.end())
        enterToMonitoredAreaObjectIndexes.erase(std::remove(enterToMonitoredAreaObjectIndexes.begin(), enterToMonitoredAreaObjectIndexes.end(), objectIndex), enterToMonitoredAreaObjectIndexes.end());

    if (std::find(potentiallyEnterToMonitoredAreaObjectIndexes.begin(), potentiallyEnterToMonitoredAreaObjectIndexes.end(), objectIndex) != potentiallyEnterToMonitoredAreaObjectIndexes.end())
        potentiallyEnterToMonitoredAreaObjectIndexes.erase(std::remove(potentiallyEnterToMonitoredAreaObjectIndexes.begin(), potentiallyEnterToMonitoredAreaObjectIndexes.end(), objectIndex), potentiallyEnterToMonitoredAreaObjectIndexes.end());
    
    if (std::find(unpermittedStoppingBeginObjectIndexes.begin(), unpermittedStoppingBeginObjectIndexes.end(), objectIndex) != unpermittedStoppingBeginObjectIndexes.end())
        unpermittedStoppingBeginObjectIndexes.erase(std::remove(unpermittedStoppingBeginObjectIndexes.begin(), unpermittedStoppingBeginObjectIndexes.end(), objectIndex), unpermittedStoppingBeginObjectIndexes.end());

    if (std::find(unpermittedStoppingEndObjectIndexes.begin(), unpermittedStoppingEndObjectIndexes.end(), objectIndex) != unpermittedStoppingEndObjectIndexes.end())
        unpermittedStoppingEndObjectIndexes.erase(std::remove(unpermittedStoppingEndObjectIndexes.begin(), unpermittedStoppingEndObjectIndexes.end(), objectIndex), unpermittedStoppingEndObjectIndexes.end());

    if (std::find(ilegalEnterToAreaObjectIndexes.begin(), ilegalEnterToAreaObjectIndexes.end(), objectIndex) != ilegalEnterToAreaObjectIndexes.end())
        ilegalEnterToAreaObjectIndexes.erase(std::remove(ilegalEnterToAreaObjectIndexes.begin(), ilegalEnterToAreaObjectIndexes.end(), objectIndex), ilegalEnterToAreaObjectIndexes.end());
    
    if (std::find(illegalPassingTheAreaObjectIndexes.begin(), illegalPassingTheAreaObjectIndexes.end(), objectIndex) != illegalPassingTheAreaObjectIndexes.end())
        illegalPassingTheAreaObjectIndexes.erase(std::remove(illegalPassingTheAreaObjectIndexes.begin(), illegalPassingTheAreaObjectIndexes.end(), objectIndex), illegalPassingTheAreaObjectIndexes.end());
    
    if (std::find(suspiciousBehaviorObjectIndexes.begin(), suspiciousBehaviorObjectIndexes.end(), objectIndex) != suspiciousBehaviorObjectIndexes.end())
        suspiciousBehaviorObjectIndexes.erase(std::remove(suspiciousBehaviorObjectIndexes.begin(), suspiciousBehaviorObjectIndexes.end(), objectIndex), suspiciousBehaviorObjectIndexes.end());

    if (std::find(busDetectedObjectIndexes.begin(), busDetectedObjectIndexes.end(), objectIndex) != busDetectedObjectIndexes.end())
        busDetectedObjectIndexes.erase(std::remove(busDetectedObjectIndexes.begin(), busDetectedObjectIndexes.end(), objectIndex), busDetectedObjectIndexes.end());

    if (std::find(unpermittedVehiclePassageIndexes.begin(), unpermittedVehiclePassageIndexes.end(), objectIndex) != unpermittedVehiclePassageIndexes.end())
        unpermittedVehiclePassageIndexes.erase(std::remove(unpermittedVehiclePassageIndexes.begin(), unpermittedVehiclePassageIndexes.end(), objectIndex), unpermittedVehiclePassageIndexes.end());
}

unsigned int EventType::getSequenceNumber()
{
    sequenceNumber++;
    if (sequenceNumber > 2147483647)
    {
        sequenceNumber = 0;
    }
    return sequenceNumber;
}
