/****************************************************************
*																*
*	VUT, AZD Praha s.r.o (Grant FV40372)						*
*																*
*   Autonomni system pro detekci rizikovych situaci v			*
*	doprave zalozeny na analyze obrazovych sekvenci				*
*																*
*	15.03.2020													*
*																*
*////////////////////////////////////////////////////////////////

/*
*	Popis projektu:
*	to do ... pridat ... popis
*
*	Preklad: Program prekladejte s nasledujicimi argumenty:
*	$ gcc -std=c99 -Wall -Wextra -Werror proj2.c -lm -o proj2
*
*	Syntax spusteni:
*	./AZDDetectionAndTrackingSystem --video "cesta_ku_videu" alebo ./AZDDetectionAndTrackingSystem --kamera
*
*   It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html
*/
//TODO:
//klouzavy prumer
//
//aproximovat polynomem
//
//pokud se hodnota hodne lisi zanedbat

// mereni rychlosti
//4572 mm = 40 mm
//ujelo 40 mm
//90 ms
//
//3, 3 cm je sirka 176, 9cm
//
//3 cm = 160
//
//
//(1, 60 / 0, 09) * 3, 6 = km / h
// (metry / cas) * 3,6

// uplatnìní, poèítání dopravy (nedostateèná stávající kapacita, organizace dopravy (správné naèasování semaforu) )

// 08.09.2021
// vyriesit structury, framy
// oznacenie aktualnej pozicie
// mazanie area

//Technický popis(Blokovka, jak to funguje, co s cim funguje, pouzite technologie)
//Uživatelsky manual(navod k pouziti, print screeny, popisy)
//Programatorsky manual(docsys)
//Instalaèní manuál




#if __has_include("cameraGrabModule.h")    
#include "cameraGrabModule.h"
#include "mutualJsonClient.h"
#include <unistd.h>

#else
#define NOMINMAX
#define SLOWCONNECTION
#endif

#include <fstream>
#include <sstream>
#include <iostream>
#include <thread>
#include <signal.h>
#include <cstdlib>
#include <ctime>
#include <time.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"
#include "DetectedObject.h"
#include "InferenceTimer.h"

#include "TrafficLight.h"
#include "EventType.h"
#include "outputFrame.h"
#include "utilities.h"
#include "systemThreads.h"
#include "config.h"
#include "LPReader.h"

#include "EventVideoThread.h"
#include "JsonSendThread.h"


#if __has_include("cameraGrabModule.h")    
#include "cameraGrabModule.h"
#include "mutualJsonClient.h"
#include <unistd.h>
#include "piau.h"
#define JETSON  1
#else
#define NOMINMAX
#include <windows.h>
#define JETSON  0
// void grabFrames(void); 
void grabFrames(threadQueue<OutputFrame>& queueFromCamera, int& fps) {}
void startStatusSending(){}
#endif

extern string pathToVideoFile;

extern int eventVideoDuration;

// Allow debug mode features
//bool isDebugMode = false;

// Allow live player
extern bool showFrames;

// Allow write video
extern bool writeVideoWithAnnotations;
extern bool writeVideoOriginal;

extern bool showGlobalFps;
extern bool showPartialFps;
extern bool showFramesResize;

extern string dnnModelConfigurationPathJson;
extern string dnnModelWeightsPathJson;
extern string dnnModelClassesPathJson;

extern string folderForRecordedVideosOriginal;
extern string folderForRecordedVideosAnnotated;

//bool saveAPicture = false;
//bool isPlaying = true;
bool isShowInferenceTimeAndFps = false;

// Open a video file or an image file or a camera stream.
std::string str;

extern cv::dnn::Net net;

extern bool isCuda;

Piau piau;
//LPReader lPReader;

int main(int argc, char** argv)
{
    threadQueue<VideoToEncode> videosToEncode;
    EventVideoThread eventVideoThread(videosToEncode, &stopGrabFrames, &cameraFps, &eventVideoDuration);
    std::thread videoEncodeThread;

    JsonSendThread jsonSendThread;
    std::thread jsonSendingThread;

    onLoad(eventVideoThread, jsonSendThread);    

    OutputFrame firstOutputFrame;

    threadQueue<OutputFrame> queueFromCamera;
    threadQueue<OutputFrame> queueFromDnn;
    threadQueue<OutputFrame> fromPostProcessQueue;


    queueFromCamera.queueName = "FromCamera";
    queueFromDnn.queueName = "FromDnn";
	fromPostProcessQueue.queueName = "ProcessQueue";

    lastFrameShowedTime = std::chrono::high_resolution_clock::now();    

    signal(SIGINT, onExitSignal);

    // Chceck 
    if (!isFileExist(dnnModelConfigurationPathJson)) {
        std::cout << "APPLICATION ERROR [Model configuration of neural network was not found : " << dnnModelClassesPathJson << "  ...]" << std::endl;
        return EXIT_FAILURE;
    }
    else if (!isFileExist(dnnModelWeightsPathJson)) {
        std::cout << "APPLICATION ERROR [Model weights of neural network was not found : " << dnnModelWeightsPathJson << " ...]" << std::endl;
        return EXIT_FAILURE;
    }
    else if (!isFileExist(dnnModelClassesPathJson)) {
        std::cout << "APPLICATION ERROR [Model classes file was not found: " << dnnModelClassesPathJson << " ...]" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        std::ifstream ifs(dnnModelClassesPathJson.c_str());
        std::string line;
        while (getline(ifs, line)) classes.push_back(line);
    }    

    // Load the network
    net = cv::dnn::readNetFromDarknet(dnnModelConfigurationPathJson, dnnModelWeightsPathJson);

    // Select Cuda or CPU for computing
    if (isCuda) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);       
    }
    else {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    // Chceck if Jetson is using    
    if (JETSON)
    {
        std::cout << "APPLICATION [Starting camera grabbing thread!]" << std::endl;
        t1 = std::thread(grabFrames, std::ref(queueFromCamera), std::ref(cameraFps));
        std::cout << "APPLICATION [Starting status message thread!]" << std::endl;
        statusSendingThread = piau.startStatusSending();
        statusSendingThread.detach();

        std::cout << "APPLICATION [Starting statistic message thread!]" << std::endl;
        statisticSendingThread = startStatisticSending();
        statisticSendingThread.detach();
    }
    else {
        if (!isFileExist(pathToVideoFile)) {
            std::cout << "APPLICATION ERROR [Video file was not found ...]" << std::endl;
            return EXIT_FAILURE;
        }
        else {
            std::cout << "APPLICATION [Openning video file!]" << std::endl;
            cap.open(pathToVideoFile);
            //cap.set(cv::CAP_PROP_POS_MSEC, 3710000);
        }
    }
    
    std::cout << "APPLICATION [Starting netForwardThread !]" << std::endl;
    netForwardThread = std::thread(runNetForward, std::ref(queueFromDnn),std::ref(queueFromCamera), std::ref(net), JETSON, std::ref(cap), std::ref(stopGrabFrames)); 
    
    std::cout << "APPLICATION [Starting postProcessThread !]" << std::endl;
    postProcessThread = std::thread(runPostProcess, std::ref(queueFromDnn), std::ref(fromPostProcessQueue), std::ref(cameraFps), std::ref(stopGrabFrames), std::ref(showPartialFps), std::ref(classes), std::ref(isDebugMode));
       
    std::cout << "APPLICATION [Starting drawThread !]" << std::endl;
    drawQueueThread = std::thread(runDraw, std::ref(fromPostProcessQueue), std::ref(eventVideoThread), JETSON, std::ref(showFrames), std::ref(stopGrabFrames), std::ref(showGlobalFps), std::ref(isDebugMode), std::ref(isShowInferenceTimeAndFps), std::ref(cap), std::ref(videoWriterWithAnnotations), std::ref(videoWriterOriginal));


    videoEncodeThread = eventVideoThread.runVideoEncode();
    jsonSendingThread = jsonSendThread.runJsonSending();


    if (writeVideoWithAnnotations)
    {     
        if (!JETSON) {       
            frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            frameHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

            //std::string folderForRecordedVideos = "";
            if (isFileExist(folderForRecordedVideosAnnotated))
            {
                std::cout << "APPLICATION [Starting recording video with annotations to: " << folderForRecordedVideosAnnotated << " !]" << std::endl;
                videoWriterWithAnnotations = eventVideoThread.createWindowsVideoWriter(folderForRecordedVideosAnnotated, frameWidth, frameHeight);
            }
            else {
                std::cout << "APPLICATION [Can't start recording! Folder: " << folderForRecordedVideosAnnotated << " not exists!]" << std::endl;
            }
            
        }
        else {
            
            while (firstOutputFrame.frameOriginal.cols <= 0)
            {                   
                firstOutputFrame = queueFromCamera.pop();
            }
            
            newFrame = firstOutputFrame.frameForDisplay;            
            frameWidth = newFrame.cols;            
            frameHeight = newFrame.rows;
            //std::string folderForRecordedVideos = "/mnt/nvme/record/areas/multipleRecorders/annotations/";
            if (isFileExist(folderForRecordedVideosAnnotated))
            {
                std::cout << "APPLICATION [Starting recording video with annotations to: " << folderForRecordedVideosAnnotated << " !]" << std::endl;
                videoWriterWithAnnotations = eventVideoThread.createJetsonVideoWriter(folderForRecordedVideosAnnotated, frameWidth, frameHeight, 600000000000.0);
            }
            else {
                std::cout << "APPLICATION [Can't start recording! Folder: " << folderForRecordedVideosAnnotated << " not exists!]" << std::endl;
            }
            
        }     
    }
    if (writeVideoOriginal)
    {
        if (!JETSON) {           
            frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            frameHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
               
            //std::string pathToVideoWriterFile = "moravany2.avi";
            if (isFileExist(folderForRecordedVideosOriginal))
            {
                std::cout << "APPLICATION [Starting recording original video to: " << folderForRecordedVideosOriginal << " !]" << std::endl;
                videoWriterOriginal.open(folderForRecordedVideosOriginal, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), (int)cap.get(cv::CAP_PROP_FPS), cv::Size(int((frameWidth / frameHeight)) * 1024, 1024), true);
            }
            else {
                std::cout << "APPLICATION [Can't start recording! Folder: " << folderForRecordedVideosOriginal << " not exists!]" << std::endl;
            }
            
        }
        else {            
            while (firstOutputFrame.frameOriginal.cols <= 0)
            {                
                firstOutputFrame = queueFromCamera.pop();
                std::cout << firstOutputFrame.frameOriginal.cols << std::endl;
            }
            frameWidth = firstOutputFrame.frameOriginal.cols;
            frameHeight = firstOutputFrame.frameOriginal.rows;
            
            //std::string folderForRecordedVideos = "/mnt/nvme/record/areas/multipleRecorders/original/";
            if (isFileExist(folderForRecordedVideosOriginal))
            {
                std::cout << "APPLICATION [Starting recording original video to: " << folderForRecordedVideosOriginal << " !]" << std::endl;
                videoWriterOriginal = eventVideoThread.createJetsonVideoWriter(folderForRecordedVideosOriginal, frameWidth, frameHeight, 600000000000.0);
            }
            else {
                std::cout << "APPLICATION [Can't start recording! Folder: " << folderForRecordedVideosOriginal << " not exists!]" << std::endl;
            }
            
        }
    } 

    // wait until exit program has been called
    while (!stopGrabFrames)
    {       
        sleep(1);
    }

    // properly exit threads
    netForwardThread.join();      
    postProcessThread.join();    
    drawQueueThread.join();
    videoEncodeThread.join();

    if(JETSON)
    {       
        t1.join();        
    }        
    
    if (!JETSON) {
        cap.release();
    }
    if (writeVideoWithAnnotations)
    {
        videoWriterWithAnnotations.release();
        std::cout << "APPLICATION [Video writer saved!]" << std::endl;
    }
    if (writeVideoOriginal)
    {
        videoWriterOriginal.release();
        std::cout << "APPLICATION [Video writer saved!]" << std::endl;
    }   
    net = cv::dnn::Net();
    return 0;
}