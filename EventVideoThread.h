#pragma once
#ifndef EVENTVIDEOTHREAD_H_
#define EVENTVIDEOTHREAD_H_

#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"
#include <iostream>
#include "threadQueue.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <iomanip>
#include <fstream>


typedef struct {
    std::vector<cv::Mat> imagesToEncode;
    std::string filePathName;
    int TTL;
}VideoToEncode;

class EventVideoThread {

public:
    //EventVideoThread();
    EventVideoThread() : videosToEncodeQueue(defaultQueue) {}    
    EventVideoThread(threadQueue<VideoToEncode>& videosToEncodeQueueNew, bool* newStopGrabFrames, int* newCameraFps, int *newEventVideoDuration) : videosToEncodeQueue(videosToEncodeQueueNew), stopGrabFrames(newStopGrabFrames), cameraFps(newCameraFps), eventVideoDuration(newEventVideoDuration) {}
    std::vector<VideoToEncode> getVideosToEncode();
    void decreaseCounterForVideosToEncode();
    //void addToVideosToEncode(VideoToEncode videoToEncode);
    void addToCounterForVideosToEncode(VideoToEncode videoToEncode);
    int getNumOfVideosInCounter();
    int* cameraFps;
    bool bufferReady = false;
    int* eventVideoDuration;

    threadQueue<VideoToEncode>& videosToEncodeQueue;
    threadQueue<VideoToEncode> defaultQueue;
    std::thread runVideoEncode();

    cv::VideoWriter createWindowsVideoWriter(std::string folderForVideoRecorder, int videoWidth, int videoHeight);
    cv::VideoWriter createJetsonVideoWriter(std::string folderForVideoRecorder, int videoWidth, int videoHeight, double nsSplit = 0);
    cv::VideoWriter createJetsonVideoWriter(std::string folderForVideoRecorder, std::string videoName, int videoWidth, int videoHeight, double nsSplit = 0);
    void bufferImage(cv::Mat frame);
    std::string* objectNames;
   
private:  
    mutable std::mutex _mutexForCounterForVideosToEncode;
    std::vector<VideoToEncode> counterForVideosToEncode; //!< All waiting videos for get images after event
    bool* stopGrabFrames;
    mutable std::mutex _mutexForBufferedImages;
    std::vector<cv::Mat> bufferedImages;

    void videoEncode();
    void createFolder(std::string folderName);
    
    std::vector<cv::Mat> getBufferedImages();
};


#endif