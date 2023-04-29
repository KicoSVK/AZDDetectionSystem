#ifndef SYSTEMTHREADS_H_
#define SYSTEMTHREADS_H_
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
//#include "IndexController.h"
#include "utilities.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "config.h"
#include <fstream>


extern bool showPartialFps;
extern bool bufferAndSendVideo;
extern bool showFramesResize;
extern bool writeVideoWithAnnotations;
extern bool writeVideoOriginal;
extern bool isDebugMode;

/**
* Function for run thread for DNN frame pass
* @param threadQueue<OutputFrame*>& dnnQueue is queue for OutputFrames returned from DNN
* @param threadQueue<OutputFrame*>& queueFromCamera is queue for OutputFrames returned from cameraGrabModule (camera)
* @param cv::dnn::Net& netDnn is ready DNN
* @param int JETSON is 0 if program not running io Jetson an 1 if program is running on Jetson
* @param cv::VideoCapture& cap is video for processing if program not running on Jetson
* @param bool& stopGrabFrames is used for stop thread when exit program is called
*/
void runNetForward(threadQueue<OutputFrame>& dnnQueue, threadQueue<OutputFrame>& queueFromCamera, cv::dnn::Net& netDnn, int JETSON, cv::VideoCapture& cap, bool& stopGrabFrames);

/**
* Function for postprocess run (Remove the bounding boxes with low confidence using non-maxima suppression)
* @param threadQueue<OutputFrame*>& dnnQueue is queue for OutputFrames returned from DNN
* @param threadQueue<OutputFrame*>& fromPostProcessQueue is queue for OutputFrames returned from postProcess
* @param int& cameraFps is FPS for which is set camera
* @param bool& stopGrabFrames is used for stop thread when exit program is called
* @param bool& showPartialFps is used for show/hide FPS of majority program parts
* std::vector<std::string>& classes is DNN classes
* bool& isDebugMode used for turn on/off debug mode
*/

void runPostProcess(threadQueue<OutputFrame>& dnnQueue, threadQueue<OutputFrame>& fromPostProcessQueue, int& cameraFps, bool& stopGrabFrames, bool& showPartialFps, std::vector<std::string>& classes, bool& isDebugMode);

/**
* Function for run user interactions and rendering
* @param threadQueue<OutputFrame*>& fromPostProcessQueue is queue for OutputFrames returned from postProcess
* @param int JETSON is 0 if program not running io Jetson an 1 if program is running on Jetson
* @param bool& showFrames is for turn ON/OFF showing live camera window with annotations
* @param bool& stopGrabFrames is used for stop thread when exit program is called
* bool& showGlobalFps is used for show/hide FPS of last part of program
* std::vector<cv::Rect>& detectedCarRect is vector with detected car rectangles
* bool& isDebugMode used for turn on/off debug mode
* bool& isShowInferenceTimeAndFps used for show/hide FPS
* cv::VideoCapture& cap is video for processing if program not running on Jetson
* cv::VideoWriter& videoWriterWithAnnotations is used for write video with annotations
* cv::VideoWriter& videoWriterOriginal is used for write original video from camera
*/
//void runDraw(threadQueue<OutputFrame>& fromPostProcessQueue, int JETSON, bool& showFrames, bool& stopGrabFrames, bool& showGlobalFps, bool& isDebugMode, bool& isShowInferenceTimeAndFps, cv::VideoCapture& cap, cv::VideoWriter& videoWriterWithAnnotations, cv::VideoWriter& videoWriterOriginal);
//void runDraw(threadQueue<OutputFrame>& fromPostProcessQueue, EventVideoThread& eventVideoThread, int JETSON, bool& showFrames, bool& stopGrabFrames, bool& showGlobalFps, std::vector<cv::Rect>& detectedCarRect, bool& isDebugMode, bool& isShowInferenceTimeAndFps, cv::VideoCapture& cap, cv::VideoWriter& videoWriterWithAnnotations, cv::VideoWriter& videoWriterOriginal);

void runDraw(threadQueue<OutputFrame>& fromPostProcessQueue, EventVideoThread& eventVideoThread, int JETSON, bool& showFrames, bool& stopGrabFrames, bool& showGlobalFps, bool& isDebugMode, bool& isShowInferenceTimeAndFps, cv::VideoCapture& cap, cv::VideoWriter& videoWriterWithAnnotations, cv::VideoWriter& videoWriterOriginal);

/**
* Function for run thread for DNN frame pass
* @param threadQueue<VideoToEncode>& videosToEncodeQueue is queue for video for encode
* @param threadQueue<OutputFrame*>& queueFromCamera is queue for OutputFrames returned from cameraGrabModule (camera)
*/
void runVideoEncode(threadQueue<VideoToEncode>& videosToEncodeQueue);

/**
* Function for user interection with GUI
* @param cv::Mat frame
* @param int key is pressed keyboard key
*/
void keypressController(cv::Mat frame, int key);

#endif