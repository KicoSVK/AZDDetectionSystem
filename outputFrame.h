#ifndef OUTPUTFRAME_H_
#define OUTPUTFRAME_H_

#include <opencv2/highgui.hpp>
#include "DetectedObject.h"
#include "TrafficLight.h"

/** @struct OuputFrame
 *  @brief This structure stored the original camera image, subsampled images, detected objects, and network outputs. Structure is used for data transfer between individual parts of the program.
 *  @var cv::Mat frameOriginal
 *  Matrix 'frameOriginal' is used for store original frame captured from camera
 *  @var cv::Mat frameForDnn
 *  Matrix 'frameForDnn' is used for store resized frame for DNN
 *  @var cv::Mat frameForProcessing
 *  Matrix 'frameForProcessing' is used for store resized frame for processing (event evaluation ...)
 *  @var cv::Mat frameForDisplay
 *  Matrix 'frameForDisplay' is used for store resized frame for display and encoding video with annotations (bounding boxes, text informations,...)
 *  @var std::vector<cv::Mat> outs
 *  Vector of matrixes 'outs' is used for store outputs from DNN
 *  @var std::vector<DetectedObject> detectedObjectsInScene
 *  Vector of DetectedObject 'detectedObjectsInScene' is used for store detected objects on actual frame
 */
typedef struct {
    // Store original frame (width, height)
    cv::Mat frameOriginal;

    // Store frame for DNN (dnn frame resolution)
    cv::Mat frameForDnn;

    // Store frame for processing
    cv::Mat frameForProcessing;

    // Store frame for display
    cv::Mat frameForDisplay;

    // Store outputs from DNN model
    std::vector<cv::Mat> outs;

    // Vector stores all important detected objects (like vehicles ...)
    std::vector<DetectedObject> detectedObjectsInScene;

    // Vector stores detected trafficLights
    std::vector<TrafficLight> trafficLights;

    // Vector stores all detected indexes
    std::vector<int> detectedIndexes;

} OutputFrame;

#endif