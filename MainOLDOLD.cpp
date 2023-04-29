// This code is written at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html

// Usage example:  ./object_detection_yolo.out --video=run.mp4
//                 ./object_detection_yolo.out --image=bird.jpg
#include <fstream>
#include <sstream>
#include <iostream>

#include <ctime>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

const char* keys =
"{help h usage ? | | Usage examples: \n\t\t./object_detection_yolo.out --image=dog.jpg \n\t\t./object_detection_yolo.out --video=run_sm.mp4}"
"{image i        |<none>| input image   }"
"{video v       |<none>| input video   }"
;
using namespace cv;
using namespace dnn;
using namespace std;

// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;  // Width of network's input image
int inpHeight = 416; // Height of network's input image
vector<string> classes;

vector<Rect> objectsRect;
vector<int> objectsIndex;
vector<String> objectsName;
vector<float> objectsConfidences;
vector<vector<cv::Point>> objectsRoute;

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat& frame, Mat& fgMask, const vector<Mat>& out);

// Draw the predicted bounding box
void drawPred(int index, String objectName, float conf, int left, int top, int right, int bottom, Mat& frame);

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net);

Rect2d roiRectangle;
bool isCrossRoadClosed;
bool isBackgroundSelected = false;
Mat overlay;
bool isRedColorToStartTimeDuration = false;

std::clock_t start;
double duration;
bool isNormalState = true;

bool isDisplayedMainMenu = false;
bool isDisplayedMonitoredAreaMenu = false;
bool isDisplayedTrafficAreaMenu = false;

//create Background Subtractor objects
Ptr<BackgroundSubtractor> pBackSub;

bool isSelectedRoiArea = false;

// vector for monitored area
vector<cv::Point> monitoredArea;

// current index of point in monitoring area
int indexOfActivePointInMonitoringArea = -1;

// current index of point in traffic area
int indexOfActivePointInTrafficArea = -1;

bool isClickedMonitoredArea = false;

// vector for traffic
vector<vector<cv::Point>> trafficArea;

// index of active/current traffic line
int indexOfActiveTrafficArea = 0;

Mat frame, fgMask;

int getIndexOfNearestPointInMonitoredArea(Point point) {
    for (size_t i = 0; i < monitoredArea.size(); i++)
    {
        if (monitoredArea[i].x - 25 < point.x && point.x < monitoredArea[i].x + 25 && monitoredArea[i].y - 25 < point.y && point.y < monitoredArea[i].y + 25) {
            return i;
        }
    }
    return -1;
}

int getIndexOfNearestPointInTrafficArea(Point point) {
    if (trafficArea[indexOfActiveTrafficArea].size() < 100) {
        for (size_t i = 0; i < trafficArea[indexOfActiveTrafficArea].size(); i++)
        {
            if (trafficArea[indexOfActiveTrafficArea][i].x - 25 < point.x && point.x < trafficArea[indexOfActiveTrafficArea][i].x + 25 && trafficArea[indexOfActiveTrafficArea][i].y - 25 < point.y && point.y < trafficArea[indexOfActiveTrafficArea][i].y + 25) {
                return i;
            }
        }
        return -1;
    }
    return -1;
}

int getIndexOfTrafficArea(Point point) {
    for (size_t i = 0; i < trafficArea.size(); i++)
    {
        if (pointPolygonTest(trafficArea[i], point, false) > 0) {
            return i;
        }
    } 
    return -1;
}

// call back function for define monitored and traffic area
void mouseEvent(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        if (isDisplayedMonitoredAreaMenu == true) {
            //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
            if (getIndexOfNearestPointInMonitoredArea(Point(x, y)) != -1) {
                indexOfActivePointInMonitoringArea = getIndexOfNearestPointInMonitoredArea(Point(x, y));
            }
            else {
                monitoredArea.push_back(Point(x, y));
            }
        }
        else if (isDisplayedTrafficAreaMenu == true) {
            if (trafficArea.size() > 0 && getIndexOfTrafficArea(Point(x, y)) != -1) {
                indexOfActiveTrafficArea = getIndexOfTrafficArea(Point(x, y));
            }
            else if (trafficArea.size() > 0 && getIndexOfNearestPointInTrafficArea(Point(x, y)) != -1) {
                indexOfActivePointInTrafficArea = getIndexOfNearestPointInTrafficArea(Point(x, y));
            }
            else if (trafficArea.size() > 0) {
                if (trafficArea[indexOfActiveTrafficArea].size() < 100) {
                    trafficArea[indexOfActiveTrafficArea].push_back(Point(x, y));
                }
                else {
                    vector<cv::Point> trafficLine;
                    trafficLine.push_back(Point(x, y));
                    trafficArea.push_back(trafficLine);
                }
            }
            else {
                vector<cv::Point> trafficLine;
                trafficLine.push_back(Point(x, y));
                trafficArea.push_back(trafficLine);
            }
        }
    }

    else if (event == EVENT_MOUSEMOVE) {
        if (isDisplayedMonitoredAreaMenu == true && monitoredArea.size() > 0 && indexOfActivePointInMonitoringArea != -1) {
            monitoredArea[indexOfActivePointInMonitoringArea] = Point(x, y);
        }
        else if (isDisplayedTrafficAreaMenu == true && trafficArea.size() > 0 && indexOfActivePointInTrafficArea != -1) {
            trafficArea[indexOfActiveTrafficArea][indexOfActivePointInTrafficArea] = Point(x, y);
        }
    }

    if (event == EVENT_LBUTTONUP) {    
        if (isDisplayedMonitoredAreaMenu == true && monitoredArea.size() > 0 && indexOfActivePointInMonitoringArea != -1) {
            indexOfActivePointInMonitoringArea = -1;
        }
        else if (isDisplayedTrafficAreaMenu == true && trafficArea.size() > 0 && indexOfActivePointInTrafficArea != -1) {
            indexOfActivePointInTrafficArea = -1;
        }
    }
}

cv::Scalar getScalarColor(int index) {
    switch (index)
    {
        case 0:
            return Scalar(0, 0, 255);
        break;
        case 1:
            return Scalar(0, 255, 0);
        break;

        case 2:
            return Scalar(255, 0, 0);
        break;

        case 3:
            return Scalar(0, 255, 255);
        break;

        case 4:
            return Scalar(255, 0, 255);
        break;

        case 5:
            return Scalar(255, 255, 0);
        break;

        case 6:
            return Scalar(0, 0, 100);
        break;
        case 7:
            return Scalar(0, 100, 0);
        break;
        case 8:
            return Scalar(100, 0, 0);
        break;
        default:
            return Scalar(255, 255, 255);
        break;
    }
}

int main(int argc, char** argv)
{
    pBackSub = createBackgroundSubtractorMOG2(350, 32, false);

    CommandLineParser parser(argc, argv, keys);
    parser.about("Use this script to run object detection using YOLO3 in OpenCV.");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    // Load names of classes
    string classesFile = "coco.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    // Give the configuration and weight files for the model
    String modelConfiguration = "yolov3.cfg";
    String modelWeights = "yolov3.weights";

    // Load the network
    Net net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);

    // Open a video file or an image file or a camera stream.
    string str, outputFile;
    VideoCapture cap;
    VideoWriter video;
    //Mat frame, fgMask, blob;
    Mat blob;

    //try {

    //    outputFile = "yolo_out_cpp.avi";
    //    if (parser.has("image"))
    //    {
    //        // Open the image file
    //        str = parser.get<String>("image");
    //        ifstream ifile(str);
    //        if (!ifile) throw("error");
    //        cap.open(str);
    //        str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.jpg");
    //        outputFile = str;
    //    }
    //    else if (parser.has("video"))
    //    {
    //        // Open the video file
    //        str = parser.get<String>("video");
    //        ifstream ifile(str);
    //        if (!ifile) throw("error");
    //        cap.open(str);
    //        str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.avi");
    //        outputFile = str;
    //    }
    //    // Open the webcaom
    //    else cap.open(parser.get<int>("device"));

    //}
    //catch (...) {
    //    cout << "Could not open the input image/video stream" << endl;
    //    return 0;
    //}
    //cap.open("W:/Moravany/18-2-2020/Basler acA2440-35uc (23064099)_20200218_172954596.mp4");
    cap.open("W:/Krizikova/Disk 1/converted/testovaci/20190723_135648-1.m4v");
    //cap.open("W:/Moravany/18-2-2020/Basler_acA2440-35uc_(23064099)_20200218_163844581.mp4");

    // Get the video writer initialized to save the output video
    if (!parser.has("image")) {
        video.open(outputFile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));
    }

    // Create a window
    static const string kWinName = "AZD detection system";
    namedWindow(kWinName, WINDOW_AUTOSIZE);

    vector<vector<cv::Point>> contours; // Vector for storing contour
    vector<Vec4i> hierarchy;

    // Process frames.
    while (1)
    {
        // get frame from the video
        cap >> frame;
        resize(frame, frame, Size(frame.cols / 2, frame.rows / 2));


        //cv::Mat diffImage, backgroundFrame, fgMask;

        //if (isBackgroundSelected == false) {
        //    frame.copyTo(backgroundFrame);
        //    isBackgroundSelected = true;
        //}
        //else
        //{
        //    cv::absdiff(backgroundFrame, frame, diffImage);
        //    cv::Mat foregroundMask = cv::Mat::zeros(diffImage.rows, diffImage.cols, CV_8UC1);
        //    imshow("foreground", foregroundMask);
        //}

        //update the background model
        pBackSub->apply(frame, fgMask);
        imshow("mask", fgMask);

        findContours(fgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<Rect> detectedObjects;

        // get key press
        int key = cv::waitKey(30);

        // tab
        if (key == 9 || key == 109) {
            if (isDisplayedMainMenu == true)
                isDisplayedMainMenu = false;
            else
                isDisplayedMainMenu = true;
        }
        else if (key == 13) {

        }
        else if (key == 115) {
            ofstream myfile;
            myfile.open("conf.txt");
            if (monitoredArea.size() > 0) {
                myfile << "nm" << "\n";
                for (size_t i = 0; i < monitoredArea.size(); i++)
                {
                    myfile << "m" << monitoredArea[i].x << "/" << monitoredArea[i].y << "\n";
                }
            }

            if (trafficArea.size() > 0) {
                for (size_t i = 0; i < trafficArea.size(); i++)
                {
                    myfile << "nt" << "\n";
                    for (size_t j = 0; j < trafficArea[i].size(); j++)
                    {
                        myfile << "t" << trafficArea[i][j].x << "/" << trafficArea[i][j].y << "\n";
                    }
                }
            }
            
            myfile.close();
        }
        else if (key == 108) {
            string line;
            ifstream myfile("conf.txt");
            if (myfile.is_open())
            {
                bool isNewMonitoredArea = false;
                bool isNewTrafficArea = false;
                bool isXOrdinateLoaded = false;

                int objectIndex = -1;

                string ordinateX = "";
                string ordinateY = "";

                while (getline(myfile, line))
                {
                    if (line[0] == 'n' && line[1] == 'm') {
                        isNewMonitoredArea = true;
                    }
                    else if (line[0] == 'n' && line[1] == 't') {
                        isNewTrafficArea = true;
                    }
                    else {
                        if (isNewMonitoredArea) {
                            for (size_t i = 0; line[i] != '\0'; i++)
                            {
                                if (line[i] == '/') {
                                    isXOrdinateLoaded = true;
                                }

                                if (isXOrdinateLoaded == false)
                                    ordinateX = ordinateX + line[i];
                                else
                                    ordinateY = ordinateY + line[i];
                            }
                            monitoredArea.push_back((Point(std::stoi(ordinateX), std::stoi(ordinateY))));

                            isNewMonitoredArea = false;
                            isXOrdinateLoaded = false;

                            ordinateX = "";
                            ordinateY = "";
                        }
                        else if (isNewTrafficArea)
                        {
                            for (size_t i = 0; line[i] != '\0'; i++)
                            {
                                if (line[i] == '/') {
                                    isXOrdinateLoaded = true;
                                }

                                if (isXOrdinateLoaded == false)
                                    ordinateX = ordinateX + line[i];
                                else
                                    ordinateY = ordinateY + line[i];
                            }
                            monitoredArea.push_back((Point(std::stoi(ordinateX), std::stoi(ordinateY))));

                            isNewTrafficArea = false;
                            isXOrdinateLoaded = false;

                            ordinateX = "";
                            ordinateY = "";
                        }
                    }
                cout << line << '\n';
                }
                myfile.close();
            }
            else {
                cout << "can not open or read file";
            }
        }
        // space
        //else if (key == 32) {
        //}
        // key z/y
        else if (key == 122 || key == 121) {
            isDisplayedMainMenu = false;
            isDisplayedTrafficAreaMenu = false;

            if (isDisplayedMonitoredAreaMenu == true)
                isDisplayedMonitoredAreaMenu = false;
            else
                isDisplayedMonitoredAreaMenu = true;

            //cv::putText(frame, "- for exit from help menu, press H", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);

            // main info message in the bottom of window
            //cv::putText(frame, "Press space for accept selection", cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);

            float alpha = 0.5;
            frame.copyTo(overlay);
            //roiRectangle = selectROI(frame, true, false);

            while (waitKey(1) < 0)
            {
                setMouseCallback(kWinName, mouseEvent, NULL);
                if (monitoredArea.size() != 0) {
                    cv::fillConvexPoly(frame, monitoredArea, Scalar(100, 255, 100));
                    for (size_t i = 0; i < monitoredArea.size(); i++)
                    {
                        cv::circle(frame, monitoredArea[i], 8, Scalar(255, 0, 0), -1);
                    }
                    //rectangle(frame, Rect(100,100,1000,1000), Scalar(0, 0, 255), -1);
                    addWeighted(overlay, alpha, frame, 1-alpha, 0, frame, 0);                
                }
                //isSelectedRoiArea = true;
                imshow(kWinName, frame);
            }
        }
        // key x
        else if (key == 120) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaMenu = false;

            if (isDisplayedTrafficAreaMenu == true)
                isDisplayedTrafficAreaMenu = false;
            else
                isDisplayedTrafficAreaMenu = true;

            //if (indexOfActiveTrafficArea == -1)
            //    indexOfActiveTrafficArea = 0;

            //cv::putText(frame, "- for exit from help menu, press H", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);

            // main info message in the bottom of window
            //cv::putText(frame, "Press space for accept selection", cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);

            float alpha = 0.5;
            frame.copyTo(overlay);
            //roiRectangle = selectROI(frame, true, false);

            while (isDisplayedTrafficAreaMenu)
            {
                // accept active traffic area and continue to next traffic area
                int key = waitKey(1);
                if (key == 32) {
                    indexOfActiveTrafficArea++;
                    cout << "S P A C E" << endl;
                }
                // accept selected areas and exit to main screen
                if (key == 27 || key == 13) {
                    isDisplayedMainMenu = false;
                    isDisplayedMonitoredAreaMenu = false;
                    isDisplayedTrafficAreaMenu = false;
                    break;
                }
                // clear active traffic area
                if (key == 99) {
                    cout << "CLEAR" << endl;
                    if (indexOfActiveTrafficArea != -1) {
                        if (trafficArea[indexOfActiveTrafficArea].size() < 100) {
                            trafficArea[indexOfActiveTrafficArea].clear();
                            trafficArea.erase(trafficArea.begin() + indexOfActiveTrafficArea);
                            indexOfActiveTrafficArea--;
                        }
                    }
                    if (indexOfActiveTrafficArea < 0)
                        indexOfActiveTrafficArea = -1;
                }

                setMouseCallback(kWinName, mouseEvent, NULL);

                if (trafficArea.size() != 0) {
                    for (size_t i = 0; i < trafficArea.size(); i++)
                    {
                        Scalar scalarColor = getScalarColor(i);
                        cv::fillConvexPoly(frame, trafficArea[i], scalarColor);
                        if (trafficArea[indexOfActiveTrafficArea].size() < 100) {
                            for (size_t j = 0; j < trafficArea[indexOfActiveTrafficArea].size(); j++)
                            {
                                cv::circle(frame, trafficArea[indexOfActiveTrafficArea][j], 8, cv::Scalar(255, 0, 0), -1);
                            }
                        }
                    }
                    //rectangle(frame, Rect(100,100,1000,1000), Scalar(0, 0, 255), -1);
                    addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
                }
                //isSelectedRoiArea = true;
                imshow(kWinName, frame);
            }
        }
        // key c
        else if (key == 99) {
            if (isDisplayedMonitoredAreaMenu) {
                trafficArea[indexOfActiveTrafficArea].clear();
            }

            if (isDisplayedTrafficAreaMenu) {
                trafficArea[indexOfActiveTrafficArea].clear();
            }
        }
        // ESC
        else if (key == 27) {
            isDisplayedMainMenu = false;
            isDisplayedMonitoredAreaMenu = false;
            isDisplayedTrafficAreaMenu = false;
        }

        //if (key == 32) {

        //    float alpha = 0.5;
        //    frame.copyTo(overlay);
        //    //roiRectangle = selectROI(frame, true, false);
        //    while (waitKey(1) < 0)
        //    {
        //        setMouseCallback(kWinName, CallBackFunc, NULL);
        //        if (roiAreaPoints.size() != 0) {
        //            cv::fillConvexPoly(frame, roiAreaPoints, Scalar(100, 255, 100));
        //            //rectangle(frame, Rect(100,100,1000,1000), Scalar(0, 0, 255), -1);
        //            addWeighted(overlay, alpha, frame, 1-alpha, 0, frame, 0);
        //            cv::putText(frame, "Press space for accept selection", cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(0, 255, 255), 1.5);
        //            imshow(kWinName, frame);
        //        }
        //        isSelectedRoiArea = true;
        //    }
        //}
        //else if (isSelectedRoiArea == false) {
        //    cv::putText(frame, "Press space for select border", cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(0, 255, 255), 1.5);
        //}

        // Stop the program if reached end of video
        if (frame.empty()) {
            cout << "Done processing !!!" << endl;
            cout << "Output file is stored as " << outputFile << endl;
            waitKey(3000);
            break;
        }
        // Create a 4D blob from a frame.
        blobFromImage(frame, blob, 1 / 255.0, Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);

        //Sets the input to the network
        net.setInput(blob);

        // Runs the forward pass to get output of the output layers
        vector<Mat> outs;
        net.forward(outs, getOutputsNames(net));

        // Remove the bounding boxes with low confidence
        postprocess(frame, fgMask, outs);

        // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time for a frame : %.2f ms", t);
        putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

        // Write the frame with the detection boxes
        Mat detectedFrame;
        overlay = frame.clone();

        frame.convertTo(detectedFrame, CV_8U);
        if (parser.has("image")) imwrite(outputFile, detectedFrame);
        else video.write(detectedFrame);

        if (monitoredArea.size() > 0) {

            //for (int i = 0; i < detectedObjects.size(); i++)
            //{
            //    pointPolygonTest()
            //}
            //rectangle(frame, boundingRect, Scalar(0, 0, 255), 2);

            float alpha = 0.8;
            isNormalState = true;

            // display all traffic lines
            for (size_t i = 0; i < trafficArea.size(); i++)
            {
                Scalar scalarColor = getScalarColor(i);
                cv::fillConvexPoly(frame, trafficArea[i], scalarColor, 0);
                addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);
            }

            // display monitored area
            if (isCrossRoadClosed == true) {
                cv::fillConvexPoly(frame, monitoredArea, Scalar(0, 0, 255));
                //rectangle(frame, Rect(100,100,1000,1000), Scalar(0, 0, 255), -1);
                addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);             

                for (int i = 0; i < contours.size(); i++) // iterate through each contour.
                {
                    double area = contourArea(contours[i], false);  //  Find the area of contour
                    if (area > 200) {
                        Rect rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
                        //drawContours(frame, contours, i, Scalar(0, 255, 0), 2);
                        //detectedObjects.push_back(rect);
                        if (pointPolygonTest(monitoredArea, Point(1,1), false) > 0) {
                            rectangle(frame, rect, Scalar(0, 0, 255), 4);
                            isNormalState = false;
                        }
                    }
                }
            }
            else {
                cv::fillConvexPoly(frame, monitoredArea, Scalar(0, 255, 0));
                addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame, 0);            
            }

            if (isNormalState == true) {
                cv::putText(frame, "State: Normal", cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(0, 255, 0), 1.5);
            }
            else {
                cv::putText(frame, "State: risk assessment", cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(255, 0, 0), 1.5);
            }
        }

        if (isDisplayedMainMenu) {
            int spaceBettewnLines = 15;
            cv::putText(frame, "Application menu:", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.8);
            cv::putText(frame, "- for define monitored area, press Y/Z", cv::Point(10, 45), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- for define traffic border, press X", cv::Point(10, 65), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- for save configuration, press S", cv::Point(10, 85), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- for load configuration, press L", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);

            //cv::putText(frame, "- for exit from help menu, press H", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);

            // main info message in the bottom of window
            cv::putText(frame, "Press ESC for exit from menu", cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);
        }
        else if (isDisplayedMonitoredAreaMenu) {
            cv::putText(frame, "Monitored area:", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.8);
            cv::putText(frame, "- define monitored area by mouse clicks", cv::Point(10, 45), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- redefine monitored area by mouse with drag and drop function", cv::Point(10, 65), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- for accept selection, press space", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
        }
        else if (isDisplayedTrafficAreaMenu) {
            cv::putText(frame, "Traffic area:", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.8);
            cv::putText(frame, "- define traffic area by mouse clicks", cv::Point(10, 45), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- redefine traffic area by mouse with drag and drop function", cv::Point(10, 65), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
            cv::putText(frame, "- for accept selection, press space", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
        }
        else {
            // main info message in the bottom of window
            cv::putText(frame, "Press M for insert to menu", cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);
        }
        
        imshow(kWinName, frame);
        //waitKey();
    }

    cap.release();
    if (!parser.has("image")) video.release();

    return 0;
}



Point getCenterOfRect(Rect rect) {
    return cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

//int getNearestDistance(Rect2d boxes) {
    //Point objectCenterCurrent = getCenterOfRect(boxes);
    ////Point objectCenterCurrent = getCenterOfRect(boxes);

    //// EuclidianDistance
    //return sqrt(boxes.);
//}

bool isSomethingInArea(Mat frame) {
    if (countNonZero(frame) >= ((frame.rows * frame.cols) * 1/4))
        return true;
    return false;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat& frame, Mat& fgMask, const vector<Mat>& outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
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
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);


    if (objectsRect.size() != 0) {
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            Rect newObjectRect = boxes[idx];

            int indexOfNearestPoint = 0;
            int lenghtBettwenTwoPoints = 0;

            int nearestRectIndex = -1;
            int nearestRectDistance = INT16_MAX;

            Point centerOfNewObjectRect = (newObjectRect.br() + newObjectRect.tl()) * 0.5;

            for (size_t j = 0; j < objectsRect.size(); j++)
            {
                Point centerOfObjectRect = (objectsRect[j].br() + objectsRect[j].tl()) * 0.5;

                if (newObjectRect.contains(centerOfObjectRect)) {
                    int distance = max(abs(centerOfNewObjectRect.x - centerOfObjectRect.x), abs(centerOfNewObjectRect.y - centerOfObjectRect.y));
                    if (distance < nearestRectDistance) {
                        nearestRectDistance = distance;
                        nearestRectIndex = j;
                    }
                }
            }

            if (nearestRectIndex != -1) {
                objectsRect[nearestRectIndex] = newObjectRect;
                objectsConfidences[nearestRectIndex] = confidences[idx];
                objectsRoute[nearestRectIndex].push_back((newObjectRect.br() + newObjectRect.tl()) * 0.5);
            }
            else {
                objectsIndex.push_back(objectsIndex.size() + 1);
                objectsName.push_back(classes[classIds[idx]]);
                objectsRect.push_back(newObjectRect);
                objectsConfidences.push_back(confidences[idx]);
                vector<Point> routePoint;
                routePoint.push_back((newObjectRect.br() + newObjectRect.tl()) * 0.5);
                objectsRoute.push_back(routePoint);
            }
        }
    }
    else {
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            Rect box = boxes[idx];

            // classIds name of class
            // confidences is percentage
            objectsIndex.push_back(i);
            objectsName.push_back(classes[classIds[idx]]);
            objectsRect.push_back(box);
            objectsConfidences.push_back(confidences[idx]);
            vector<Point> routePoint;
            routePoint.push_back((box.br() + box.tl()) * 0.5);
            objectsRoute.push_back(routePoint);
            //objectsRoute[0].push_back((box.br() + box.tl()) * 0.5);
        }
    }

    if (objectsRect.size() > 0) {
        for (size_t i = 0; i < objectsRect.size(); i++)
        {
            cout << "" << objectsRect[i].x << " " << objectsRect[i].y << " " << objectsRect[i].width << " " << objectsRect[i].height << endl;
            if (fgMask.rows > 0) {
                if (isSomethingInArea(fgMask(objectsRect[i]))) {
                    // drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
                    drawPred(objectsIndex[i], objectsName[i], objectsConfidences[i], objectsRect[i].x, objectsRect[i].y,
                        objectsRect[i].x + objectsRect[i].width, objectsRect[i].y + objectsRect[i].height, frame);
                    // objectTracker v1.0
                    cv::drawMarker(frame, getCenterOfRect(objectsRect[i]), cv::Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
                    for (size_t j = 0; j < objectsRoute[i].size(); ++j)
                    {
                        //circle(frame, objectsRoute[i][j], 0, Scalar(0, 255, 100), 1, 1);
                        cv::drawMarker(frame, objectsRoute[i][j], cv::Scalar(0, 255, 0), MARKER_CROSS, 5, 2);
                    }
                }
            }
        }
    }

    //objectsIndex.clear();
    //objectsName.clear();
    //objectsRect.clear();
    //objectsConfidences.clear();
}

// Draw the predicted bounding box
void drawPred(int index, String objectName, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(165, 85, 0), 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        label = objectName + ":" + label;
        if (objectName == "trainTrafficLightRed" || objectName == "trafficLightRed") {
            isCrossRoadClosed = true;
            //if (isAllowedToStartTimer == false) {
            //    //start = std::clock();
            //    //isAllowedToStartTimer = true;
            //}
        }
        else if(objectName == "trainTrafficLightWhite" || objectName == "trafficLightGreen") {
            isCrossRoadClosed = false;
        }
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(2 * labelSize.height)), Point(left + round(1.6 * labelSize.width), top + baseLine), Scalar(165, 85, 0), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_DUPLEX, 0.75, Scalar(0, 0, 0), 2);

    putText(frame, to_string(index), Point(left, bottom), FONT_HERSHEY_DUPLEX, 0.75, Scalar(255, 255, 0), 2);
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
