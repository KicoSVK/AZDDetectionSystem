#pragma once
#ifndef	CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

// https://github.com/spmallick/learnopencv
// https://markhedleyjones.com/projects/calibration-checkerboard-collection
// https://learnopencv.com/camera-calibration-using-opencv/

/*! CameraCalibration class contains tools for camera calibration */
class CameraCalibration {

public:

    /**
     * Default constructor for CameraCalibration class
    */
    CameraCalibration();

    /**
     * Function return true or false
     * if is camera frame calibrated
     *
     * @return true or false, if is camera frame calibrated
    */
    bool isCameraCalibrated();

    /**
     * Function for run a camera frame calibration
    */
    void runCalibration();

    /**
     * Function for show a camera frame calibration parameters
    */
    void showCalibrationParameters();

    /**
     * Function for set a camera calibration matrix
     *
     * @param cv::Mat cameraMatrix is camera matrix
    */
    void setCameraMatrix(cv::Mat cameraMatrix);

    /**
     * Function for show a camera frame calibration parameters
     *
     * @param cv::Mat distCoeffs is matrix, which represents camera distortion coeficients
    */
    void setDistCoeffs(cv::Mat distCoeffs);

    /**
     * Function for show a camera frame calibration parameters
     *
     * @param cv::Mat frame is matrixdistortion coeficients
    */
    cv::Mat undistortFrame(cv::Mat frame);
    cv::Mat getCameraMatrix();
    cv::Mat getDistCoeffs();

private:

    // Defining the dimensions of checkerboard
    int CHECKERBOARD[2]{ 7,9 };

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    bool cameraCalibrationState;

    bool calibrate;
    cv::Mat cameraMatrix, distCoeffs, R, T;
};

#endif