#include "CameraCalibration.h"

// https://github.com/spmallick/learnopencv
// https://markhedleyjones.com/projects/calibration-checkerboard-collection
// https://learnopencv.com/camera-calibration-using-opencv/

CameraCalibration::CameraCalibration() {
    cameraMatrix = cv::Mat(3, 3, CV_32F);
    distCoeffs = cv::Mat(1, 5, CV_32F);
    cameraCalibrationState = false;
    calibrate = false;
}

bool CameraCalibration::isCameraCalibrated() {
    return cameraCalibrationState;
}

void CameraCalibration::runCalibration() {

    std::cout << "Camera is calibrating ..." << std::endl;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "./images/*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;

        // Looping over all the images in the directory
        for (int i = 0; i < images.size(); i++)
        {
            frame = cv::imread(images[i]);
            //cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));

            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // Finding checker board corners
            // If desired number of corners are found in the image then success = true  
            success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

            /*
                * If desired number of corner are detected,
                * we refine the pixel coordinates and display
                * them on the images of checker board
            */
            if (success)
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

                // refining pixel coordinates for given 2d points.
                cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                // Displaying the detected corner points on the checker board
                cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
                std::cout << images[i] << ": corners was detected successfully" << std::endl;
            }

            //cv::imshow("Image", frame);
            //cv::waitKey(0);
        }

        cv::destroyAllWindows();

        /*
            * Performing camera calibration by
            * passing the value of known 3D points (objpoints)
            * and corresponding pixel coordinates of the
            * detected corners (imgpoints)
        */
        cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

        std::cout << "cameraMatrix : " << std::endl << cameraMatrix << std::endl;
        std::cout << "distCoeffs : " << std::endl << distCoeffs << std::endl;
        std::cout << "Rotation vector : " << std::endl << R << std::endl;
        std::cout << "Translation vector : " << std::endl << T << std::endl;
}

void CameraCalibration::showCalibrationParameters() {
    std::cout << "cameraMatrix : " << std::endl << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << std::endl << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << std::endl << R << std::endl;
    std::cout << "Translation vector : " << std::endl << T << std::endl;
}

void CameraCalibration::setCameraMatrix(cv::Mat cameraMatrix) {
    this->cameraMatrix = cameraMatrix;
}

void CameraCalibration::setDistCoeffs(cv::Mat distCoeffs) {
    this->distCoeffs = distCoeffs;
}

cv::Mat CameraCalibration::getCameraMatrix()
{
    return cameraMatrix;
}

cv::Mat CameraCalibration::getDistCoeffs()
{
    return distCoeffs;
}

cv::Mat CameraCalibration::undistortFrame(cv::Mat frame) {
    // Trying to undistort the image using the camera parameters obtained from calibration
    cv::Mat dst, map1, map2, new_camera_matrix;
    cv::Size imageSize(cv::Size(frame.cols, frame.rows));

    // Refining the camera matrix using parameters obtained by calibration
    new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);

    // Method 1 to undistort the image
    cv::undistort(frame, dst, new_camera_matrix, distCoeffs, new_camera_matrix);

    return dst;
}
