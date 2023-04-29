#ifndef LPREADER_H_
#define LPREADER_H_
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <tuple>
#include <random>
#include <chrono>
#include <fstream>
#if __has_include("cameraGrabModule.h")



#include <cmath>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <cstdlib>
/*! LPReaded class contains a license plate reader */
class LPReader {

private:
	//OCR engine
	tesseract::TessBaseAPI* ocr;
	
	/**
	 * Function return an enhanced image for LP better readability
	 *
	 * @param cv::Mat origImage for enhance
	 * @return cv::Mat of enhanced image
	 */
	cv::Mat enhanceImage(cv::Mat origImage);

	/**
	 * Function return image with filtration based on HSV model for LP better readability
	 *
	 * @param cv::Mat colorImage is frame for enhance
	 * @return cv::Mat of enhanced image
	 */
	cv::Mat HSVFilter(cv::Mat colorImage);

	/**
	 * Function for remove unpermitted characters in LP text
	 *
	 * @param std::string stringToClean is string which will be cleaned
	 * @return string which is cleaned
	 */
	std::string removeUnpermittedChars(std::string stringToClean);

	/**
	 * Function for find rectangle in given image
	 *
	 * @param cv::Mat colored is matrix with LP
	 * @return std::vector<cv::Point> is rectangle with LP
	 */
	std::vector<cv::Point> getLPRectangle(cv::Mat colored);

	/**
	 * Function to pass matrix to OCR reader
	 *
	 * @param cv::Mat frameWithLP is image with LP to read by OCR
	 * @param tesseract::PageSegMode segMode is segmentation mode for use in OCR reading by Tesseract
	 * @return std::tuple<std::string, int> with LP readed text and confindece
	 */

	std::tuple<std::string, int> ORCRead(cv::Mat frameWithLP, tesseract::PageSegMode segMode);
	 /**
	  * Function to pass matrix to OCR reader
	  *
	  * @param cv::Mat frameWithLP is image with LP to read by OCR
	  * @param tesseract::PageSegMode segMode is segmentation mode for use in OCR reading by Tesseract
	  * @return std::tuple<std::string, int> with LP readed text and confindece
	  */

	/**
	* Function for sort() function to sort rectangle points (1st iteration)
	*
	* @param const std::pair<float, float>& point1 is first point of rectangle
	* @param const std::pair<float, float>& point2 is second point of rectangle
	* @return bool true if first point is lower than second
	*/
	static bool sortBBox1(const std::pair<float, float>& point1, const std::pair<float, float>& point2);

	/**
	* Function for sort() function to sort rectangle points (2st iteration)
	*
	* @param const std::pair<float, float>& point1 is first point of rectangle
	* @param const std::pair<float, float>& point2 is second point of rectangle
	* @return bool true if first point is lower than second
	*/
	static bool sortBBox2(const std::pair<float, float>& point1, const std::pair<float, float>& point2);

	mutable std::mutex _mutexForOcrEngine; //!< Lock when OCR engine is running


public:
	/**
	 * Default constructor for LPReader (License Plate reader) class
	 */
	LPReader();

	/**
	 * Destructor for LPReader (License Plate reader) class
	 */
	~LPReader();

	/**
	 * Function return tuple with readed text and confidence of readed License Plate in matrix
	 *
	 * @param cv::Mat frameWithLP is matrix containing LP for read
	 * @return tuple with readed text and confidence of LP
	 */
	std::tuple<std::string, int> readLP(cv::Mat frameWithLP);		
};
#else
/*! AppWindow class contains a app window */
class LPReader {

private:

public:
	LPReader();
	std::tuple<std::string, int> readLP(cv::Mat frameWithLP);
};

#endif
#endif