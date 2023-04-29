#pragma once
#include <opencv2/highgui.hpp>
#include <ctime>

/*! DetectedOtherObject class contains a all other detected objects */
class DetectedOtherObject {

private:
	cv::Rect objectRect; /*!< detected object rectangle */
	cv::Point objectCenter; /*!< detected object rectangle center point */
	std::chrono::high_resolution_clock::time_point startTime; /*!< detected object start time */
	int ttl; /*!< detected object TTL */

public:
	/**
	* Constructor for DetectedOtherObject class
	*
	* @param cv::Rect objectRect detected object rectangle
	* @param cv::Point objectCenter detected object rectangle center point
	* @param std::chrono::high_resolution_clock::time_point startTime detected object start time
	*/
	DetectedOtherObject(cv::Rect objectRect, cv::Point objectCenter, std::chrono::high_resolution_clock::time_point startTime);

	/**
	 * Function return detected object rectangle
	 *
	 * @return cv::Rect detected object rectangle
	 */
	cv::Rect getObjectRect();

	/**
	 * Function return detected object rectangle center
	 *
	 * @return cv::Point detected object rectangle center
	 */
	cv::Point getObjectCenter();

	/**
	 * Function return detected object start time
	 *
	 * @return std::chrono::high_resolution_clock::time_point detected object start time
	 */
	std::chrono::high_resolution_clock::time_point getStartTime();

	/**
	* Function reset detected object TTL
	*/
	void resetTtl();

	/**
	* Function increase detected object TTL
	*/
	void increaseTtl();

	/**
	* Function decrease detected object TTL
	*/
	void decreaseTtl();

	/**
	* Function for update object rectangle
	*
	* @param cv::Rect objectRect is detected object rectangle
	*/
	void updateObjectRect(cv::Rect objectRect);

	/**
	* Function for get TTL
	*/
	int getTtl();
};