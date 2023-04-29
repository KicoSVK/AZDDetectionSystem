#pragma once
#ifndef	TRAFFICAREA_H_
#define TRAFFICAREA_H_

#include <iostream>
#include <vector> 

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class TrafficArea {

public:
	/** Enum type store state of current traffic area direction
	*/
	enum TrafficAreaDirection { none, leftTop, rightTop, leftBottom, rightBottom };

	/**
	*  Default constructor
	*/
	TrafficArea();

	/**
	* Add new point to traffic area
	*
	* @param cv::Point point in traffic area
	*/
	void addPointToTrafficArea(cv::Point point);

	/**
	* Get reference on traffic area
	*
	* @return std::vector<cv::Point> traffic area
	*/
	std::vector<cv::Point> getTrafficArea();

	/**
	* Delete traffic area
	*/
	void removeTrafficArea();

	/**
	* Remove point from traffic area
	*
	* @param int index from traffic area
	*/
	void removePointInTrafficArea(int index);

	/**
	* Update point in traffic area
	*
	* @param int index of point in traffic area
	* @param cv::Point point in traffic area
	*/
	void updatePointInTrafficArea(int index, cv::Point point);

	// 
	/**
	* Get size of traffic area
	*
	* @return int size of traffic area
	*/
	int getSizeOfTrafficArea();

	/**
	* Get current state of traffic area
	*
	* @return enum TrafficAreaDirection state in traffic area
	*/
	enum TrafficAreaDirection getTrafficAreaDirection();

	/**
	* Check if object in traffic area is moving allowed
	*
	* @return bool state, true if object in traffic area is moving allowed
	*/
	bool isObjectMovingInTrafficAreaAllowed();

	/**
	* Get index of nearest point in traffic area
	*
	* @param cv::Point point is testing point
	* @return int index of nearest point in traffic area
	*/
	int getIndexOfNearestPointInTrafficArea(cv::Point point);

	/**
	* Get point of nearest point in traffic area
	*
	* @param cv::Point point is testing point
	* @return int point of nearest point in traffic area
	*/
	cv::Point getPointOfNearestPointInTrafficArea(cv::Point point);

	/**
	* Check if is clicked in direction rectangle
	*
	* @param cv::Point clickPoint clicked position
	* @return bool if is clicked in direction rectangle
	*/
	bool isClickedDirectionRect(cv::Point clickPoint);

	/**
	* Set traffic area direction
	*
	* @param cv::Point clickPoint clicked position
	* @return cv::Rect traffic area direction rectangle
	*/
	cv::Rect setTrafficAreaDirection(cv::Point clickPoint, float alpha_slider);

	/**
	* Set traffic area direction index
	*
	* @param int index clickPoint clicked position
	*/
	void setTrafficAreaDirection(int index);

	/**
	* Get state if is selected traffic area direction
	*
	* @return bool, true if is selected traffic area direction
	*/
	bool getIsSelectedTrafficAreaDirection();

	/**
	* Get traffic area center 
	*
	* @return Point center of traffic area
	*/
	cv::Point getCenterOfTrafficArea();

	/**
	* Check state if direction of movement is allowed
	*
	* @return bool, true if direction of movement is allowed
	*/
	bool isDirectionOfMovementAllowed(enum TrafficArea::TrafficAreaDirection movement);

	/**
	* Set traffic area index
	*
	* @param int traffic area index
	*/
	void setTrafficAreaIndex(int id);

	/**
	* Get traffic area index
	*
	* @return int traffic area index
	*/
	int getTrafficAreaIndex();

	cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);

	cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);

private:
	std::vector<cv::Point> trafficArea; /*!< vector for store all traffic area points */
	TrafficAreaDirection trafficAreaDirection; /*!< Enum traffic area direction */
	bool isSelectedTrafficAreaDirection; /*!< state if is selected traffic area direction */
	int trafficAreaIndex; /*!< traffic area index */
};

#endif