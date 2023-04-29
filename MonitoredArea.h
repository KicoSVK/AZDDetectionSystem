#ifndef	MONITOREDAREA_H_
#define MONITOREDAREA_H_

#include <iostream>
#include <vector> 

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <fstream>

/*! MonitoredArea class contains all about monitored area */
class MonitoredArea {

public:
	/**
	* Default constructor for MonitoredArea class
	*/
	MonitoredArea();

	/** Enum type store state of current traffic light color
	*/
	enum MonitoredAreaState { ready, standBy, restricted };

	/**
	* Add new point to monitored area
	*
	* @param cv::Point point in monitored area
	*/
	void addPointToMonitoredArea(cv::Point point);

	/**
	* Get reference on monitored area
	*
	* @return std::vector<cv::Point> monitored area
	*/
	std::vector<cv::Point> getMonitoredArea();

	/**
	* Delete monitored area
	*/
	void removeMonitoredArea();

	/**
	* Remove point from monitored area
	*
	* @param int index from monitored area
	*/
	void removePointInMonitoredArea(int index);

	/**
	* Update point in monitored area
	*
	* @param int index of point in monitored area
	* @param cv::Point point in monitored area
	*/
	void updatePointInMonitoredArea(int index, cv::Point point);

	/**
	* Get size of monitored area
	*
	* @return int size of monitored area
	*/
	int getSizeOfMonitoredArea();

	/**
	* Get current state of monitored area
	*
	* @return enum MonitoredAreaState state in monitored area
	*/
	enum MonitoredAreaState getStateInMonitoredArea();

	/**
	* Check if monitored area is restricted
	*
	* @return bool state, true if monitored area is restricted
	*/
	bool isMonitoredAreaRestricted();

	/**
	* Set restricted state in monitored area
	*/
	void setRestrictedStateInMonitoredArea();

	/**
	* Set stand by state in monitored area
	*/
	void setStandByStateInMonitoredArea();

	/**
	* Set ready state in monitored area
	*/
	void setReadyStateInMonitoredArea();

	/**
	* Get index of nearest point in monitored area
	*
	* @param cv::Point point for testing
	* @return int index of nearest point in monitored area
	*/
	int getIndexOfNearestPointInMonitoredArea(cv::Point point);

	/**
	* Check if is clicked in direction rectangle
	*
	* @param cv::Point clickPoint clicked position
	* @return bool if is clicked in direction rectangle
	*/
	bool isClickedDirectionRect(cv::Point clickPoint);

	/**
	* Add traffic area direction
	*
	* @param cv::Point clickPoint clicked position
	*/
	void addTrafficAreaDirection(cv::Point clickPoint, float alpha_slider);

	/** Subset of supported monitored area directions
	*/
	enum MonitoredAreaDirection { none, leftTop, rightTop, leftBottom, rightBottom };
	
	/**
	* Get current all states of monitored area
	*
	* @return std::vector<enum MonitoredAreaDirection> monitored area direction
	*/
	std::vector<enum MonitoredAreaDirection> getMonitoredAreaDirection();

	/**
	* Get current all monitored area and traffic area direction indexes
	*
	* @return std::vector<int> monitored area and traffic area direction index
	*/
	std::vector<int> getMonitoredAreaTrafficAreaDirectionIndexes();

	/**
	* Set monitored and traffic area direction index
	*/
	void setMonitoredAreaTrafficAreaDirectionIndexes(int index);

	/**
	* Set monitored and direction index
	*
	* @param int index of monitored area
	*/
	void setMonitoredAreaDirection(int index);

	/**
	* Get monitored area detecting delay
	*
	* @param int index of monitored area
	*/
	int getDeleyAfterRestricted();

	int allowedDelayAfterRestricted = 0; /*!< allowed delay after detecting in restricted state */	 	

	cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);

	cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);
private:
	// vector for monitored area
	std::vector<cv::Point> monitoredArea; /*!< monitored area */	
	MonitoredArea::MonitoredAreaState monitoredAreaState; /*!< monitored area state */
	std::vector<enum MonitoredAreaDirection> monitoredAreaDirection; /*!< monitored area direction */
	std::vector<int> monitoredAreaTrafficAreaDirectionIndexes; /*!< monitored area direction indexes */
	std::chrono::system_clock::time_point lastRestrictedTimeStamp; /*!< last restricted time */
};

#endif