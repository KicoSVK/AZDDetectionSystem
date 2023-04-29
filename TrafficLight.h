#ifndef TRAFFICLIGHT_H_
#define TRAFFICLIGHT_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <map>

/*! TrafficLight class store all about detected traffic light objects */
class TrafficLight {

public:
	/** Enum type store state of current traffic light color
	*/
	enum TrafficLightColor { ready, standBy, restricted };

	/**
	* Default constructor for TrafficLight class
	*/
	TrafficLight();

	/**
	* Constructor for TrafficLight class
	*
	* @param cv::Rect rect is rectangle of detected traffic light
	* @param TrafficLight::TrafficLightColor color is color of detected traffic light
	*/
	TrafficLight(cv::Rect rect, TrafficLight::TrafficLightColor color);

	/**
	* Function for set traffic light position
	*
	* @param cv::Rect rect is traffic light position
	*/
	void setTrafficLightPosition(cv::Rect rect);

	/**
	* Function for get traffic light position
	*
	* @return cv::Rect is traffic light position
	*/
	cv::Rect getTrafficLightPosition();

	/**
	* Function for set traffic light color
	*
	* @param TrafficLight::TrafficLightColor color is traffic light color
	*/
	void setTrafficsLightColor(TrafficLight::TrafficLightColor color);

	/**
	* Function for get traffic light color
	*
	* @return TrafficLightColor is traffic light color
	*/
	TrafficLightColor getTrafficLightColor();

	/**
	* Function for set traffic light color by string
	*
	* @param std::string color is traffic light color
	*/
	void setTrafficLightColorString(std::string color);

	/**
	* Function for get traffic light color string
	*
	* @return std::string is traffic light color string
	*/
	std::string getTrafficLightColorString();

private:		
	cv::Rect trafficLightPosition; /*!< traffic light position */
	TrafficLightColor trafficLightColor; /*!< traffic light color */
	std::string trafficLightColorString; /*!< traffic light color string */
};
#endif