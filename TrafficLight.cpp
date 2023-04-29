#include "TrafficLight.h"

TrafficLight::TrafficLight(){
	trafficLightColorString = "green";
}

TrafficLight::TrafficLight(cv::Rect rect, TrafficLight::TrafficLightColor color)
{
	trafficLightPosition = rect;
	trafficLightColor = color;
}

void TrafficLight::setTrafficLightPosition(cv::Rect rect)
{
	trafficLightPosition = rect;
}

cv::Rect TrafficLight::getTrafficLightPosition()
{
	return trafficLightPosition;
}

void TrafficLight::setTrafficsLightColor(TrafficLight::TrafficLightColor color)
{
	trafficLightColor = color;
}

TrafficLight::TrafficLightColor TrafficLight::getTrafficLightColor()
{
	return trafficLightColor;
}

//Typy signálù :
//{ "green", "volno" },
//{ "yellow", "žlutá" },
//{ "red", "stùj" },
//{ "yellowRed", "žlutoèervená" },
//{ "blinkingYellow", "blikavá žlutá" },
//{ "white", "aktivní výstraha" },
//{ "none", "není aktivní výstraha" },
//{ "unknown", "stav neznámý" }

std::string TrafficLight::getTrafficLightColorString()
{
	//std::map<std::string, std::string> trafficLightDnnToServerName;
	return trafficLightColorString;
}

void TrafficLight::setTrafficLightColorString(std::string color) {
	trafficLightColorString = color;
}
