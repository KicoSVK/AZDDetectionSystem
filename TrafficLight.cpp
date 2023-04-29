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

//Typy sign�l� :
//{ "green", "volno" },
//{ "yellow", "�lut�" },
//{ "red", "st�j" },
//{ "yellowRed", "�luto�erven�" },
//{ "blinkingYellow", "blikav� �lut�" },
//{ "white", "aktivn� v�straha" },
//{ "none", "nen� aktivn� v�straha" },
//{ "unknown", "stav nezn�m�" }

std::string TrafficLight::getTrafficLightColorString()
{
	//std::map<std::string, std::string> trafficLightDnnToServerName;
	return trafficLightColorString;
}

void TrafficLight::setTrafficLightColorString(std::string color) {
	trafficLightColorString = color;
}
