#include "DetectedOtherObject.h"

DetectedOtherObject::DetectedOtherObject(cv::Rect objectRect, cv::Point objectCenter, std::chrono::high_resolution_clock::time_point startTime)
{
	this->objectCenter = objectCenter;
	this->objectRect = objectRect;
	this->startTime = startTime;
	this->ttl = 14;
}

cv::Rect DetectedOtherObject::getObjectRect()
{
	return objectRect;
}

cv::Point DetectedOtherObject::getObjectCenter()
{
	return objectCenter;
}

std::chrono::high_resolution_clock::time_point DetectedOtherObject::getStartTime()
{
	return startTime;
}

void DetectedOtherObject::resetTtl()
{
	this->ttl = 14;
}

void DetectedOtherObject::increaseTtl()
{
	if(this->ttl <= 14)
		this->ttl = this->ttl + 1;
}

void DetectedOtherObject::decreaseTtl()
{
	if (this->ttl >= -1)
		this->ttl = this->ttl - 1;
}

int DetectedOtherObject::getTtl() {
	return this->ttl;
}

void DetectedOtherObject::updateObjectRect(cv::Rect objectRect) {
	this->objectRect = objectRect;
}
