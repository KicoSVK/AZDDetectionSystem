#include "MonitoredArea.h"

	MonitoredArea::MonitoredArea() {
		monitoredAreaState = MonitoredArea::MonitoredAreaState::ready;
	}

	void MonitoredArea::addPointToMonitoredArea(cv::Point point) {
		monitoredArea.push_back(point);		
	}

	std::vector<cv::Point> MonitoredArea::getMonitoredArea() {
		return monitoredArea;
	}

	void MonitoredArea::removeMonitoredArea() {
		monitoredArea.clear();
		monitoredAreaDirection.clear();
		monitoredAreaTrafficAreaDirectionIndexes.clear();
	}

	void MonitoredArea::removePointInMonitoredArea(int index) {
		monitoredArea.erase(monitoredArea.begin() + index);
	}

	void MonitoredArea::updatePointInMonitoredArea(int index, cv::Point point) {
		monitoredArea.at(index) = point;
	}

	int MonitoredArea::getSizeOfMonitoredArea() {
		return monitoredArea.size();
	}

	enum MonitoredArea::MonitoredAreaState MonitoredArea::getStateInMonitoredArea() {
		return monitoredAreaState;
	}

	bool MonitoredArea::isMonitoredAreaRestricted() {
		switch (monitoredAreaState)
		{
		case MonitoredArea::MonitoredAreaState::ready:
			return false;
			break;
		case MonitoredArea::MonitoredAreaState::standBy:
			return false;
			break;
		case MonitoredArea::MonitoredAreaState::restricted:
			return true;
			break;
		default:
			return false;
			break;
		}
	}

	void MonitoredArea::setRestrictedStateInMonitoredArea() {
		monitoredAreaState = MonitoredArea::MonitoredAreaState::restricted;
		lastRestrictedTimeStamp = std::chrono::system_clock::now();		
	}

	void MonitoredArea::setStandByStateInMonitoredArea() {
		monitoredAreaState = MonitoredArea::MonitoredAreaState::standBy;
	}

	void MonitoredArea::setReadyStateInMonitoredArea() {
		monitoredAreaState = MonitoredArea::MonitoredAreaState::ready;
	}

	int MonitoredArea::getIndexOfNearestPointInMonitoredArea(cv::Point point) {
		for (size_t i = 0; i < monitoredArea.size(); i++)
		{
			if (monitoredArea[i].x - 25 < point.x && point.x < monitoredArea[i].x + 25 && monitoredArea[i].y - 25 < point.y && point.y < monitoredArea[i].y + 25) {
				return i;
			}
		}
		return -1;
	}

	bool MonitoredArea::isClickedDirectionRect(cv::Point clickPoint)
	{
		if (monitoredArea.size() >= 4) {
			// get moments
			cv::Moments mu = moments(monitoredArea, false);
			// get center of moments
			cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

			cv::Rect boundingRect = cv::boundingRect(monitoredArea);

			cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.1), cMu.y - (boundingRect.width * 0.1)), cv::Point(cMu.x + (boundingRect.width * 0.1), cMu.y + (boundingRect.width * 0.1)));

			if (tmpRect.contains(clickPoint)) {
				return true;
			}
			else {
				return false;
			}

		}
		else {
			return false;
		}
	}

	cv::Point2f MonitoredArea::rotate2d(const cv::Point2f& inPoint, const double& angRad)
	{
		cv::Point2f outPoint;
		//CW rotation
		outPoint.x = std::cos(angRad) * inPoint.x - std::sin(angRad) * inPoint.y;
		outPoint.y = std::sin(angRad) * inPoint.x + std::cos(angRad) * inPoint.y;
		return outPoint;
	}

	cv::Point2f MonitoredArea::rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad)
	{
		return rotate2d(inPoint - center, angRad) + center;
	}

	void MonitoredArea::addTrafficAreaDirection(cv::Point clickPoint, float alpha_slider)
	{
		#define M_PI       3.14159265358979323846   // pi

		// get moments
		cv::Moments mu = moments(monitoredArea, false);
		// get center of moments
		cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

		cv::Rect boundingRect = cv::boundingRect(monitoredArea);

		cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.1), cMu.y - (boundingRect.width * 0.1)), cv::Point(cMu.x + (boundingRect.width * 0.1), cMu.y + (boundingRect.width * 0.1)));

		// LT
		cv::Rect lt(cv::Point(tmpRect.x, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
		// RT
		cv::Rect rt(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
		// LB
		cv::Rect lb(cv::Point(tmpRect.x, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
		// RB
		cv::Rect rb(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));

		std::vector<cv::Point> ltRect = { cv::Point(lt.x, lt.y), cv::Point(lt.x + lt.width, lt.y), cv::Point(lt.x + lt.width, lt.y + lt.height), cv::Point(lt.x, lt.y + lt.height) };
		std::vector<cv::Point> rtRect = { cv::Point(rt.x, rt.y), cv::Point(rt.x + rt.width, rt.y), cv::Point(rt.x + rt.width, rt.y + rt.height), cv::Point(rt.x, rt.y + rt.height) };
		std::vector<cv::Point> rbRect = { cv::Point(rb.x, rb.y), cv::Point(rb.x + rb.width, rb.y), cv::Point(rb.x + rb.width, rb.y + rb.height), cv::Point(rb.x, rb.y + rb.height) };
		std::vector<cv::Point> lbRect = { cv::Point(lb.x, lb.y), cv::Point(lb.x + lb.width, lb.y), cv::Point(lb.x + lb.width, lb.y + lb.height), cv::Point(lb.x, lb.y + lb.height) };
				
		// LT
		ltRect[0] = rotatePoint(ltRect[0], cMu, (alpha_slider) * (M_PI / 180));
		ltRect[1] = rotatePoint(ltRect[1], cMu, (alpha_slider) * (M_PI / 180));
		ltRect[2] = rotatePoint(ltRect[2], cMu, (alpha_slider) * (M_PI / 180));
		ltRect[3] = rotatePoint(ltRect[3], cMu, (alpha_slider) * (M_PI / 180));

		// RT
		rtRect[0] = rotatePoint(rtRect[0], cMu, (alpha_slider) * (M_PI / 180));
		rtRect[1] = rotatePoint(rtRect[1], cMu, (alpha_slider) * (M_PI / 180));
		rtRect[2] = rotatePoint(rtRect[2], cMu, (alpha_slider) * (M_PI / 180));
		rtRect[3] = rotatePoint(rtRect[3], cMu, (alpha_slider) * (M_PI / 180));

		// RB
		rbRect[0] = rotatePoint(rbRect[0], cMu, (alpha_slider) * (M_PI / 180));
		rbRect[1] = rotatePoint(rbRect[1], cMu, (alpha_slider) * (M_PI / 180));
		rbRect[2] = rotatePoint(rbRect[2], cMu, (alpha_slider) * (M_PI / 180));
		rbRect[3] = rotatePoint(rbRect[3], cMu, (alpha_slider) * (M_PI / 180));

		// LB
		lbRect[0] = rotatePoint(lbRect[0], cMu, (alpha_slider) * (M_PI / 180));
		lbRect[1] = rotatePoint(lbRect[1], cMu, (alpha_slider) * (M_PI / 180));
		lbRect[2] = rotatePoint(lbRect[2], cMu, (alpha_slider) * (M_PI / 180));
		lbRect[3] = rotatePoint(lbRect[3], cMu, (alpha_slider) * (M_PI / 180));

		if (cv::pointPolygonTest(ltRect, clickPoint, false) > 0) {
			bool isDirectionExisted = false;
			// check if contains
			if (monitoredAreaDirection.size() > 0) {
				for (size_t i = 0; i < monitoredAreaDirection.size(); i++)
				{
					if (monitoredAreaDirection.at(i) == MonitoredAreaDirection::leftTop) {
						monitoredAreaDirection.erase(monitoredAreaDirection.begin() + i);
						isDirectionExisted = true;
					}
				}
			}
			// if not contains direction, so push back in vector
			if(isDirectionExisted == false)
				monitoredAreaDirection.push_back(MonitoredAreaDirection::leftTop);
		}
		else if (cv::pointPolygonTest(rtRect, clickPoint, false) > 0) {
			bool isDirectionExisted = false;
			// check if contains
			if (monitoredAreaDirection.size() > 0) {
				for (size_t i = 0; i < monitoredAreaDirection.size(); i++)
				{
					if (monitoredAreaDirection.at(i) == MonitoredAreaDirection::rightTop) {
						monitoredAreaDirection.erase(monitoredAreaDirection.begin() + i);
						isDirectionExisted = true;
					}
				}
			}
			// if not contains direction, so push back in vector
			if (isDirectionExisted == false)
				monitoredAreaDirection.push_back(MonitoredAreaDirection::rightTop);
		}
		else if (cv::pointPolygonTest(lbRect, clickPoint, false) > 0) {
			bool isDirectionExisted = false;
			// check if contains
			if (monitoredAreaDirection.size() > 0) {
				for (size_t i = 0; i < monitoredAreaDirection.size(); i++)
				{
					if (monitoredAreaDirection.at(i) == MonitoredAreaDirection::leftBottom) {
						monitoredAreaDirection.erase(monitoredAreaDirection.begin() + i);
						isDirectionExisted = true;
					}
				}
			}
			// if not contains direction, so push back in vector
			if (isDirectionExisted == false)
				monitoredAreaDirection.push_back(MonitoredAreaDirection::leftBottom);
		}
		else if (cv::pointPolygonTest(rbRect, clickPoint, false) > 0) {
			bool isDirectionExisted = false;
			// check if contains
			if (monitoredAreaDirection.size() > 0) {
				for (size_t i = 0; i < monitoredAreaDirection.size(); i++)
				{
					if (monitoredAreaDirection.at(i) == MonitoredAreaDirection::rightBottom) {
						monitoredAreaDirection.erase(monitoredAreaDirection.begin() + i);
						isDirectionExisted = true;
					}
				}
			}
			// if not contains direction, so push back in vector
			if (isDirectionExisted == false)
				monitoredAreaDirection.push_back(MonitoredAreaDirection::rightBottom);
		}
	}

	std::vector<enum MonitoredArea::MonitoredAreaDirection> MonitoredArea::getMonitoredAreaDirection()
	{
		return monitoredAreaDirection;
	}

	std::vector<int> MonitoredArea::getMonitoredAreaTrafficAreaDirectionIndexes()
	{
		return monitoredAreaTrafficAreaDirectionIndexes;
	}

	void MonitoredArea::setMonitoredAreaTrafficAreaDirectionIndexes(int index)
	{
		if (!(std::find(monitoredAreaTrafficAreaDirectionIndexes.begin(), monitoredAreaTrafficAreaDirectionIndexes.end(), index) != monitoredAreaTrafficAreaDirectionIndexes.end())) {
			// if index not found
			monitoredAreaTrafficAreaDirectionIndexes.push_back(index);
		}
	}

	// none = 0, leftTop = 1, rightTop = 2, leftBottom = 3, rightBottom = 4
	void MonitoredArea::setMonitoredAreaDirection(int index)
	{
		if (index == 0) {
			monitoredAreaDirection.push_back(MonitoredAreaDirection::none);
		}
		else if (index == 1) {
			monitoredAreaDirection.push_back(MonitoredAreaDirection::leftTop);
		}
		else if (index == 2) {
			monitoredAreaDirection.push_back(MonitoredAreaDirection::rightTop);
		}
		else if (index == 3) {
			monitoredAreaDirection.push_back(MonitoredAreaDirection::leftBottom);
		}
		else if (index == 4) {
			monitoredAreaDirection.push_back(MonitoredAreaDirection::rightBottom);
		}
	}

int MonitoredArea::getDeleyAfterRestricted()
{	
	int delayMs = std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::system_clock::now() - lastRestrictedTimeStamp)).count();
	//std::cout << "delay: " << std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::system_clock::now() - lastRestrictedTimeStamp)).count() << std::endl;
	return delayMs;
}


	//void MonitoredArea::saveObj(std::string path, MonitoredArea object) {
	//	//https://www.geeksforgeeks.org/readwrite-class-objects-fromto-file-c/
	//	std::ofstream fileWithObject;
	//	fileWithObject.open("Input.txt", std::ios::out | std::ios::binary);
	//	fileWithObject << monitoredArea.size() << " ";
	//	fileWithObject << monitoredAreaDirection.size() << " ";
	//	fileWithObject << monitoredAreaTrafficAreaDirectionIndexes.size() << " ";
	//	fileWithObject << std::endl;
	//	fileWithObject.write((char*)&*this, sizeof(*this));
	//}

	//void MonitoredArea::loadObj(std::string path, MonitoredArea object) {
	//	//https://www.geeksforgeeks.org/readwrite-class-objects-fromto-file-c/
	//	std::ifstream fileWithObject;
	//	std::ofstream tempFile;
	//	tempFile.open("temp.txt", std::ios::out | std::ios::binary);
	//	fileWithObject.open("Input.txt", std::ios::binary);
	//	std::ifstream file("Input.txt");
	//	std::string str;
	//	std::string subString;
	//	std::string stringToFind = " ";
	//	size_t pos = 0;
	//	int laneNum = 0;

	//	
	//	monitoredArea.clear();
	//	monitoredAreaDirection.clear();
	//	monitoredAreaTrafficAreaDirectionIndexes.clear();
	//	while (std::getline(file, str))
	//	{
	//		if (laneNum == 0)
	//		{
	//			while ((pos = str.find(stringToFind)) != std::string::npos) {
	//				subString = str.substr(0, pos);
	//				std::cout << subString << std::endl;
	//				if (laneNum == 0)
	//				{
	//					for (int count = 0; count < std::stoi(subString.c_str());count ++)
	//						monitoredArea.push_back(cv::Point());
	//				}
	//				else if (laneNum == 1)
	//				{
	//					for (int count = 0; count < std::stoi(subString.c_str()); count++)
	//						monitoredAreaDirection.push_back(none);
	//				} else if (laneNum == 2)
	//				{
	//					for (int count = 0; count < std::stoi(subString.c_str()); count++)
	//						monitoredAreaTrafficAreaDirectionIndexes.push_back(0);
	//				}

	//				str.erase(0, pos + stringToFind.length());
	//				laneNum++;
	//			}

	//			
	//			continue;
	//		}
	//		tempFile << str << std::endl;
	//		
	//	}
	//	tempFile.close();
	//	fileWithObject.close();
	//	fileWithObject.open("temp.txt", std::ios::binary);
	//	fileWithObject.read((char*)&*this, sizeof(*this));
	//	fileWithObject.close();
	//	//remove("temp.txt");
	//}