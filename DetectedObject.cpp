#include "DetectedObject.h"

// Constructor for all detected vehicles with license plate
DetectedObject::DetectedObject(std::string objectName, float objectConfidences, cv::Rect objectRect, int objectIndex, cv::Point objectRoutePoint, cv::Mat LP){

	this->objectName = objectName;
	this->objectConfidences = objectConfidences;
	this->objectRect = objectRect;
	this->objectIndex = objectIndex;
	objectRoute.push_back(objectRoutePoint);
	this->ttl = detectedObjectTTL;

	this->objectStopped = false;
	
	this->inMonitoredAreaObjectMovingTTL = monitoredAreaObjectMovingTTL;

	this->inTrafficAreaObjectDirectionMovementTTL = trafficAreaObjectDirectionMovementTTL;
	//movementDirectionTtl.push_back(1);

	this->indexOfTrafficArea = -1;

	// call Kalman filter init function
	kalmanFilterInit((objectRect.br() + objectRect.tl()) * 0.5);

	updateKalmanFilter((objectRect.br() + objectRect.tl()) * 0.5);

	this->LP = LP;
}

// Constructor for all detected vehicles
DetectedObject::DetectedObject(std::string objectName, float objectConfidences, cv::Rect objectRect, int objectIndex, cv::Point objectRoutePoint) {
	this->objectName = objectName;
	this->objectConfidences = objectConfidences;
	this->objectRect = objectRect;
	this->objectIndex = objectIndex;
	objectRoute.push_back(objectRoutePoint);
	this->ttl = detectedObjectTTL;

	this->inMonitoredAreaObjectMovingTTL = monitoredAreaObjectMovingTTL;

	this->objectStopped = false;

	this->inTrafficAreaObjectDirectionMovementTTL = trafficAreaObjectDirectionMovementTTL;
	//movementDirectionTtl.push_back(1);

	this->indexOfTrafficArea = -1;

	// call Kalman filter init function
	kalmanFilterInit((objectRect.br() + objectRect.tl()) * 0.5);

	updateKalmanFilter((objectRect.br() + objectRect.tl()) * 0.5);
}

// Constructor for other not important objects
DetectedObject::DetectedObject(std::string objectName, float objectConfidences, cv::Rect objectRect) {
	this->objectName = objectName;
	this->objectConfidences = objectConfidences;
	this->objectRect = objectRect;
	this->objectIndex = -1;

	// call Kalman filter init function
	//kalmanFilterInit();
}

// Default destructor
DetectedObject::~DetectedObject() {
	objectRoute.clear();
	//movementDirectionTtl.clear();

}

// Kalman filter function for initialize important matrix and parameters
void DetectedObject::kalmanFilterInit(cv::Point objectCenter) {
	kf = cv::KalmanFilter(4, 2, 0);	

	kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

	kf.statePre.at<float>(0) = objectCenter.x;
	kf.statePre.at<float>(1) = objectCenter.y;
	kf.statePre.at<float>(2) = 0;
	kf.statePre.at<float>(3) = 0;

	//isFoundInitialDectedObject = false;
	//cv::Mat procNoise(stateSize, 1, type)
	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	//cv::setIdentity(kf.transitionMatrix);
	//kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	//kf.measurementMatrix = cv::Mat::zeros(measureParams, dynamParams, type);
	//kf.measurementMatrix.at<float>(0) = 1.0f;
	//kf.measurementMatrix.at<float>(7) = 1.0f;
	//kf.measurementMatrix.at<float>(16) = 1.0f;
	//kf.measurementMatrix.at<float>(23) = 1.0f;
	cv::setIdentity(kf.measurementMatrix);

	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	//kf.processNoiseCov.at<float>(0) = 1e-2;
	//kf.processNoiseCov.at<float>(7) = 1e-2;
	//kf.processNoiseCov.at<float>(14) = 5.0f;
	//kf.processNoiseCov.at<float>(21) = 5.0f;
	//kf.processNoiseCov.at<float>(28) = 1e-2;
	//kf.processNoiseCov.at<float>(35) = 1e-2;
	//setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));

	// Measures Noise Covariance Matrix R
	//cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(10));

	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(.1));
}

// Getters
std::string DetectedObject::getObjectName() {
	return objectName;
}

std::string DetectedObject::getOldObjectName() {
	return oldObjectName;
}

int DetectedObject::getIndexOfObject() {
	return objectIndex;
}

float DetectedObject::getObjectConfidence() {
	return objectConfidences;
}

cv::Rect DetectedObject::getObjectRect() {
	return objectRect;
}

int DetectedObject::getObjectRouteSize() {
	return objectRoute.size();
}

std::vector<cv::Point2f> DetectedObject::getObjectRoute() {
	return objectRoute;
}

int DetectedObject::getTtl() {
	return ttl;
}

bool DetectedObject::getIsFoundInitialDectedObject() {
	return isFoundInitialDectedObject;
}

cv::Rect DetectedObject::getPredictedRectByKF() {
	return this->objectRect;
}

cv::Rect DetectedObject::getObjectPredRect() {
	if (kfPredictedMatrix.rows > 0) {
		cv::Point predictedPoint(kfPredictedMatrix.at<float>(0), kfPredictedMatrix.at<float>(1));

		cv::Rect predictedRect(cv::Point(predictedPoint.x - (getObjectRect().width / 2), predictedPoint.y - (getObjectRect().height / 2)), cv::Point(predictedPoint.x + (getObjectRect().width / 2), predictedPoint.y + (getObjectRect().height / 2)));

		return predictedRect;
	}
	else {
		return cv::Rect(cv::Point(0, 0), cv::Point(0, 0));
	}
}

cv::Point DetectedObject::getObjectCenter()
{
	return (this->objectRect.br() + this->objectRect.tl()) * 0.5;
}

cv::Point DetectedObject::getObjectCenterOffset() {
	cv::Point tmpPoint((this->objectRect.br() + this->objectRect.tl()) * 0.5);

	tmpPoint.y = tmpPoint.y + (objectRect.height / 4);

	return tmpPoint;
}

cv::Point DetectedObject::getObjectPredRectCenter() {

	if (kfPredictedMatrix.rows > 0) {
		cv::Point predictedPoint(kfPredictedMatrix.at<float>(0), kfPredictedMatrix.at<float>(1));
		
		return predictedPoint;
	}
	else {
		return cv::Point(0,0);
	}
}

void DetectedObject::applyKalmanPrediction() {
	kfPredictedMatrix = kf.predict();
}

void DetectedObject::updateKalmanFilter(cv::Point point) {
	cv::Mat_<float> positionMatrix(2, 1);
	positionMatrix(0) = point.x;
	positionMatrix(1) = point.y;
	
	kf.correct(positionMatrix);
}

int DetectedObject::getMovementDirectionTtl()
{
	//return this->movementDirectionTtl[0];
	//std::cout << "MOVEMENT: " << this->movementDirectionTtl << std::endl;
	return this->inTrafficAreaObjectDirectionMovementTTL;
}

void DetectedObject::decreaseMovementDirectionTtl()
{
	this->inTrafficAreaObjectDirectionMovementTTL--;
	//std::cout << "MOVEMENT decrease to: " << this->inTrafficAreaObjectDirectionMovementTTL << std::endl;
}

void DetectedObject::resetMovementDirectionTtl()
{
	//this->movementDirectionTtl[0] = 1;
	this->inTrafficAreaObjectDirectionMovementTTL = trafficAreaObjectDirectionMovementTTL;
}

void DetectedObject::resetObjectMovingTTL() {
	this->inMonitoredAreaObjectMovingTTL = monitoredAreaObjectMovingTTL;
}

void DetectedObject::decreaseObjectMovingTTL() {
	this->inMonitoredAreaObjectMovingTTL--;
	//std::cout << "Decrease to: " << this->objectMovingTTL << std::endl;
}

void DetectedObject::increaseObjectMovingTTL() {
	this->inMonitoredAreaObjectMovingTTL++;
	//std::cout << "Decrease to: " << this->objectMovingTTL << std::endl;
}

int DetectedObject::getObjectMovingTTL() {
	return this->inMonitoredAreaObjectMovingTTL;
}

bool DetectedObject::isObjectMoving() {
	int averageLastObjectMovingDistance = 0;

	if (getObjectRouteSize() >= objectHistoryMovingSteps) {

		for (int i = 0; i < objectHistoryMovingSteps; i++)
		{
			float pointsDistance = cv::max(abs(getObjectRoute()[getObjectRouteSize() - (i + 1)].x - getObjectRoute()[getObjectRouteSize() - (i + 2)].x), abs(getObjectRoute()[getObjectRouteSize() - (i + 1)].y - getObjectRoute()[getObjectRouteSize() - (i + 2)].y));
			averageLastObjectMovingDistance = averageLastObjectMovingDistance + pointsDistance;
		}

		averageLastObjectMovingDistance = averageLastObjectMovingDistance / objectHistoryMovingSteps;

		//std::cout << averageLastObjectMovingDistance << std::endl;
		//std::cout << averageLastObjectMovingDistance << std::endl;		
		//std::cout << getObjectMovingTTL() << std::endl;

		if (averageLastObjectMovingDistance < minLastObjectMovingDistanceForStopping) {
			decreaseObjectMovingTTL();
			//std::cout << "Avg distance: " << averageLastObjectMovingDistance << std::endl;
			//decreaseObjectMovingTTL();
			//std::cout << "Object moving TTL: " << getObjectMovingTTL() << std::endl;
		}
		else {
			//resetObjectMovingTTL();
			increaseObjectMovingTTL();
			return true;
		}

		if (getObjectMovingTTL() <= 0) {
			this->objectStopped = true;
			return false;
		}
	}
	return true;
}

bool DetectedObject::isObjectStopped() {
	return this->objectStopped;
}

void DetectedObject::setObjectIsMovingAgain() {
	this->objectStopped = false;
}

void DetectedObject::setObjectName(std::string objectName) {
	this->objectName = objectName;
}

void DetectedObject::setOldObjectName(std::string objectName) {
	this->oldObjectName = objectName;
}

void DetectedObject::setIndexOfObject(int objectIndex) {
	this->objectIndex = objectIndex;
}

void DetectedObject::setObjectConfidence(float objectConfidences) {
	this->objectConfidences = objectConfidences;
}

void DetectedObject::setObjectRect(cv::Rect objectRect) {
	this->objectRect = objectRect;
	
	updateKalmanFilter(cv::Point(objectRect.br() + objectRect.tl()) * 0.5);
}

void DetectedObject::addObjectRoutePoint(cv::Point routePoint) {
	if (objectRoute.size() > 32) {
		objectRoute.erase(objectRoute.begin());
	}
	objectRoute.push_back(routePoint);
}

cv::KalmanFilter DetectedObject::getKalmanFilter() {
	return kf;
}

//// Matrix of state, for Kalman filter
//cv::Mat DetectedObject::getStateMatrix() {
//	return stateMatrix;
//}
//
//// Matrix of measurement, for Kalman filter
//cv::Mat DetectedObject::getMeasurementMatrix() {
//	return measurementMatrix;
//}
//
//cv::Mat DetectedObject::getKfPredictMatrix() {
//	return kfPredictMatrix;
//}
//
//void DetectedObject::setKalmanFilter(int dynamParams, int measureParams, int controlParams, int type) {
//	kf.init(dynamParams, measureParams, controlParams, type);
//}
//
//// Matrix of state, for Kalman filter
//void DetectedObject::setStateMatrix(cv::Mat stateMatrix) {
//	this->stateMatrix = stateMatrix;
//}
//
//// Matrix of measurement, for Kalman filter
//void DetectedObject::setMeasurementMatrix(cv::Mat measurementMatrix) {
//	this->measurementMatrix = measurementMatrix;
//}

void DetectedObject::setTtl(int ttl) {
	this->ttl = ttl;
}

void DetectedObject::resetTtl() {
	this->ttl = detectedObjectTTL;
}

void DetectedObject::decreaseTtl() {
	this->ttl = this->ttl - 1;
}

void DetectedObject::decreaseTtl(int value) {
	this->ttl = this->ttl - value;
}

void DetectedObject::setIsFoundInitialDectedObject(bool state) {
	this->isFoundInitialDectedObject = state;
}

void DetectedObject::setObjectPredRect(cv::Rect predRect) {
	this->predRect = predRect;
}

void DetectedObject::setObjectPredRectCenter(cv::Point predRectCenter) {
	this->predRectCenter = predRectCenter;
}
//
//void DetectedObject::setKfPredictMatrix(cv::Mat kfPredictMat) {
//	this->kfPredictMatrix = kfPredictMat;
//}
//
//void DetectedObject::setDT(float dt) {
//	this->dT = dt;
//}

cv::Mat DetectedObject::getLP() {
	return this->LP;
}

void DetectedObject::setLP(cv::Mat LP) {
	this->LP = LP;
}

cv::Point DetectedObject::rotatePoint(cv::Point pointToRotate, int angleDegrees)
{
	cv::Point rotatedPoint;
	double angleRad = (angleDegrees * M_PI) / 180.0;

	rotatedPoint.x = std::cos(angleRad) * pointToRotate.x - std::sin(angleRad) * pointToRotate.y;
	rotatedPoint.y = std::sin(angleRad) * pointToRotate.x + std::cos(angleRad) * pointToRotate.y;

	return rotatedPoint;
}

cv::Point DetectedObject::rotatePointAroundCenter(cv::Point pointToRotate, cv::Point centerPoint, int angleDegrees)
{
	return rotatePoint(pointToRotate - centerPoint, angleDegrees) + centerPoint;
}

//https://stackoverflow.com/questions/7953316/rotate-a-point-around-a-point-with-opencv
TrafficArea::TrafficAreaDirection DetectedObject::getDirectionOfMovement()
{
	//std::cout << alpha_slider << std::endl;
	std::string directiveOfObject = "";	

	if (DetectedObject::getObjectRoute().size() > lastTrajectoryDistanceForObjectMovementInTrafficArea) {
		int averageOfPointX = 0, averageOfPointY = 0;
		//int angleDegrees = 0;
		cv::Point centerOfTrajectory = DetectedObject::getObjectRoute().at(round(DetectedObject::getObjectRoute().size() / 2));

		// check if object is moving
		int distanceBetweenLastTwoStepsX = std::abs(DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - 1).x - DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - 2).x);
		int distanceBetweenLastTwoStepsY = std::abs(DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - 1).y - DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - 2).y);

		//std::cout << distanceBetweenLastTwoStepsX << " / " << distanceBetweenLastTwoStepsY << std::endl;

		if (distanceBetweenLastTwoStepsX > minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea || distanceBetweenLastTwoStepsY > minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea) {
			for (int i = 1; i < lastTrajectoryDistanceForObjectMovementInTrafficArea; i++)
			{
				averageOfPointX = averageOfPointX + DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - i).x;
				averageOfPointY = averageOfPointY + DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - i).y;
			}

			averageOfPointX = averageOfPointX / (lastTrajectoryDistanceForObjectMovementInTrafficArea - 1);
			averageOfPointY = averageOfPointY / (lastTrajectoryDistanceForObjectMovementInTrafficArea - 1);

			int p2x = DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - 1).x;
			int p2y = DetectedObject::getObjectRoute().at(DetectedObject::getObjectRoute().size() - 1).y;

			if (alpha_slider != 0)
			{
				cv::Point rotatedPoint = rotatePointAroundCenter(cv::Point(p2x, p2y), centerOfTrajectory, alpha_slider);
				p2x = rotatedPoint.x;
				p2y = rotatedPoint.y;

				rotatedPoint = rotatePointAroundCenter(cv::Point(averageOfPointX, averageOfPointY), centerOfTrajectory, alpha_slider);
				averageOfPointX = rotatedPoint.x;
				averageOfPointY = rotatedPoint.y;
			}

			int directivePointX = p2x - (averageOfPointX - p2x);
			int directivePointY = p2y - (averageOfPointY - p2y);

			// x direction
			if (directivePointX > averageOfPointX) {
				directiveOfObject = "R";				
			}
			else {
				directiveOfObject = "L";
			}

			// y direction
			if (directivePointY > averageOfPointY) {
				if (directiveOfObject == "R") {
					//std::cout << DetectedObject::getIndexOfObject() << ": " << "rightBottom" << std::endl;
					return TrafficArea::TrafficAreaDirection::rightBottom;
				}
				else if (directiveOfObject == "L") {
					//std::cout << DetectedObject::getIndexOfObject() << ": " << "leftBottom" << std::endl;

					return TrafficArea::TrafficAreaDirection::leftBottom;
				}
			}
			else {
				if (directiveOfObject == "R") {
					//std::cout << DetectedObject::getIndexOfObject() << ": " << "rightTop" << std::endl;

					return TrafficArea::TrafficAreaDirection::rightTop;
				}
				else if (directiveOfObject == "L") {
					//std::cout << DetectedObject::getIndexOfObject() << ": " << "leftTop" << std::endl;

					return TrafficArea::TrafficAreaDirection::leftTop;
				}
			}			
		}
		else {
			return TrafficArea::TrafficAreaDirection::none;
		}
	}
	// return none, because it can not calculate a directive of line
	else {
		return TrafficArea::TrafficAreaDirection::none;
	}
}

void DetectedObject::setIndexOfTrafficArea(int indexOfTrafficArea)
{
	this->indexOfTrafficArea = indexOfTrafficArea;
}

int DetectedObject::getIndexOfTrafficArea()
{
	return this->indexOfTrafficArea;
}

void DetectedObject::setDetectedTime(float time)
{
	if (currentDetectedTime <= 0) {
		previousDetectedTime = time;
	}
	else {
		previousDetectedTime = currentDetectedTime;
		currentDetectedTime = time;
	}
}

float DetectedObject::getObjectSpeed(float distanceConstant, float timeConstant, cv::Mat perspectiveMatrix)
{
	// realD = 19,4m
	// imgD = 470,2 px
	// 1px = 0,0412590387069332
	// v = s / t

	//7,83m
	//600px
	//1px = 600

	if (perspectiveMatrix.rows > 0) {
		std::vector<cv::Point2f> outputPoints;

		// average last 4 object route distance
		cv::perspectiveTransform(objectRoute, outputPoints, perspectiveMatrix);

		float distanceInPx = 0;
		float realDistance = 0;
		float speed = 0;
		int countOfLastMeasuring = 10;

		for (size_t i = 1; i <= countOfLastMeasuring; i++)
		{
			distanceInPx = sqrt(pow(outputPoints[outputPoints.size() - i].x - outputPoints[outputPoints.size() - (i + 1)].x, 2) + pow(outputPoints[outputPoints.size() - i].y - outputPoints[outputPoints.size() - (i + 1)].y, 2));

			if (distanceInPx > 2) {
				realDistance = distanceConstant * distanceInPx;
			}
			else {
				realDistance = 0;
				if (objectSpeed.size() > 0) {
					for (int i = 0; i < objectSpeed.size(); i++)
					{
						objectSpeed.at(i) = 0;
					}
				}
				return 0;
			}

			if (realDistance > 0) {
				if (objectSpeed.size() > (countOfLastMeasuring - 1)) {
					objectSpeed.erase(objectSpeed.begin());
				}
				objectSpeed.push_back((realDistance / timeConstant) * 3.6);
			}
		}

		for (int i = 0; i < objectSpeed.size(); i++)
		{
			speed = speed + objectSpeed.at(i);
		}
		speed = speed / objectSpeed.size();

		//return realDistance / (currentDetectedTime - previousDetectedTime);
		if (speed > 199 || speed < 0) {
			return 45;
		}
		else {
			return speed;
		}
	}	
	return 0;
}