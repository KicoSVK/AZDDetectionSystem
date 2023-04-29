#ifndef DETECTEDOBJECT_H_
#define DETECTEDOBJECT_H_

#include <iostream>
#include <math.h>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

#include "TrafficArea.h"

#define _USE_MATH_DEFINES

# define M_PI           3.14159265358979323846  /* pi */

extern int alpha_slider;
extern int objectHistoryMovingSteps;
extern int lastTrajectoryDistanceForObjectMovementInTrafficArea;
extern int minimalDistanceBetweenLastTwoStepsForObjectMovementInTrafficArea;
extern int minLastObjectMovingDistanceForStopping;
extern int monitoredAreaObjectMovingTTL;
extern int trafficAreaObjectDirectionMovementTTL;
extern int detectedObjectTTL;

/*! DetectedObject class contains a all detected objects */
class DetectedObject {

private:
	std::string objectName; /*!< detected object name */
	std::string oldObjectName; /*!< old detected object name */
	int objectIndex; /*!< old detected object name */
	float objectConfidences; /*!< detected object confidence */
	cv::Rect objectRect; /*!< detected object rectangle */
	std::vector<cv::Point2f> objectRoute; /*!< detected object trajectory */
	bool isFoundInitialDectedObject; /*!< state if is detected object initialized */
	int ttl; /*!< detected object TTL */

	std::vector<int> objectSpeed; /*!< detected object speed */

	bool objectStopped; /*!< state if detected object is stopped */

	int inTrafficAreaObjectDirectionMovementTTL; /*!< detected object direction movement TTL */

	int inMonitoredAreaObjectMovingTTL; /*!< detected object movement TTL in monitored area */

	int indexOfTrafficArea; /*!< traffic area index */

	cv::Rect predRect; /*!< object predicted rectangle */
	cv::Point predRectCenter; /*!< object predicted rectangle center */

	// Kalman Filter
	// https://docs.opencv.org/master/dd/d6a/classcv_1_1KalmanFilter.html
	// https://www.myzhar.com/blog/tutorials/tutorial-opencv-ball-tracker-using-kalman-filter/

	cv::KalmanFilter kf; /*!< Kalman filter object */
	cv::Mat kfPredictedMatrix; /*!< Kalman filter object */

	cv::Mat LP; /*!< license plate matrix */

	// Speed measurement parameters
	float previousDetectedTime; /*!< previous detected time for speed measurement */
	float currentDetectedTime; /*!< current detected time for speed measurement */

public:
	/**
	* Constructor for all detected vehicles with LP
	*/
	DetectedObject(std::string objectName, float objectConfidences, cv::Rect objectRect, int objectIndex, cv::Point objectRoutePoint, cv::Mat LP);

	/**
	* Constructor for all detected vehicles
	*/
	DetectedObject(std::string objectName, float objectConfidences, cv::Rect objectRect, int objectIndex, cv::Point objectRoutePoint);
	
	/**
	* Constructor for other not important objects
	*/
	DetectedObject(std::string objectName, float objectConfidences, cv::Rect objectRect);

	/**
	* Default constructor for DetectedObject class
	*/
	~DetectedObject();

	/**
	* Function for initialize Kalmans, important matrix and parameters
	*
	* @param cv::Point objectCenter is center of object
	*/
	void kalmanFilterInit(cv::Point objectCenter);

	/**
	* Function for get detected object name
	*
	* @return std::string name of object
	*/
	std::string getObjectName();

	/**
	* Function for get detected object name
	*
	* @return std::string name of object
	*/
	std::string getOldObjectName();

	/**
	* Function for get detected object name
	*
	* @return int index of object
	*/
	int getIndexOfObject();

	/**
	* Function for get detected object confidence
	*
	* @return float object confidence
	*/
	float getObjectConfidence();

	/**
	* Function for get detected object rectangle
	*
	* @return cv::Rect object rectangle
	*/
	cv::Rect getObjectRect();

	/**
	* Function for get detected object trajectory points size
	*
	* @return int object trajectory size
	*/
	int getObjectRouteSize();

	/**
	* Function for get detected object trajectory
	*
	* @return std::vector<cv::Point2f> object trajectory
	*/	
	std::vector<cv::Point2f> getObjectRoute();

	/**
	* Function for get detected object TTL
	*
	* @return int object TTL
	*/
	int getTtl();

	/**
	* Function for get state, if is found initial detected object
	*
	* @return bool state, if is found initial detected object
	*/
	bool getIsFoundInitialDectedObject();

	/**
	* Function for get predicted rectangle of detected object
	*
	* @return cv::Rect rectangle of detected object
	*/
	cv::Rect getPredictedRectByKF();

	/**
	* Function for get predicted rectangle of detected object
	*
	* @return cv::Rect rectangle of detected object
	*/
	cv::Rect getObjectPredRect();

	/**
	* Function for get center of detected object
	*
	* @return cv::Point center of detected object
	*/
	cv::Point getObjectCenter();

	/**
	* Function for get center of detected object with offset
	*
	* @return cv::Point center of detected object with offset
	*/
	cv::Point getObjectCenterOffset();

	/**
	* Function for get center of predicted detected object
	*
	* @return cv::Point center of predicted detected object
	*/
	cv::Point getObjectPredRectCenter();

	/**
	* Function for get detected object movement TTL
	*
	* @return int detected object movement TTL
	*/
	int getMovementDirectionTtl();

	/**
	* Function for increase detected object movement TTL
	*/
	void increaseObjectMovingTTL();

	/**
	* Function for decrease detected object movement TTL
	*/
	void decreaseMovementDirectionTtl();

	/**
	* Function for reset detected object movement TTL
	*/
	void resetMovementDirectionTtl();

	/**
	* Function for reset detected object moving TTL
	*/
	void resetObjectMovingTTL();

	/**
	* Function for decrease detected object moving TTL
	*/
	void decreaseObjectMovingTTL();

	/**
	* Function for apply Kalman prediction
	*/
	void applyKalmanPrediction();

	/**
	* Function for update Kalman filter
	*
	* @param cv::Point point object point
	*/
	void updateKalmanFilter(cv::Point point);

	/**
	* Function for set object name
	*
	* @param std::string object name
	*/
	void setObjectName(std::string objectName);

	/**
	* Function for set old, previous name of object
	*
	* @param std::string previous object name
	*/
	void setOldObjectName(std::string objectName);

	/**
	* Function for set index of detected object
	*
	* @param int objectIndex is index of object
	*/
	void setIndexOfObject(int objectIndex);

	/**
	* Function for set confidence of detected object
	*
	* @param float objectConfidences is confidence of detected object
	*/
	void setObjectConfidence(float objectConfidences);

	/**
	* Function for set rectangle of detected object
	*
	* @param cv::Rect objectRect is rectangle of detected object
	*/
	void setObjectRect(cv::Rect objectRect);

	/**
	* Function for add route point of detected object trajektory
	*
	* @param cv::Point routePoint is trajektory point of detected object
	*/
	void addObjectRoutePoint(cv::Point routePoint);

	/**
	* Function for get Kalman filter object reference
	*
	* @return cv::KalmanFilter is Kalman filter object reference
	*/
	cv::KalmanFilter getKalmanFilter();

	/**
	* Function for get state, if detected object is moving
	*
	* @return bool true/false, if detected object is moving
	*/
	bool isObjectMoving();

	/**
	* Function for get state, if detected object is stopped
	*
	* @return bool true/false, if detected object is stopped
	*/
	bool isObjectStopped();

	/**
	* Function for set state, that detected object is moving again
	*/
	void setObjectIsMovingAgain();

	/**
	* Function for get object moving TTL
	*
	* @return int get object moving TTL
	*/
	int getObjectMovingTTL();

	/**
	* Function for set object TTL
	*
	* @param int object TTL
	*/
	void setTtl(int ttl);

	/**
	* Function for reset object TTL
	*/
	void resetTtl();

	/**
	* Function for decrease object TTL
	*/
	void decreaseTtl();

	/**
	* Function for decrease object TTL
	*
	* @param int value TTL
	*/
	void decreaseTtl(int value);

	/**
	* Function for set state, if is found initial detected object
	*
	* @param bool state, if is found initial detected object
	*/
	void setIsFoundInitialDectedObject(bool state);

	/**
	* Function for set predicted rectangle of detected object
	*
	* @param cv::Rect predRect is predicted rectangle of detected object
	*/
	void setObjectPredRect(cv::Rect predRect);

	/**
	* Function for set center of predicted rectangle
	*
	* @param cv::Point center of predicted rectangle
	*/
	void setObjectPredRectCenter(cv::Point predRectCenter);

	/**
	* Function for get license plate
	*
	* @return cv::Mat get license plate matrix
	*/
	cv::Mat getLP();

	/**
	* Function for set license plate
	*
	* @param cv::Mat set license plate matrix
	*/
	void setLP(cv::Mat LP);

	/**
	* Function for rotate point
	*
	* @param cv::Point pointToRotate point to rotate
	* @param int angleDegrees point angle to rotate
	*
	* @return cv::Point rotated point
	*/
	cv::Point rotatePoint(cv::Point pointToRotate, int angleDegrees);

	/**
	* Function for rotate point around the centre
	*
	* @param cv::Point pointToRotate point to rotate
	* @param int angleDegrees point angle to rotate
	*
	* @return cv::Point rotated point
	*/
	cv::Point rotatePointAroundCenter(cv::Point pointToRotate, cv::Point centerPoint, int angleDegrees);

	/**
	* Function for set KF predict matrix
	*
	* @param cv::Mat kfPredictMat a KF predict matrix
	*/
	void setKfPredictMatrix(cv::Mat kfPredictMat);

	/**
	* Function for set index of traffic area
	*
	* @param int indexOfTrafficArea is index of traffic area
	*/
	void setIndexOfTrafficArea(int indexOfTrafficArea);

	/**
	* Function for get index of traffic area
	*
	* @return int index of traffic area
	*/
	int getIndexOfTrafficArea();

	/**
	* Function for set detected time
	*
	* @param float time is time
	*/
	void setDetectedTime(float time);

	/**
	* Function for set detected time
	*
	* @param float distanceConstant is distance constant
	* @param float timeConstant is time constant
	* @param float perspectiveMatrix is perspective matrix
	*
	* @return float is object speed
	*/
	float getObjectSpeed(float distanceConstant, float timeConstant, cv::Mat perspectiveMatrix);

	/**
	* Function for get traffic area direction
	*
	* @return TrafficArea::TrafficAreaDirection traffic area direction
	*/
	TrafficArea::TrafficAreaDirection getDirectionOfMovement();
};

#endif