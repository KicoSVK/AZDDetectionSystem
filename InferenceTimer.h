#ifndef	INFERENCETIMER_H_
#define INFERENCETIMER_H_

#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <chrono>

/*! AppWindow class contains a app window */
class InferenceTimer {

private:
	std::clock_t startTime; //!< used for store start time, from which time count start
	std::vector<double> averageVector; //!< used for store old calculated durations for count average duration
	double frameProcessingDuration; //!< used for store duration of frame processing time
	std::chrono::time_point<std::chrono::high_resolution_clock> t_start; //!< used for store start time, from which time count start

public:
	/**
	 * Function for initialization inference timer
	 */
	void timerInitialization();

	/**
	 * Function for set start time
	 * @param std::clock_t startTime is time for set
	 */
	void setStartTime(std::clock_t startTime);

	/**
	 * Function for get start time
	 * @return std::clock_t is start time
	 */
	std::clock_t getStartTime();

	/**
	 * Function for get current time
	 * @return std::clock_t is current time
	 */
	std::clock_t getCurrentTime();

	/**
	 * Function for get time durration in milliseconds
	 * @return double is time durration in milliseconds
	 */
	double getInferenceTimeInMilliseconds();

	/**
	 * Function for get time durration in seconds
	 * @return double is time durration in seconds
	 */
	double getInferenceTimeInSeconds();

	/**
	 * Function for get FPS
	 * @return double is FPS
	 */
	double getFps();

	/**
	 * Function for get average time duration in milliseconds
	 * @param bool updateVector is used to set if average vector should be updated or no
	 * @return double is average time duration in milliseconds
	 */
	double getInferenceAverageTimeInMilliseconds(bool updateVector);

	/**
	 * Function for get average FPS	 
	 * @return double is average FPS
	 */
	double getAverageFps();
};

#endif