#include "InferenceTimer.h"

// InferenceTimer initialization function
void InferenceTimer::timerInitialization() {
	this->startTime = std::clock();
	this->t_start = std::chrono::high_resolution_clock::now();
}

// Initialize start time
void InferenceTimer::setStartTime(std::clock_t startTime) {
	this->startTime = startTime;
}

// Get start time
std::clock_t InferenceTimer::getStartTime() {
	if (startTime == 0) {
		return NAN;
	}
	else
		return startTime;
}

// Get current time
std::clock_t InferenceTimer::getCurrentTime() {
	return std::clock();
}

// Get inference time in milli seconds
double InferenceTimer::getInferenceTimeInMilliseconds() {
	double time = 0.0;
	if (getStartTime() == NAN) {
		std::cout << "InferenceTimer is not initialized !" << std::endl;
		return NAN;
	}
	else {
		#ifdef __linux__
			//return ((std::clock() - getStartTime()) / 1000);
			time = (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count());			
		#else 
			time = (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count());					
		#endif
		return time;
	}
		
}

// Get inference time in seconds
double InferenceTimer::getInferenceTimeInSeconds() {
	if (getStartTime() == NAN) {
		std::cout << "InferenceTimer is not initialized !" << std::endl;
		return NAN;
	}
	else
		return ((std::clock() - getStartTime()) / (double)CLOCKS_PER_SEC);
}

// Get fps
double InferenceTimer::getFps() {
	return 1 / getInferenceTimeInSeconds();
}

// get average getInferenceTimeInSeconds
double InferenceTimer::getInferenceAverageTimeInMilliseconds(bool updateVector) {
	double time = 0.0;
	if (getStartTime() == NAN) {
		std::cout << "InferenceTimer is not initialized !" << std::endl;
		return NAN;
	}
	else {		
		time = getInferenceTimeInMilliseconds();
		
		if (updateVector)
		{
			if (averageVector.size() > 10)
				averageVector.erase(averageVector.begin());
			averageVector.push_back(time);
		}

		time = 0.0;
		for (auto oneTime : averageVector) {
			time += oneTime;
		}

		time = time / (averageVector.size() );		
		return time;
	}
}

// Get average fps
double InferenceTimer::getAverageFps() {

	return 1 / (getInferenceAverageTimeInMilliseconds(false)/1000);
}