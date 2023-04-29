#pragma once

#ifndef	BARRIERS_H_
#define BARRIERS_H_

#include <iostream>
#include <vector> 

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/*! Barriers class contains a all detected barriers */
class Barriers {

public:

	/**
	* Default constructor for Barriers class
	*/
	Barriers();

	/** Enum type store state of current road crossing barriers
	*/
	enum BarriersState { open, down, none };

	/**
	* Function set a barriers state
	*
	* @param Barriers::BarriersState state is state of detected barriers
	*/
	void setBarriersState(Barriers::BarriersState state);

	/**
	* Function get a barriers state
	*
	* @return enum state of detected barriers
	*/
	enum BarriersState getBarriersState();

	/**
	* Function get a true or false if detected barriers is down
	*
	* @return bool, true or false if detected barriers is down
	*/
	bool isBarrierDown();

private:

	BarriersState barriersState; /*!< Enum type store state of current road crossing barriers */
};

#endif