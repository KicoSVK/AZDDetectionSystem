#pragma once
#ifndef	APPGUI_H_
#define APPGUI_H_

#include <iostream>
#include <vector> 

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "config.h"

/*! AppWindow class contains a app window */
class AppWindow {

public:

	/**
	 * Default constructor for AppWindow class
	 */
	AppWindow();

	/** Subset of this scenario are supported
	*/
	enum AppWindowScenario {
		MONITORED_AREA_SCENARIO = 0, //!< monitored area selecting
		TRAFFIC_AREA_SCENARIO = 1, //!< traffic area selecting
		TRAFFIC_LIGHT_AREA_SCENARIO = 2, //!< traffic light area selecting
		NO_DETECTING_AREA_SCENARIO = 3, //!< no detecting area selecting
		UNPERMITTED_VEHICLE_AREA_SCENARIO = 4, //!< unpermitted vehicle area selecting
		CONTROL_LINE_SCENARIO = 5 //!< control line selecting
	};

	/**
	 * Function draw a window
	 *
	 * @param cv::Mat frame is frame for display an image
	 * @param string windowTitleText is text for a window title
	 * @param string windowContentText01 is text for a window button
	 * @param string windowContentText02 is text for a window button
	 */
	void drawWindow(cv::Mat& frame, std::string windowTitleText, std::string windowContentText01, std::string windowContentText02);

	/**
	 * Function draw a window
	 *
	 * @param cv::Mat frame is frame for display an image
	 * @param string windowTitleText is text for a window title
	 * @param string windowContentText01 is text for a window button
	 * @param string windowContentText02 is text for a window button
	 * @param string windowContentText03 is text for a window button
	 */
	void drawWindow(cv::Mat& frame, std::string windowTitleText, std::string windowContentText01, std::string windowContentText02, std::string windowContentText03);

	/**
	 * Function return true or false
	 * if is clicked inside AppWindow
	 *
	 * @param clickPosition is cv::Point of clicked position in frame
	 * @return true or false, if is clicked inside AppWindow
	 */
	bool isAppWindowClicked(cv::Point clickPosition);

	/**
	 * Function return id of clicked button
	 *
	 * @param clickPosition is cv::Point of clicked position in frame
	 * @param scenario is AppWindow::AppWindowScenario supported scenarion in AppWindow
	 * @return id of clicked button
	 */
	int checkButtonClick(cv::Point clickPosition, AppWindow::AppWindowScenario scenario);

	/**
	 * Function return id of clicked button
	 *
	 * @param clickPosition is cv::Point of clicked position in frame
	 * @return id of clicked button
	 */
	int checkButtonClick(cv::Point clickPosition);

	/**
	 * Function return id of clicked button
	 *
	 * @return id of clicked button
	 */
	int getLastClickedButtonId();

	/**
	* Function enable show a window
	*/
	void showAppWindow();

	/**
	 * Function disable show a window
	 */
	void hideAppWindow();

	/**
	 * Function return true or false
	 * if window is displayed
	 *
	 * @return true or false if window is displayed
	 */
	bool isDisplayAppWindow();

private:

	/**
	 * Function draw a content of window
	 *
	 * @param cv::Mat frame is frame for display an image
	 * @param cv::Rect target is position and size for display an window
	 * @param int face is font type
	 * @param int thickness is font thickness
	 * @param cv::Scalar color is color of font
	 * @param const std::string& str is text for display
	 */
	void drawContentToWindow(cv::Mat& mat, cv::Rect target, int face, int thickness, cv::Scalar color, const std::string& str);

	/**
	 * Function draw a title of window
	 *
	 * @param cv::Mat frame is frame for display an image
	 * @param cv::Rect target is position and size for display an window
	 * @param int face is font type
	 * @param int thickness is font thickness
	 * @param cv::Scalar color is color of font
	 * @param const std::string& str is text for display
	 */
	void drawTitleToWindow(cv::Mat& mat, cv::Rect target, int face, int thickness, cv::Scalar color, const std::string& str);

	// Window color
	cv::Scalar buttonColor; /*!< Button background color */
	cv::Scalar windowBackgroundColor; /*!< Window background color */
	cv::Scalar windowBorderColor; /*!< Window border color */
	cv::Scalar windowTitleTextColor; /*!< Window title text color */
	cv::Scalar windowButtonTextColor; /*!< Window button text color */

	// Window size properties
	int frameCenterX; /*!< Window frame center X */
	int frameCenterY; /*!< Window frame center X */
	double windowWidth; /*!< Window frame width */
	double windowHeight; /*!< Window frame height */

	// Window position
	cv::Point startWindowPosition; /*!< Window start position cv::Point */
	cv::Rect windowRect; /*!< Window dimensions cv::Rect*/

	// Button in window
	cv::Rect button01; /*!< Button dimensions cv::Rect */
	cv::Rect button02; /*!< Button dimensions cv::Rect */
	cv::Rect button03; /*!< Button dimensions cv::Rect */
	//cv::Rect button03;
	//cv::Rect button04;
	//cv::Rect button05;

	// Id of clicked button
	int lastClickedButtonId; /*!< Last clicked button ID */

	// State of window display
	bool displayAppWindow; /*!< Bool state of displaying app window */
};

#endif