#include "AppWindow.h"
#include <string>
#include <iostream>

extern int textThicknessInGUI;

AppWindow::AppWindow() {
    lastClickedButtonId = -1;
    displayAppWindow = false;
}

void AppWindow::drawTitleToWindow(cv::Mat& frame, cv::Rect target, int face, int thickness, cv::Scalar color, const std::string& str)
{
    target = cv::Rect(cv::Point(target.x + ((target.width / 100.0) * 5.0), target.y + ((target.height / 100.0) * 5.0)), cv::Point((target.x + target.width) - ((target.width / 100.0) * 5.0), (target.y + target.height) - ((target.height / 100.0) * 5.0)));
    cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
    double scalex = (double)target.width / (double)rect.width;
    double scaley = (double)target.height / (double)rect.height;
    double scale = std::min(scalex, scaley);
    int marginx = scale == scalex ? 0 : (int)((double)target.width * (scalex - scale) / scalex * 0.5);
    int marginy = scale == scaley ? 0 : (int)((double)target.height * (scaley - scale) / scaley * 0.5);
    cv::putText(frame, str, cv::Point(target.x + marginx, target.y + target.height - (marginy * 2)), face, scale, color, thickness, 8, false);
}

void AppWindow::drawContentToWindow(cv::Mat& frame, cv::Rect target, int face, int thickness, cv::Scalar color, const std::string& str)
{
    // https://stackoverflow.com/questions/50353884/calculate-text-size
    target = cv::Rect(cv::Point(target.x + ((target.width / 100.0) * 5.0), target.y + ((target.height / 100.0) * 5.0)), cv::Point((target.x + target.width) - ((target.width / 100.0) * 5.0), (target.y + target.height) - ((target.height / 100.0) * 5.0)));
    cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
    double scalex = (double)target.width / (double)rect.width;
    double scaley = (double)target.height / (double)rect.height;
    double scale = std::min(scalex, scaley);
    int marginx = scale == scalex ? 0 : (int)((double)target.width * (scalex - scale) / scalex * 0.5);
    int marginy = scale == scaley ? 0 : (int)((double)target.height * (scaley - scale) / scaley * 0.5);
    cv::putText(frame, str, cv::Point(target.x + marginx, target.y + target.height - marginy), face, scale, color, thickness, 8, false);
}


void AppWindow::drawWindow(cv::Mat& frame, std::string windowTitleText, std::string windowContentText01, std::string windowContentText02) {
    frameCenterX = frame.cols / 2;
    frameCenterY = frame.rows / 2;
    windowWidth = (frame.cols / 100) * 40.0;
    windowHeight = (frame.rows / 100) * 30.0;

    startWindowPosition = cv::Point(frameCenterX - (windowWidth / 2), frameCenterY - (windowHeight / 2));
    windowRect = cv::Rect(startWindowPosition.x, startWindowPosition.y, windowWidth, windowHeight);

    // Main window
    cv::rectangle(frame, windowRect, cv::Scalar(230, 170, 120), -1);
    cv::rectangle(frame, windowRect, cv::Scalar(255, 25, 25), 4);

    // Window title text
    drawTitleToWindow(frame, windowRect, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(125, 25, 25), windowTitleText);

    // Buttons
    button01 = cv::Rect(windowRect.x + ((windowRect.width / 100.0) * 5.0), (windowRect.y + windowRect.height) - ((windowRect.height / 100.0) * 70.0), windowRect.width - ((windowRect.width / 100) * 10), (windowRect.height / 100) * 30);
    cv::rectangle(frame, button01, cv::Scalar(255, 255, 255), -1);
    cv::rectangle(frame, button01, cv::Scalar(255, 25, 25), 2);
    drawContentToWindow(frame, button01, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(255, 25, 25), windowContentText01);

    button02 = cv::Rect(windowRect.x + ((windowRect.width / 100.0) * 5.0), (windowRect.y + windowRect.height) - ((windowRect.height / 100.0) * 35.0), windowRect.width - ((windowRect.width / 100) * 10), (windowRect.height / 100) * 30);
    cv::rectangle(frame, button02, cv::Scalar(255, 255, 255), -1);
    cv::rectangle(frame, button02, cv::Scalar(255, 25, 25), 2);
    drawContentToWindow(frame, button02, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(255, 25, 25), windowContentText02);
}

void AppWindow::drawWindow(cv::Mat& frame, std::string windowTitleText, std::string windowContentText01, std::string windowContentText02, std::string windowContentText03) {
    frameCenterX = frame.cols / 2;
    frameCenterY = frame.rows / 2;
    windowWidth = (frame.cols / 100) * 40.0;
    windowHeight = (frame.rows / 100) * 45.0;

    startWindowPosition = cv::Point(frameCenterX - (windowWidth / 2), frameCenterY - (windowHeight / 2));
    windowRect = cv::Rect(startWindowPosition.x, startWindowPosition.y, windowWidth, windowHeight);

    // Main window
    cv::rectangle(frame, windowRect, cv::Scalar(230, 170, 120), -1);
    cv::rectangle(frame, windowRect, cv::Scalar(255, 25, 25), 4);

    // Window title text
    drawTitleToWindow(frame, windowRect, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(125, 25, 25), windowTitleText);

    // Buttons
    button01 = cv::Rect(windowRect.x + ((windowRect.width / 100.0) * 5.0), (windowRect.y + windowRect.height) - ((windowRect.height / 100.0) * 80.0), windowRect.width - ((windowRect.width / 100) * 10), (windowRect.height / 100) * 25);
    cv::rectangle(frame, button01, cv::Scalar(255, 255, 255), -1);
    cv::rectangle(frame, button01, cv::Scalar(255, 25, 25), 2);
    drawContentToWindow(frame, button01, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(255, 25, 25), windowContentText01);

    button02 = cv::Rect(windowRect.x + ((windowRect.width / 100.0) * 5.0), (windowRect.y + windowRect.height) - ((windowRect.height / 100.0) * 55.0), windowRect.width - ((windowRect.width / 100) * 10), (windowRect.height / 100) * 25);
    cv::rectangle(frame, button02, cv::Scalar(255, 255, 255), -1);
    cv::rectangle(frame, button02, cv::Scalar(255, 25, 25), 2);
    drawContentToWindow(frame, button02, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(255, 25, 25), windowContentText02);

    button03 = cv::Rect(windowRect.x + ((windowRect.width / 100.0) * 5.0), (windowRect.y + windowRect.height) - ((windowRect.height / 100.0) * 30.0), windowRect.width - ((windowRect.width / 100) * 10), (windowRect.height / 100) * 25);
    cv::rectangle(frame, button03, cv::Scalar(255, 255, 255), -1);
    cv::rectangle(frame, button03, cv::Scalar(255, 25, 25), 2);
    drawContentToWindow(frame, button03, cv::FONT_HERSHEY_DUPLEX, textThicknessInGUI, cv::Scalar(255, 25, 25), windowContentText03);
}

bool AppWindow::isAppWindowClicked(cv::Point clickPosition) {
    if (windowRect.contains(clickPosition)) {
        return true;
    }
    else {
        return false;
    }
}

int AppWindow::checkButtonClick(cv::Point clickPosition, AppWindow::AppWindowScenario scenario) {
    switch (scenario)
    {
    case AppWindow::TRAFFIC_AREA_SCENARIO:
        if (button01.contains(clickPosition)) {
            lastClickedButtonId = 0;
            return 0;
        }
        else if (button02.contains(clickPosition)) {
            lastClickedButtonId = 1;
            return 1;
        }
        else if (button03.contains(clickPosition)) {
            lastClickedButtonId = 2;
            return 2;
        }
        break;
    case AppWindow::MONITORED_AREA_SCENARIO:
        if (button01.contains(clickPosition)) {
            lastClickedButtonId = 0;
            return 0;
        }
        else if (button02.contains(clickPosition)) {
            lastClickedButtonId = 1;
            return 1;
        }
        else if (button03.contains(clickPosition)) {
            lastClickedButtonId = 2;
            return 2;
        }
        break;
    case AppWindow::NO_DETECTING_AREA_SCENARIO:
        if (button01.contains(clickPosition)) {
            lastClickedButtonId = 0;
            return 0;
        }
        else if (button02.contains(clickPosition)) {
            lastClickedButtonId = 1;
            return 1;
        }
        else if (button03.contains(clickPosition)) {
            lastClickedButtonId = 2;
            return 2;
        }
        break;
    default:
        break;
    }
}

int AppWindow::checkButtonClick(cv::Point clickPosition) {
    
    if (button01.contains(clickPosition)) {
        lastClickedButtonId = 0;
        return 0;
    }
    else if (button02.contains(clickPosition)) {
        lastClickedButtonId = 1;
        return 1;
    }
    else if (button03.contains(clickPosition)) {
        lastClickedButtonId = 2;
        return 2;
    }
}

int AppWindow::getLastClickedButtonId() {  
    int buttonId = lastClickedButtonId;
    lastClickedButtonId = -1;
    return buttonId;
}

void AppWindow::showAppWindow() {
    displayAppWindow = true;
}


void AppWindow::hideAppWindow() {
    displayAppWindow = false;
}

bool AppWindow::isDisplayAppWindow() {
    return displayAppWindow;
}

