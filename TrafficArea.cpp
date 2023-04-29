#include "TrafficArea.h"

TrafficArea::TrafficArea()
{
    isSelectedTrafficAreaDirection = false;
}

void TrafficArea::addPointToTrafficArea(cv::Point point)
{
	trafficArea.push_back(point);
}

std::vector<cv::Point> TrafficArea::getTrafficArea()
{
	return trafficArea;
}

void TrafficArea::removeTrafficArea()
{
	trafficArea.clear();
}

void TrafficArea::removePointInTrafficArea(int index)
{
	trafficArea.erase(trafficArea.begin() + index);
}

void TrafficArea::updatePointInTrafficArea(int index, cv::Point point)
{
	trafficArea.at(index) = point;
}

int TrafficArea::getSizeOfTrafficArea()
{
	return trafficArea.size();
}

enum TrafficArea::TrafficAreaDirection TrafficArea::getTrafficAreaDirection()
{
	return TrafficArea::trafficAreaDirection;
}

bool TrafficArea::isObjectMovingInTrafficAreaAllowed()
{
	return false;
}

int TrafficArea::getIndexOfNearestPointInTrafficArea(cv::Point point)
{
	for (size_t i = 0; i < trafficArea.size(); i++)
	{
		if (trafficArea[i].x - 25 < point.x && point.x < trafficArea[i].x + 25 && trafficArea[i].y - 25 < point.y && point.y < trafficArea[i].y + 25) {
			return i;
		}
	}
	return -1;
}

cv::Point TrafficArea::getPointOfNearestPointInTrafficArea(cv::Point point) {
    for (size_t i = 0; i < trafficArea.size(); i++)
    {
        if (trafficArea[i].x - 25 < point.x && point.x < trafficArea[i].x + 25 && trafficArea[i].y - 25 < point.y && point.y < trafficArea[i].y + 25) {
            return trafficArea[i];
        }
    }
    return cv::Point(0,0);
}

bool TrafficArea::isClickedDirectionRect(cv::Point clickPoint)
{
    if (trafficArea.size() > 0) {
        // get moments
        cv::Moments mu = moments(trafficArea, false);
        // get center of moments
        cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

        cv::Rect boundingRect = cv::boundingRect(trafficArea);

        cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05)), cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05)));
        
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

cv::Point2f TrafficArea::rotate2d(const cv::Point2f& inPoint, const double& angRad)
{
    cv::Point2f outPoint;
    //CW rotation
    outPoint.x = std::cos(angRad) * inPoint.x - std::sin(angRad) * inPoint.y;
    outPoint.y = std::sin(angRad) * inPoint.x + std::cos(angRad) * inPoint.y;
    return outPoint;
}

cv::Point2f TrafficArea::rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad)
{
    return rotate2d(inPoint - center, angRad) + center;
}

cv::Rect TrafficArea::setTrafficAreaDirection(cv::Point clickPoint, float alpha_slider)
{
    //// get moments
    //cv::Moments mu = moments(trafficArea, false);
    //// get center of moments
    //cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

    //cv::Rect boundingRect = cv::boundingRect(trafficArea);

    //cv::Rect tmpRect(cv::Point(cMu.x - (boundingRect.width * 0.05), cMu.y - (boundingRect.width * 0.05)), cv::Point(cMu.x + (boundingRect.width * 0.05), cMu.y + (boundingRect.width * 0.05)));
    //
    //// LT
    //cv::Rect lt(cv::Point(tmpRect.x, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
    //// RT
    //cv::Rect rt(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
    //// LB
    //cv::Rect lb(cv::Point(tmpRect.x, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));
    //// RB
    //cv::Rect rb(cv::Point(tmpRect.x + tmpRect.width / 2, tmpRect.y + tmpRect.height / 2), cv::Size(tmpRect.width / 2, tmpRect.height / 2));

    #define M_PI       3.14159265358979323846   // pi

    // get moments
    cv::Moments mu = cv::moments(trafficArea, false);
    // get center of moments
    cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

    cv::Rect boundingRect = cv::boundingRect(trafficArea);

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
        trafficAreaDirection = TrafficAreaDirection::leftTop;
        isSelectedTrafficAreaDirection = true;
        return lt;
    }
    else if (cv::pointPolygonTest(rtRect, clickPoint, false) > 0) {
        trafficAreaDirection = TrafficAreaDirection::rightTop;
        isSelectedTrafficAreaDirection = true;
        return rt;
    }
    else if (cv::pointPolygonTest(lbRect, clickPoint, false) > 0) {
        trafficAreaDirection = TrafficAreaDirection::leftBottom;
        isSelectedTrafficAreaDirection = true;
        return lb;
    }
    else if (cv::pointPolygonTest(rbRect, clickPoint, false) > 0) {
        trafficAreaDirection = TrafficAreaDirection::rightBottom;
        isSelectedTrafficAreaDirection = true;
        return rb;
    }
    return cv::Rect();
}

// none = 0, leftTop = 1, rightTop = 2, leftBottom = 3, rightBottom = 4
void TrafficArea::setTrafficAreaDirection(int index)
{
    if (index == 0) {
        trafficAreaDirection = TrafficAreaDirection::none;
    }
    else if (index == 1) {
        trafficAreaDirection = TrafficAreaDirection::leftTop;
        isSelectedTrafficAreaDirection = true;
    }
    else if (index == 2) {
        trafficAreaDirection = TrafficAreaDirection::rightTop;
        isSelectedTrafficAreaDirection = true;
    }
    else if (index == 3) {
        trafficAreaDirection = TrafficAreaDirection::leftBottom;
        isSelectedTrafficAreaDirection = true;
    }
    else if (index == 4) {
        trafficAreaDirection = TrafficAreaDirection::rightBottom;
        isSelectedTrafficAreaDirection = true;
    }
}

//void TrafficArea::setTrafficAreaDirection(TrafficArea::TrafficAreaDirection direction)
//{
//    trafficAreaDirection = direction;
//}

bool TrafficArea::getIsSelectedTrafficAreaDirection()
{
    return TrafficArea::isSelectedTrafficAreaDirection;
}

cv::Point TrafficArea::getCenterOfTrafficArea()
{
    cv::Moments mu = moments(trafficArea, false);
    // get center of moments
    cv::Point cMu(mu.m10 / mu.m00, mu.m01 / mu.m00);

    return cMu;
}

void TrafficArea::setTrafficAreaIndex(int index)
{
    this->trafficAreaIndex = index;
}

int TrafficArea::getTrafficAreaIndex()
{
    return this->trafficAreaIndex;
}

bool TrafficArea::isDirectionOfMovementAllowed(TrafficArea::TrafficAreaDirection movement)
{
    // left top direction
    if (movement == TrafficArea::TrafficAreaDirection::leftBottom || movement == TrafficArea::TrafficAreaDirection::rightTop || movement == TrafficArea::TrafficAreaDirection::leftTop) {
        if (TrafficArea::getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::leftTop)
            return true;
    }
    // left bottom direction
    if (movement == TrafficArea::TrafficAreaDirection::leftTop || movement == TrafficArea::TrafficAreaDirection::leftBottom || movement == TrafficArea::TrafficAreaDirection::rightBottom) {
        if (TrafficArea::getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::leftBottom)
            return true;
    }
    // right bottom direction
    if (movement == TrafficArea::TrafficAreaDirection::leftBottom || movement == TrafficArea::TrafficAreaDirection::rightTop || movement == TrafficArea::TrafficAreaDirection::rightBottom) {
        if (TrafficArea::getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::rightBottom)
            return true;
    }
    // right top direction
    if (movement == TrafficArea::TrafficAreaDirection::leftTop || movement == TrafficArea::TrafficAreaDirection::rightBottom || movement == TrafficArea::TrafficAreaDirection::rightTop) {
        if (TrafficArea::getTrafficAreaDirection() == TrafficArea::TrafficAreaDirection::rightTop)
            return true;
    }
    return false;
}
