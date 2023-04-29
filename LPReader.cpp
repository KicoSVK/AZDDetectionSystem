#include "LPReader.h"

#if __has_include("cameraGrabModule.h")    

LPReader::LPReader() {
    ocr = new tesseract::TessBaseAPI();
    ocr->Init(NULL, "eng");
    }

LPReader::~LPReader() {
    //delete ocr;
}

cv::Mat LPReader::enhanceImage(cv::Mat origImage)
{
    cv::Mat enhancedImage;

    if (origImage.channels() < 3)
        enhancedImage = origImage;
    else
        cvtColor(origImage, enhancedImage, cv::COLOR_BGR2GRAY);
    equalizeHist(enhancedImage, enhancedImage);
    
    int newHeight = 20;
    int newWidth = int(round(newHeight * 4.0));

        cv::Mat kernel3 = (cv::Mat_<double>(3, 3) << 0, -1, 0,
        -1, 5, -1,
        0, -1, 0);
    cv::filter2D(enhancedImage, enhancedImage, -1, kernel3, cv::Point(-1, -1), 15, cv::BORDER_DEFAULT);
    
    adaptiveThreshold(enhancedImage, enhancedImage, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 3);

    cv::Mat edges;
    cv::Mat cannyImage;
   
    cv::resize(enhancedImage, enhancedImage, cv::Size(newWidth, newHeight), cv::INTER_CUBIC);
    return enhancedImage;
}

cv::Mat LPReader::HSVFilter(cv::Mat colorImage)
{
    cv::Mat mask;
    cv::Mat HSV;
    cv::Mat masked;
    cv::cvtColor(colorImage, HSV, cv::COLOR_BGR2HSV);
    cv::inRange(HSV, cv::Scalar(100, 80, 80), cv::Scalar(140, 255, 255), mask);   
    cv::bitwise_not(mask, mask);
    cv::bitwise_and(colorImage, colorImage, masked, mask);
    
    return masked;
}

std::string LPReader::removeUnpermittedChars(std::string stringToClean)
{
    std::string substring = "";
    for (std::string::iterator i = stringToClean.begin(); i != stringToClean.end(); i++)
    {
        if (!isalpha(stringToClean.at(i - stringToClean.begin())) && !isdigit(stringToClean.at(i - stringToClean.begin())))
        {
            stringToClean.erase(i);
            i--;
        }
    }
    return stringToClean;
}

std::vector<cv::Point> LPReader::getLPRectangle(cv::Mat colored)
{
    cv::Mat edges;
    cv::Mat cannyWithFrame;
    cv::Mat grayscale;
    
    cvtColor(colored, grayscale, cv::COLOR_BGR2GRAY);
    adaptiveThreshold(grayscale, grayscale, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 3);

    grayscale.copyTo(cannyWithFrame);
    
    cv::threshold(grayscale, grayscale, 0, 255, cv::THRESH_OTSU);
    
    cv::Canny(grayscale, edges, 30, 100);
    cv::Mat approxPoly_mask(edges.rows, edges.cols, CV_8UC1);
    approxPoly_mask = cv::Scalar(0);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    
    std::vector<int> indices(contours.size());
    iota(indices.begin(), indices.end(), 0);

    sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
        return contours[lhs].size() > contours[rhs].size();
        });

    std::vector<std::vector<cv::Point> >hull(1);
    cv::convexHull(cv::Mat(contours[indices[0]]), hull[0], false);

    std::vector<cv::Point>findedPoints;
    cv::Point2f rect_points[4];
    cv::RotatedRect minRect = minAreaRect(contours[indices[0]]);

    minRect.points(rect_points);

    for (int j = 0; j < 4; j++)
    {
        line(cannyWithFrame, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(80));
    }
    
    cv::drawContours(approxPoly_mask, hull, 0, cv::Scalar(255));
    
    for (int indexPoint = 0; indexPoint < 4; indexPoint++)
    {       
        findedPoints.push_back(cv::Point(int(round(rect_points[indexPoint].x)), int(round(rect_points[indexPoint].y))));       
    }

    return(findedPoints);
}

std::tuple<std::string, int> LPReader::ORCRead(cv::Mat frameWithLP, tesseract::PageSegMode segMode)
{
    std::string outText = "";           
    frameWithLP = enhanceImage(frameWithLP);   
    std::lock_guard<std::mutex> l(_mutexForOcrEngine);
    ocr->SetPageSegMode(segMode);    
    ocr->SetImage(frameWithLP.data, frameWithLP.cols, frameWithLP.rows, frameWithLP.channels(), frameWithLP.step);
    const char* OCROutput= ocr->GetUTF8Text();
    if (OCROutput != NULL)
        outText = std::string(ocr->GetUTF8Text());
    else
        std::cout << "OCR cant read!" << std::endl;
    outText = removeUnpermittedChars(outText);

    int confidence = ocr->MeanTextConf();
    return { outText, confidence };
}

std::tuple<std::string, int> LPReader::readLP(cv::Mat frameWithLP)
{
    std::tuple<std::string, int> lpOutput;
    try {
        cv::Mat gray_image;

        int newWidth = 260;
        float width = frameWithLP.cols;
        float height = frameWithLP.rows;

        resize(frameWithLP, frameWithLP, cv::Size(newWidth, (height / width) * newWidth), cv::INTER_CUBIC);

        cv::Mat input_grey;
        cvtColor(frameWithLP, input_grey, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point> card_corners = getLPRectangle(frameWithLP);

        if (card_corners.size() == 4)
        {
            std::vector<std::pair<float, float>> pointsToSort;

            pointsToSort.push_back(std::make_pair(card_corners[0].x, card_corners[0].y));
            pointsToSort.push_back(std::make_pair(card_corners[1].x, card_corners[1].y));
            pointsToSort.push_back(std::make_pair(card_corners[2].x, card_corners[2].y));
            pointsToSort.push_back(std::make_pair(card_corners[3].x, card_corners[3].y));

            std::vector<cv::Point> sortededPoints;
            sort(pointsToSort.begin(), pointsToSort.end(), sortBBox1);

            cv::Point topLeft = cv::Point(pointsToSort[0].first, pointsToSort[0].second);
            cv::Point bottomRight = cv::Point(pointsToSort[3].first, pointsToSort[3].second);

            sort(pointsToSort.begin(), pointsToSort.end(), sortBBox2);

            cv::Point topRight = cv::Point(pointsToSort[0].first, pointsToSort[0].second);
            cv::Point bottomLeft = cv::Point(pointsToSort[3].first, pointsToSort[3].second);

            sortededPoints.push_back(topLeft);
            sortededPoints.push_back(topRight);
            sortededPoints.push_back(bottomLeft);
            sortededPoints.push_back(bottomRight);

            cv::Rect LPRectangle = boundingRect(card_corners);

            std::vector<cv::Point> tempCorners;
            std::vector<cv::Point> sortedCorners;

            cv::Scalar color = cv::Scalar(256, 256, 256);
            cv::Mat warpedCard(LPRectangle.height - (LPRectangle.height * 0.0), LPRectangle.width, CV_8UC3);

            cv::Mat homography = findHomography(sortededPoints, std::vector<cv::Point>{cv::Point(0, 0), cv::Point(warpedCard.cols, 0), cv::Point(0, warpedCard.rows), cv::Point(warpedCard.cols, warpedCard.rows)});

            warpPerspective(frameWithLP, warpedCard, homography, cv::Size(warpedCard.cols, warpedCard.rows));
            lpOutput = ORCRead(warpedCard, tesseract::PSM_SINGLE_BLOCK);

        }
        else {
            lpOutput = ORCRead(frameWithLP, tesseract::PSM_SINGLE_BLOCK);
        }

    }
    catch (const std::exception& e) // caught by reference to base             
    {
        std::cout << "Exception was caught. Message: '" << e.what() << std::endl;
        std::cout << __LINE__ << " " << __func__ << std::endl;

        std::ofstream fileToWrite;
        fileToWrite.open("exceptions.log");
        fileToWrite << "Exception was caught. Message: '" << e.what() << "'\n";
        fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
        fileToWrite.close();

    }
    return lpOutput;
}

bool LPReader::sortBBox1(const std::pair<float, float>& point1, const std::pair<float, float>& point2)
{
    float s1 = point1.first + point1.second;
    float s2 = point2.first + point2.second;
    return s1 < s2;
}

bool LPReader::sortBBox2(const std::pair<float, float>& point1, const std::pair<float, float>& point2)
{
    float s1 = point1.second - point1.first;
    float s2 = point2.second - point2.first;
    return s1 < s2;
}

#else

LPReader::LPReader() {

}

std::tuple<std::string, int> LPReader::readLP(cv::Mat frameWithLP) {
    return { std::string(), -1 };
}

#endif