//
// Created by lars on 10/14/19.
//

#include "Vision.h"

Vision::Vision()
{

}

void Vision::cameraCallbackRaw(ConstImageStampedPtr &msg)
{
    width = msg->image().width();
    height = msg->image().height();
    data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();
    cv::cvtColor(im, im, CV_RGB2BGR);

    mutexCamera.lock();
    cv::imshow("camera raw", im);
    mutexCamera.unlock();

}

void Vision::cameraCallbackHough(ConstImageStampedPtr &msg)
{
    marbleFound = false;

    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();

    cv::cvtColor(im, im, CV_RGB2GRAY);

    GaussianBlur(im, im, cv::Size(9, 9), 2, 0);

    ////
    std::vector<cv::Vec3f> circles;
    static cv::Point current(-1, -1);

    /// Apply the Hough Transform to find the circles
    //
    HoughCircles(im, circles, CV_HOUGH_GRADIENT,
                 1,   // accumulator resolution (size of the image / 2)
                 3000,  // minimum distance between two circles
                 15, // Canny high threshold
                 30, // minimum number of votes
                 0, 0); // min and max radius

    // the following values work okay, but not perfect
//            1,   // accumulator resolution (size of the image / 2)
//            3000,  // minimum distance between two circles
//            20, // Canny high threshold
//            30, // minimum number of votes
//            0, 0); // min and max radius
    int biggestRadius = 0;
    cv::Point closestMarble;
    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++)
        {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(im, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        circle(im, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

//        if (current != center)
//            {
//            std::cout << "Position of the white ball is:" << center << std::endl; //
//            current = center;
//            }
        if (radius > biggestRadius)
            {
            biggestRadius = radius;
            closestMarble = center;
            marbleFound = true;
            }
        }
    ////

    mutexCamera.lock();
    cv::imshow("Camera with Hough circle transform", im);// temp name
    mutexCamera.unlock();

    // from measurement:
    // Camera spread:   ~ 0.5 rad, +-0.25 rad
    //        distance: ~ 2
    //        image x:  ~ 1

    if ( marbleFound == true )
        {
        std::tie(marbleDir, marbleDist) = calculateMarblePos(closestMarble, biggestRadius);
        }

}

std::tuple<bool, float, float> Vision::getMarble()
{
    return std::tuple<bool, float, float>(marbleFound, marbleDir, marbleDist);
}


std::tuple<float, float> Vision::calculateMarblePos(cv::Point center, int radius)
{
    // Calculate marble angle:
    const int cameraWidth = 320;
    float normCenterX = 0.5 - (float(center.x) / float(cameraWidth)); // Normalize to match image width of 1, and set middle of image as 0.
    float marbleDir = atan2(normCenterX, 2);

    // Calculate (estimate) marble distance
    // Assumptions: Triangle between edges of marble and is perpendicular no matter position
    //              Distance "to screen" is 2 no matter the angle

    // Hough Circles outputs a radius,
    float normDiameter = (float(radius)*4) / float(cameraWidth);
    float marbleDist = 2 / normDiameter;




    return std::tuple<float, float> (marbleDir, marbleDist);
}

Vision::~Vision()
{

}





