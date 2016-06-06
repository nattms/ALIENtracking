#ifndef TRACKER_H
#define TRACKER_H
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include "CharacteristicsPoints.h"

class Tracker
{
public:

    cv::Point2f position; //centro
    float scale; //media de escala x e y
    float rotation;

    // Contructores parametrizamos

    Tracker();
    Tracker(cv::Point2f pos, float s, float rho);

    cv::RotatedRect ShowTracker(cv::Size2f sizeT, cv::Mat image, std::vector<cv::KeyPoint> pointsK);

    //void Tracker::boxTracker(cv::Mat_<float> transformation, cv::RotatedRect region);

//    cv::Mat getHomography(CharacteristicsPoints ft, CharacteristicsPoints Object);

//    void getHomographyItems(cv::Mat homography);

};

#endif // TRACKER_H
