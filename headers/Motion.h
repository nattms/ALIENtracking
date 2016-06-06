#ifndef MOTION_H
#define MOTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>


class Motion {
    public:

    cv::Mat_<float> motionMat;

    Motion();
    Motion(std::vector< cv::Point2f > Pt1, std::vector< cv::Point2f > Pt2);

    float getScale();
    cv::Point2f getTraslation();
    float getOrientation();





};

#endif // MOTION_H
