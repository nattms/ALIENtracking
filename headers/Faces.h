#ifndef FACES_H
#define FACES_H

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>

class Faces{

public:

    std::vector<cv::Rect> detectAndDisplayFace(cv::Mat frame, cv::CascadeClassifier face_cascade,
        cv::String face_cascade_name);


        void dimeHola ();
};


#endif // FACES_H
