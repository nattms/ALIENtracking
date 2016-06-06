#ifndef CharacteristicsPoints_H
#define CharacteristicsPoints_H


#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>

class CharacteristicsPoints{


    public:

        std::vector<cv::KeyPoint> KPoints;
        cv::Mat descriptorsSIFT;


        void matchesClass(CharacteristicsPoints A, cv::BFMatcher matcher, CharacteristicsPoints B, double lambda);
        void DeleteRepeatedPoints(CharacteristicsPoints A,CharacteristicsPoints B);
        void DeleteRepeatedPoints();
        void SortingSimilarity(CharacteristicsPoints A, cv::BFMatcher matcher, CharacteristicsPoints B);

        void PointsInRotatedRect(cv::RotatedRect rB);

        void PointsInRotatedRect(cv::RotatedRect rB, CharacteristicsPoints A, cv::Mat image);

        void TransformateChrPoints(cv::Mat InvertTransform, CharacteristicsPoints A);

        void MergeChrPoints(CharacteristicsPoints A);

        void RandomLimitedSelection(int N);


        std::vector<cv::Point2f> ConvertTo2f();

};


#endif // CharacteristicsPoints_H
