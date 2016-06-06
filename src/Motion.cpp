#include "Motion.h"

Motion::Motion()
{
    motionMat(0,0) = 1;
    motionMat(0,1) = 0;
    motionMat(0,2) = 0;
    motionMat(1,0) = 0;
    motionMat(1,1) = 1;
    motionMat(1,2) = 0;

}

Motion::Motion(std::vector< cv::Point2f > Pt1, std::vector< cv::Point2f > Pt2)
{


    //Para estimateRigidTransform en CV::Mat
    cv::Mat objectM = cv::Mat(Pt1);
    cv::Mat ftM = cv::Mat(Pt2);

    motionMat = cv::estimateRigidTransform(objectM,ftM,true);

}

float Motion::getOrientation(){

    //Vamos a fijar una sola orientación
    float rho = float(atan(double(-(motionMat(0,1)))/double(motionMat(0,0))));

    return rho;

}

float Motion::getScale(){

    float angle = getOrientation();

    float sX = motionMat(0,0) / cos(angle);

    float sY = motionMat(1,1) / cos(angle);

    std::cout << std::endl << "ESCALAS (X e Y): " << sX << "\t" << sY;

    float s = (sX + sY) / float(2);

    return s;
}

cv::Point2f Motion::getTraslation(){

    cv::Point2f translation;

    translation.x = motionMat(0,2);
    translation.y = motionMat(1,2);

    return translation;

}
