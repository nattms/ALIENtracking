#include "homographyMatrix.h"


HomographyMatrix::HomographyMatrix()
{

    homographyMat = cv::Mat::eye(cv::Size(3,3),6);
//    homographyMat(0,0) = cos(0);
//    homographyMat(0,1) = -sin(0);
//    homographyMat(1,0) = sin(0);
//    homographyMat(1,1) = cos(0);
//    homographyMat(0,2) = 0;
//    homographyMat(1,2) = 0;


}


HomographyMatrix::HomographyMatrix(std::vector< cv::Point2f > Pt1,std::vector< cv::Point2f > Pt2){

    for(unsigned i = 0; i < Pt1.size(); ++i){
        std::cout << Pt1[i] << "   " << Pt2[i] << std::endl;
    }
    homographyMat = cv::findHomography(Pt1,  Pt2,cv::RANSAC, 4, cv::noArray(), 1000, 0.999);

    std::cout << homographyMat << std::endl;
}

float HomographyMatrix::getOrientation(){

    //Vamos a fijar una sola orientación
    float rho = float(atan(double(-(homographyMat(0,1)))/double(homographyMat(0,0))));
    //8float rhoY = atan2(double(homographyMat(1,1)),double(homographyMat(1,0)));
    //cv::Point2f angles(rhoX,rhoY);
    return rho;

}

void HomographyMatrix::printfHomographyMat()
{
    std::cout<< homographyMat(0,0) << "\t" << homographyMat(0,1) <<"\t" << homographyMat(0,2) << "\n" << homographyMat(1,0) << "\t" << homographyMat(1,1) << "\t" << homographyMat(1,2) << "\n" << homographyMat(2,0) << "\t" << homographyMat(2,1) << "\t" << homographyMat(2,2);
}

float HomographyMatrix::getScale(){

    float angle = getOrientation();

    float sX = homographyMat(0,0) / cos(angle);

    float sY = homographyMat(1,1) / cos(angle);

    std::cout << std::endl << "ESCALAS (X e Y): " << sX << "\t" << sY;

    float s = (sX + sY) / float(2);

    return s;
}

cv::Point2f HomographyMatrix::getTraslation(){

    cv::Point2f position;

    position.x = homographyMat(0,2);
    position.y = homographyMat(1,2);

    return position;

}


