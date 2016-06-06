#include "tracker.h"
#include "Motion.h"


Tracker::Tracker(){

    position = cv::Point2f(0,0);
    rotation = 0;
    scale = 1;
}

Tracker::Tracker(cv::Point2f pos, float s, float rho){

    position = pos;
    rotation = rho;
    scale = s;

}

cv::RotatedRect Tracker::ShowTracker(cv::Size2f sizeT, cv::Mat image, std::vector<cv::KeyPoint> pointsK)
{
    //cv::Point2f centreR = cv::Point2f(position.x + sizeT.width/2, position.y + sizeT.height/2);
    cv::RotatedRect Track(position,cv::Size2f(scale*sizeT.width,scale*sizeT.height),rotation);
    cv::Point2f points[4];
    Track.points(points);
    for (int i = 0; i < 4; i++)
        line(image, points[i], points[(i+1)%4], cv::Scalar(0,0,255));

    cv::drawKeypoints(image,pointsK,image,cv::Scalar(0,0,255));

    cv::imshow("Cap 1", image);

    return Track;
}

/*cv::RotatedRect Tracker::boxTracker(cv::Mat_<float> transformation, cv::RotatedRect region)
{
    cv::Mat_<float> pointPos(3,1,6);
    pointPos(0) = position.x;
    pointPos(1) = position.y;
    pointPos(2) = 1;

    cv::Mat_<float> centerN = (cv::Mat_<float>) (transformation * pointPos);

    // Pasamos el centro al tracker
    position.x = centerN(0);
    position.y = centerN(1);



    if (rotation != 0){

        cv::Point2f ulPoint;
        float d = sqrt((pow(region.size.width))+(pow(region.size.height)))/2;
        float theta = region.angle + atan2(region.size.height, region.size.width);
        ulPoint.x = region.center.x - d*sin(theta);
        ulPoint.y = region.center.y - d*cos(theta);


        cv::Point2f diff1 = centerN - ulPoint;
        cv::Point2f diff2 = centerN - newOrigin;
        float rho = atan2(diff2.y,diff2.x) - atan2(diff1.y,diff1.x);
        rotation += rho;
        if (rotation<0) rotation = 2*PI+rotation;
        if (rotation>=2*PI) rotation = rotation-2*PI;

    }


}*/

//cv::Mat Tracker::getHomography(CharacteristicsPoints ft, CharacteristicsPoints Object){

//    //RANSAC
//    cv::BFMatcher matching;
//    CharacteristicsPoints prevObject;

//    //Se crea en prevObject el objecto "Objeto" ordenado segun la distancia con ft para poder sacar RANSAC
//    prevObject.SortingSimilarity(ft,matching,Object);

//    // Para hacer RANSAC: Necesitamos vectores de puntos
//    std::vector< cv::Point2f > ftPoints, objectPoints;
//    cv::KeyPoint::convert(ft.KPoints,ftPoints);
//    cv::KeyPoint::convert(prevObject.KPoints,objectPoints);
//    cv::Mat homography = cv::findHomography(ftPoints,objectPoints,cv::RANSAC);

//    //comprobar
//    return homography;

//}


//void Tracker::getHomographyItems(cv::Mat homography)
//{
//    cv::Mat Mrot,Mtras;
//    cv::decomposeHomographyMat(homography,cv::Mat(),Mrot,Mtras,cv::Mat());
//    std::cout<<Mrot;

//    std::cout << "prueba";
//    //displacement = homography<homography.type()>.at(0,3)



//}

