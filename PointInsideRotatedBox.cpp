#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video.hpp>


/*bool PointInsideRotatedBox(cv::Point2f pt, cv::RotatedRect rB){

    bool inside = false;

    //down left
    cv::Point2f points[4];
    rB.points(points);

    //Corners
    cv::Point2f A = points[0];
    cv::Point2f B = points[1];
    cv::Point2f C = points[2];
    cv::Point2f D = points[3];

    cv::Point2f Apt = pt - A;
    cv::Point2f AB = B - A;
    float ABm = cv::norm(AB);

    float a = Apt.dot(AB);

    //si a es mayor que 0 y menor que el modulo de AB

    cv::Point2f Bpt = pt - B;
    cv::Point2f BC = C - B;
    float BCm = cv::norm(BC);;

    float b = Bpt.dot(BC);

    cv::Point2f Cpt = pt - C;
    cv::Point2f CD = D - C;
    float CDm = cv::norm(CD);;

    float c = Cpt.dot(CD);


    cv::Point2f Dpt = pt - D;
    cv::Point2f DA = A - D;
    float DAm = cv::norm(DA);;

    float d = Dpt.dot(DA);


    if (a > 0 && a < ABm && b > 0 && BCm && c > 0 && c < CDm && d > 0 && d < DAm){

        inside = true;

    }



}*/




