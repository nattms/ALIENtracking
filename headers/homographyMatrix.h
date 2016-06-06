#ifndef HOMOGRAPHYMATRIX_H
#define HOMOGRAPHYMATRIX_H

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>

#define PI 3.14159265

class HomographyMatrix
{
public:

    cv::Mat_<float> homographyMat;

    /**
     * @brief HomographyMatrix Crea una matriz identidad.
     */
    HomographyMatrix();

    /**
     * @brief HomographyMatrix Crea una matriz con los valores suministrados. Ambos vectores de puntos
     * deben tener la misma cardinalidad y estar en el mismo orden.
     * @param Pt1 Vector de puntos de la imagen origen
     * @param Pt2 Vector de puntos en la imagen destino
     */
    HomographyMatrix(std::vector< cv::Point2f > Pt1,std::vector< cv::Point2f > Pt2);

    float getScale();
    cv::Point2f getTraslation();
    float getOrientation();

    void printfHomographyMat();
};

#endif // HOMOGRAPHYMATRIX_H
