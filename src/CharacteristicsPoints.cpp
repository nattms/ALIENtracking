#include "CharacteristicsPoints.h"


void CharacteristicsPoints::matchesClass(CharacteristicsPoints A, cv::BFMatcher matcher, CharacteristicsPoints B, double lambda){

    //Comparamos descriptores
    std::vector<std::vector <cv::DMatch> > contextMatches;

    //Se obtienen los puntos más parecidos de A y B, siendo el resultado un vector de tamaño el número de puntos y dentro la distancia entre ellos.
    matcher.knnMatch(A.descriptorsSIFT,B.descriptorsSIFT,contextMatches,2);

    for (unsigned i = 0; i < A.KPoints.size(); i++){

        std::vector<cv::DMatch> classDescriptor = contextMatches[i];

        double relation = classDescriptor.at(0).distance /classDescriptor.at(1).distance;

        //si la relación es menor (una distancia pequeña) que el umbral lambda, pertenecen a la clase
        if (relation < lambda){
            KPoints.push_back(A.KPoints[i]);
            descriptorsSIFT.push_back(A.descriptorsSIFT.row(i));
        }

    }


}


void CharacteristicsPoints::DeleteRepeatedPoints(CharacteristicsPoints A,CharacteristicsPoints B){

    for (unsigned i = 0; i < A.KPoints.size(); i++){
        bool igual = false;

        for (unsigned j = 0; j < B.KPoints.size(); j++){
            float X = A.KPoints[i].pt.x;
            float Y = A.KPoints[i].pt.y;
            if(X == B.KPoints[j].pt.x && Y == B.KPoints[j].pt.y){
                igual = true;
                break;
            }
        }
        if(!igual){
            descriptorsSIFT.push_back(A.descriptorsSIFT.row(i));
            KPoints.push_back(A.KPoints[i]);
        }
    }

}

void CharacteristicsPoints::DeleteRepeatedPoints(){

    CharacteristicsPoints B;

    for (unsigned i = 0; i < KPoints.size(); i++){
        bool igual = false;

        for (unsigned j = i+1; j < KPoints.size(); j++){
            float X = KPoints[i].pt.x;
            float Y = KPoints[i].pt.y;
            if(X == KPoints[j].pt.x && Y == KPoints[j].pt.y){
                igual = true;
                break;
            }
        }
        if(!igual){
            B.descriptorsSIFT.push_back(descriptorsSIFT.row(i));
            B.KPoints.push_back(KPoints[i]);
        }
    }

    KPoints = B.KPoints;
    descriptorsSIFT = B.descriptorsSIFT;

}




void CharacteristicsPoints::SortingSimilarity(CharacteristicsPoints A, cv::BFMatcher matcher, CharacteristicsPoints B){

    std::vector<std::vector <cv::DMatch> > newMatches;
    //Buscamos el match del objeto ya encontrado (frame anterior) y ft
    matcher.knnMatch(A.descriptorsSIFT,B.descriptorsSIFT,newMatches,1);

    // Creamos otro objeto Objeto (del frame anterior) pero ordenado para utilizar RANSAC
    for (unsigned i = 0; i < newMatches.size(); i++){

        std::vector <cv::DMatch> Mtch = newMatches.at(i);
        int index = Mtch.at(0).trainIdx; //con cual se le empareja en la distancia menor

        //Ordenamos segun el emparejamiento de la distancia menor
        descriptorsSIFT.push_back(B.descriptorsSIFT.row(index));
        KPoints.push_back(B.KPoints[index]);
    }

}

void CharacteristicsPoints::PointsInRotatedRect(cv::RotatedRect rB)
{
    CharacteristicsPoints tmp;

    for (unsigned i =0; i < KPoints.size(); i++){

        cv::Point2f pt = KPoints[i].pt;

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

            tmp.KPoints.push_back(KPoints[i]);
            tmp.descriptorsSIFT.push_back(descriptorsSIFT.row(i));

        }


    }

    KPoints.clear();
    descriptorsSIFT.release();

    tmp.descriptorsSIFT.copyTo(descriptorsSIFT);
    KPoints = tmp.KPoints;
}

void CharacteristicsPoints::PointsInRotatedRect(cv::RotatedRect rB, CharacteristicsPoints Apoint,cv::Mat image)
{

    cv::Mat Mask = cv::Mat::zeros(image.rows,image.cols,CV_64F);

    double x0 = double(rB.center.x-rB.size.width*0.5);
    double y0 = double(rB.center.y-rB.size.height*0.5);

    cv::Rect ROIr = cv::Rect(x0,y0,rB.size.width,rB.size.height);

    Mask(ROIr) = 1;

    cv::Point2f src_center(Mask.cols/2.0F, Mask.rows/2.0F);
    cv::Mat rot_mat = getRotationMatrix2D(src_center, rB.angle, 1.0);

    warpAffine(Mask, Mask, rot_mat, image.size());

//    cv::namedWindow("mascara");
//    cv::imshow("mascara",Mask);
//    cv::waitKey(0);
//    cv::destroyAllWindows();


    for (unsigned i =0; i < Apoint.KPoints.size(); i++){

        cv::Point2f pt = Apoint.KPoints[i].pt;

        if(Mask.at<double>(pt.y,pt.x) != 0){

            KPoints.push_back(Apoint.KPoints[i]);
            descriptorsSIFT.push_back(Apoint.descriptorsSIFT.row(i));

        }


    }


}

void CharacteristicsPoints::TransformateChrPoints(cv::Mat InvertTransform, CharacteristicsPoints A)
{

    KPoints = A.KPoints;
    descriptorsSIFT = A.descriptorsSIFT.clone();

    for (unsigned i =0; i < A.KPoints.size(); i++){

        cv::Point2f pt = A.KPoints[i].pt;
        cv::Mat Point= cv::Mat(1,2,InvertTransform.type());

        Point.at<float>(0,0) = pt.x;
        Point.at<float>(0,1) = pt.y;


        cv::Mat nPt =  Point * InvertTransform;

        KPoints[i].pt.x = nPt.at<float>(0,0);;
        KPoints[i].pt.y = nPt.at<float>(0,1);;



    }

}

void CharacteristicsPoints::MergeChrPoints(CharacteristicsPoints A)
{

    descriptorsSIFT.push_back(A.descriptorsSIFT);
    KPoints.insert( KPoints.end(), A.KPoints.begin(), A.KPoints.end() );

}

void CharacteristicsPoints::RandomLimitedSelection(int N)
{
    int size = KPoints.size();
    CharacteristicsPoints tmp;

    for (int i = 0; i < N; i++){
        tmp.KPoints.push_back(KPoints[rand() % size]);
        tmp.descriptorsSIFT.push_back(descriptorsSIFT.row(rand() % size));

    }

    KPoints.clear();
    descriptorsSIFT.release();

    tmp.descriptorsSIFT.copyTo(descriptorsSIFT);
    KPoints = tmp.KPoints;

}



std::vector<cv::Point2f> CharacteristicsPoints::ConvertTo2f()
{
    std::vector< cv::Point2f > Apoints;
    cv::KeyPoint::convert(KPoints,Apoints);
    return Apoints;
}

