#include <opencv2\opencv.hpp>
#include <opencv2\videoio.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\core.hpp>
#include <opencv2\xfeatures2d.hpp>
#include "opencv2\videoio\videoio.hpp"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "Faces.h"
#include "CharacteristicsPoints.h"
#include "Motion.h"
#include "tracker.h"
//#include "homographyMatrix.h"


#define PI 3.14159265358979323846



int main(/*int argc, char** argv*/) {


	cv::VideoCapture cap("C:/Users/nmira/Documents/Personal/montreal/cap1.avi");

    double fps = cap.get(CV_CAP_PROP_FPS);
	
    //double nFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    double nFrame = 25;
    cap.set(CV_CAP_PROP_POS_FRAMES, nFrame);

    cv::Mat initial;
    cap >> initial;
	std::cout << initial << std::endl;
	cv::namedWindow("P", CV_WINDOW_AUTOSIZE);
	cv::imshow("P", initial);
	cv::waitKey(0);
    // Detectamos cara
    cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
    cv::CascadeClassifier face_cascade;
    Faces facerect;
    std::vector<cv::Rect> FacesV = facerect.detectAndDisplayFace(initial, face_cascade, face_cascade_name);


    if(FacesV.empty()){
        int i = 0;
        do{ //en el caso de que no haya encontrado ninguna cara
            cap.set(CV_CAP_PROP_POS_FRAMES, nFrame+25*i);
            cap >> initial;
            FacesV = facerect.detectAndDisplayFace(initial, face_cascade, face_cascade_name);
            i++;
        }while(FacesV.empty());
    }


    //Cogemos la primera cara y creamos las zonas de ROI y Área de búsqueda
    cv::Rect Face0 = FacesV.at(0);

    //St = regionOfSearch
    cv::Rect regionOfSearch = cv::Rect(Face0.x-Face0.width*0.5,Face0.y-Face0.height*0.5,Face0.width*2,Face0.height*2);
//    cv::Mat Ct = initial(RegionOfSearch);

    cv::RotatedRect trBox(cv::Point2f(Face0.x + Face0.width/2,Face0.y + Face0.height/2), Face0.size(), 0);

    //Creamos los objetos de los puntos con sus características
    CharacteristicsPoints Object, Context, SearchArea, ft, Et, Ot, Dt, Ett;

    //Sacamos los puntos de interés SIFT junto con sus descriptores
    cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(); //(se crea el objeto sift)

    //Mascaras
    cv::Mat maskObject = cv::Mat::zeros(initial.rows,initial.cols,CV_8U);
    maskObject(Face0) = 1;

    // Solo queremos los puntos del contexto, no los de la máscara
    cv::Mat maskContext = cv::Mat::zeros(initial.rows,initial.cols,CV_8U);
    maskContext(regionOfSearch) = 1;
    maskContext(Face0) = 0;

    //obtenemos los puntos sift con las mascaras de objeto y contexto para cada uno
    sift->detectAndCompute(initial,maskContext,Context.KPoints,Context.descriptorsSIFT);
    sift->detectAndCompute(initial,maskObject,Object.KPoints,Object.descriptorsSIFT);

    Context.DeleteRepeatedPoints();
    Object.DeleteRepeatedPoints();


    ft = Object;

    //Mostramos las regiones
    cv::namedWindow("Inicial",CV_WINDOW_AUTOSIZE);
    cv::Mat Print = initial.clone();
    cv::drawKeypoints(Print,Context.KPoints,Print,cv::Scalar(0,255,0));
    cv::drawKeypoints(Print,Object.KPoints,Print,cv::Scalar(0,0,255));
    cv::rectangle(Print,Face0,cv::Scalar(0,0,255));
    cv::rectangle(Print,regionOfSearch,cv::Scalar(0,255,0));

    cv::imshow("Inicial",Print);
    cv::waitKey(0);


    float scale = 1;
    float rotation = 0;
    // Le pasamos el centro del tracker
    Tracker tracker(cv::Point2f((Face0.x+Face0.br().x)/2,(Face0.y+Face0.br().y)/2),scale,rotation);

    //Motion M0(tracker.position,tracker.rotation);

    //cv::Mat M0 = createM(Face0.x,Face0.y,0);

    //std::cout << "M0:" <<std::endl << M0 << std::endl;

    //puesto del paper
    double lambdaT = 1/2.5;
    double lambdaC = 1/2.5;

    float ks = 0.5;
    float krho = 90;
    int NO = 5;
    int NT = 1000;
    int ND = 1000;


    cv::Size2f sizeTrack = Face0.size();

    //Recorremos el video
    for (;;) {

        cv::Mat frame;
        cap >> frame;

        //Imshow
        Print = frame.clone();
        tracker.ShowTracker(sizeTrack,Print,ft.KPoints);



        cv::Mat mask = cv::Mat::zeros(frame.rows,frame.cols,CV_8U);
        mask(regionOfSearch) = 1; //Sacamos la región donde puede estar el objeto
        sift->detectAndCompute(frame,mask,SearchArea.KPoints,SearchArea.descriptorsSIFT);
        SearchArea.DeleteRepeatedPoints();
        // Características de la region de búsqueda completado

        //Comparamos los puntos sacados para saber a que clase pertenece (contexto u objeto)
        cv::BFMatcher matcher;
        CharacteristicsPoints tn,cn;
        //Sacamos los puntos correspondientes a cada clase
        tn.matchesClass(SearchArea, matcher , Object, lambdaT);
        cn.matchesClass(SearchArea, matcher , Context, lambdaC);

        //Nos quedamos en ft con los puntos que no sean coincidentes (Ft = Tt* \ Ct*)
        ft.DeleteRepeatedPoints(tn,cn);

        cv::BFMatcher matching;
        CharacteristicsPoints prevObject;

        //Se crea en prevObject el objecto "Objeto" ordenado segun la distancia con ft para poder sacar RANSAC
        prevObject.SortingSimilarity(ft,matching,Object);

        // Para hacer RANSAC: Necesitamos vectores de puntos
        std::vector< cv::Point2f > ftPoints = ft.ConvertTo2f();
        std::vector< cv::Point2f > objectPoints = prevObject.ConvertTo2f();


        //Para estimateRigidTransform en CV::Mat
        cv::Mat objectM = cv::Mat(objectPoints);
        cv::Mat ftM = cv::Mat(ftPoints);

//        cv::Mat transform = cv::estimateRigidTransform(objectM,ftM,true);
//        std::cout <<std::endl << transform << "\n";
        Motion affineMat(objectPoints,ftPoints);

        std::cout << affineMat.motionMat;

        //Sacar Scale, rotation and displacement de la matriz; una vez obtenido esto, se transforma en tracker (rectangulo)

        float scale = affineMat.getScale();
        float rotation = affineMat.getOrientation();
        cv::Point2f traslationXY = affineMat.getTraslation();



        // comprobamos si el objeto se ha detectado if(|st - st-1| < ks && |rhot - rhot-1| < krho)

        float difScale = fabs(tracker.scale - scale);
        float difRot = fabs(tracker.rotation - rotation) * 180 / PI;


        if (difScale < ks && difRot < krho){

            // Actualizamos el tracker
            tracker.rotation = rotation; //radians
            tracker.position =  tracker.position + traslationXY;
            tracker.scale = scale;

            sizeTrack = sizeTrack * tracker.scale;

            trBox = tracker.ShowTracker(sizeTrack,frame,ft.KPoints);
            //cv::drawKeypoints(frame,cn.KPoints,frame,cv::Scalar(0,255,0));

            //Se saca Et que es los ChaPoints dentro del BB encontrados (sean Ct o Tt da igual)
            Et.PointsInRotatedRect(trBox,SearchArea,frame);

            //Ot ChPoints dentro de BB encontrados como Ct

            Ot.PointsInRotatedRect(trBox,cn,frame);

            // si el num de Ot es pequeño, no está ocluido y actualizamos

            if (Ot.KPoints.size() < NO){

                // Et' son los Et transformados por la M' (transformación inversa)

                Ett.TransformateChrPoints(affineMat.motionMat,Et);

                //Tt act. -> se une Tt con Et'

                tn.MergeChrPoints(Ett);

                //Dt act -> se le añaden los ocluidos

                Dt.MergeChrPoints(Ot);

                //Ct act. -> todos los puntos de St excluyendo los de Et y uniendo los de Dt.

                cn.DeleteRepeatedPoints(SearchArea,Et);

                cn.MergeChrPoints(Dt);


                //Si se superan el num de puntos, se actualizan random a un numero máximo

                if(tn.KPoints.size() > NT)
                    tn.RandomLimitedSelection(NT);
                if(Dt.KPoints.size() > ND)
                    Dt.RandomLimitedSelection(ND);

            }

        }

        //tracker solo 4 puntos y pintas líneas

        //si utilizas la homografía o lo que sea transformas los puntos --> Crear clase tracker



        /*Print = frame.clone();
        cv::drawKeypoints(Print,cn.KPoints,Print,cv::Scalar(0,255,0));
        cv::drawKeypoints(Print,ft.KPoints,Print,cv::Scalar(0,0,255));
        cv::rectangle(Print,regionOfSearch,cv::Scalar(0,255,0));

        cv::imshow("Cap 1", Print);*/
        if (cv::waitKey(fps) >= 0)
            break;
    }

    cv::waitKey(0);
    return 0;
}

