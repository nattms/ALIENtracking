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
	
	//Leemos carpetas de pruebas
	cv::VideoCapture cap("C:/Users/nmira/Documents/Personal/Pruebas tracking/FaceOcc1/img/%04d.jpg"); // would work with: "/my/folder/0013.jpg", 
	
	cv::Mat initial;
	
	cap >> initial;
	
	cv::Rect Face0 = cv::Rect(118,69,114,162);

	

	//St = regionOfSearch
	cv::Rect regionOfSearch = cv::Rect(Face0.x - Face0.width*0.25, Face0.y - Face0.height*0.25, Face0.width * 1.5, Face0.height * 1.5);
	//    cv::Mat Ct = initial(RegionOfSearch);

	cv::RotatedRect trBox(cv::Point2f(Face0.x + Face0.width / 2, Face0.y + Face0.height / 2), Face0.size(), 0);

	//Creamos los objetos de los puntos con sus características
	CharacteristicsPoints Tt, Ct, St, ft, Et, Ot, Dt, Ett;

	//Sacamos los puntos de interés SIFT junto con sus descriptores
	cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(); //(se crea el objeto sift)

	//Mascaras
	cv::Mat maskTt = cv::Mat::zeros(initial.rows, initial.cols, CV_8U);
	maskTt(Face0) = 1;

	// Solo queremos los puntos del Cto, no los de la máscara
	cv::Mat maskCt = cv::Mat::zeros(initial.rows, initial.cols, CV_8U);
	maskCt(regionOfSearch) = 1;
	maskCt(Face0) = 0;

	//obtenemos los puntos sift con las mascaras de objeto y Cto para cada uno
	sift->detectAndCompute(initial, maskCt, Ct.KPoints, Ct.descriptorsSIFT);
	sift->detectAndCompute(initial, maskTt, Tt.KPoints, Tt.descriptorsSIFT);

	Ct.DeleteRepeatedPoints();
	Tt.DeleteRepeatedPoints();


	ft = Tt;

	//Mostramos las regiones
	cv::namedWindow("Inicial", CV_WINDOW_AUTOSIZE);
	cv::Mat Print = initial.clone();
	cv::drawKeypoints(Print, Ct.KPoints, Print, cv::Scalar(0, 255, 0));
	cv::drawKeypoints(Print, Tt.KPoints, Print, cv::Scalar(0, 0, 255));
	cv::rectangle(Print, Face0, cv::Scalar(0, 0, 255));
	cv::rectangle(Print, regionOfSearch, cv::Scalar(0, 255, 0));

	cv::imshow("Inicial", Print);
	cv::waitKey(0);


	float scale = 1;
	float rotation = 0;
	// Le pasamos el centro del tracker
	Tracker tracker(cv::Point2f((Face0.x + Face0.br().x) / 2, (Face0.y + Face0.br().y) / 2), scale, rotation);

	//Motion M0(tracker.position,tracker.rotation);

	//cv::Mat M0 = createM(Face0.x,Face0.y,0);

	//std::cout << "M0:" <<std::endl << M0 << std::endl;

	//puesto del paper
	double lambdaT = 1 / 2.5;
	double lambdaC = 1 / 2.5;

	float ks = 0.5;
	float krho = 90;
	int NO = 5;
	int NT = 1000;
	int ND = 1000;


	cv::Size2f sizeTrack = Face0.size();

	//Recorremos el video
	while (cap.isOpened())
	{

		cv::Mat frame;
		cap >> frame;

		//Imshow
		Print = frame.clone();
		tracker.ShowTracker(sizeTrack, Print, ft.KPoints);



		cv::Mat mask = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
		mask(regionOfSearch) = 1; //Sacamos la región donde puede estar el objeto
		sift->detectAndCompute(frame, mask, St.KPoints, St.descriptorsSIFT);
		St.DeleteRepeatedPoints();
		// Características de la region de búsqueda completado

		//Comparamos los puntos sacados para saber a que clase pertenece (Cto u objeto)
		cv::BFMatcher matcher;

		//Sacamos los puntos correspondientes a cada clase
		Tt.matchesClass(St, matcher, Tt, lambdaT);
		Ct.matchesClass(St, matcher, Ct, lambdaC);

		//Nos quedamos en ft con los puntos que no sean coincidentes (Ft = Tt* \ Ct*)
		ft.DeleteRepeatedPoints(Tt, Ct);

		cv::BFMatcher matching;
		CharacteristicsPoints prevTt;

		//Se crea en prevTt el Tto "Objeto" ordenado segun la distancia con ft para poder sacar RANSAC
		prevTt.SortingSimilarity(ft, matching, Tt);

		// Para hacer RANSAC: Necesitamos vectores de puntos
		std::vector< cv::Point2f > ftPoints = ft.ConvertTo2f();
		std::vector< cv::Point2f > TtPoints = prevTt.ConvertTo2f();


		//Para estimateRigidTransform en CV::Mat
		cv::Mat TtM = cv::Mat(TtPoints);
		cv::Mat ftM = cv::Mat(ftPoints);

		//        cv::Mat transform = cv::estimateRigidTransform(TtM,ftM,true);
		//        std::cout <<std::endl << transform << "\n";
		Motion affineMat(TtPoints, ftPoints);

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
			tracker.position = tracker.position + traslationXY;
			tracker.scale = scale;

			sizeTrack = sizeTrack * tracker.scale;

			trBox = tracker.ShowTracker(sizeTrack, frame, ft.KPoints);
			//cv::drawKeypoints(frame,Ct.KPoints,frame,cv::Scalar(0,255,0));

			//Se saca Et que es los ChaPoints dentro del BB encontrados (sean Ct o Tt da igual)
			Et.PointsInRotatedRect(trBox, St, frame);

			//Ot ChPoints dentro de BB encontrados como Ct

			Ot.PointsInRotatedRect(trBox, Ct, frame);

			// si el num de Ot es pequeño, no está ocluido y actualizamos

			if (Ot.KPoints.size() < NO){

				// Et' son los Et transformados por la M' (transformación inversa)

				Ett.TransformateChrPoints(affineMat.motionMat, Et);

				//Tt act. -> se une Tt con Et'

				Tt.MergeChrPoints(Ett);

				//Dt act -> se le añaden los ocluidos

				Dt.MergeChrPoints(Ot);

				//Ct act. -> todos los puntos de St excluyendo los de Et y uniendo los de Dt.

				Ct.DeleteRepeatedPoints(St, Et);

				Ct.MergeChrPoints(Dt);


				//Si se superan el num de puntos, se actualizan random a un numero máximo

				if (Tt.KPoints.size() > NT)
					Tt.RandomLimitedSelection(NT);
				if (Dt.KPoints.size() > ND)
					Dt.RandomLimitedSelection(ND);

			}

		}

		//tracker solo 4 puntos y pintas líneas

		//si utilizas la homografía o lo que sea transformas los puntos --> Crear clase tracker



		/*Print = frame.clone();
		cv::drawKeypoints(Print,Ct.KPoints,Print,cv::Scalar(0,255,0));
		cv::drawKeypoints(Print,ft.KPoints,Print,cv::Scalar(0,0,255));
		cv::rectangle(Print,regionOfSearch,cv::Scalar(0,255,0));

		cv::imshow("Cap 1", Print);*/
		/*if (cv::waitKey(25) >= 0)
			break;*/
		cv::waitKey(0);
	}

	cv::waitKey(0);
	return 0;
}

