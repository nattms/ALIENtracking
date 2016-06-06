#include "Faces.h"

std::vector<cv::Rect> Faces::detectAndDisplayFace(cv::Mat frame, cv::CascadeClassifier face_cascade,
        cv::String face_cascade_name) {

	if (!face_cascade.load(face_cascade_name))
		std::cout << "No se lee" << std::endl;

    //cv::equalizeHist();
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;

	if (frame.channels() > 1){
		cvtColor(frame, frame_gray, CV_BGR2GRAY);
	}
	else{
		frame_gray = frame.clone();
	}
    
    cv::equalizeHist(frame_gray, frame_gray);


	
	
//-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.05, 3, 0 | CV_HAAR_SCALE_IMAGE);

    for (size_t i = 0; i < faces.size(); i++) {
        cv::Point pointf(faces[i].x + faces[i].width,
                faces[i].y + faces[i].height);
        cv::Point pointi(faces[i].x, faces[i].y);

        cv::rectangle(frame, pointi, pointf, cv::Scalar(0, 0, 255), 2);

        cv::Mat faceROI = frame_gray(faces[i]);
    }
//-- Show what you got
    //cv::imshow("Inicial cara", frame);

	cv::imshow("P", frame_gray);
	cv::waitKey(0);

    return faces;
}



void dimeHola () {
    std::cout << "Hola" << std::endl;
}
