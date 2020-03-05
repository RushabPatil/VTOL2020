#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;


const float calibrationSquareDimension = ; //meters
const float arucoSquareDimension =  ; //meters
const Size chessboardDimensions = Size(6, 9);

//Create Aruco Markers for 4X4_50 Aruco Markers
void createArucoMarkers(){

  Mat outputMarker;

 Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

 for (int i; i < 50; i++){
    aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
    ostringstream  convert;
    string imageName = "4x4Marker";
    convert << imageName << i << ".jpg";
    imwrite(convert.str(), outputMarker);
  }

}


//
void creatKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
  for (int i = 0; i < boardSize.height; i++) {

    for (int j = 0; j < boardSize.width; j++) {

      corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));

    }
  }
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults){

  for(vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++){
    vector<Point2f> pointBuf;
    bool found = findChessboardCorners(*iter, Size(9,6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE);

    if(found){
      allFoundCorners.push_back(pointBuf);
    }

    if(showResults)
    {
      drawChessboardCorners(*iter, Size(9,6), pointBuf, found);
      imshow("Looking for corners", *iter);
      waitKey(0);
    }
  }
}


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int startCameraMonitoring(const Mat& cameraMatrix. const Mat& distanceCoefficients, float arucoSquareDimensions)
{
  Mat frame;

 vector<int> markerIds;
 vector<vector<Point2f> markerCorners, rejectedCandidates;

 aruco::DetectorParameters parameters;

 Ptr < aruco:: Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruoco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50)

 //Initialize camera
 int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
       capture_height,
       display_width,
       display_height,
       framerate,
       flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  if (!vid.isOpened())
    {
      return -1;
    }


    namedWindow("CSI Camera", CV_WINDOW_AUTOSIZE);

    vector<Vec3d> rotationVectors, translationVectors;

    while(true){

      if (!vid.read(frame))
        break;

        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds)
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors)

        for(int i = 0; i < markerIds.size(); i++){
// draw axes
        aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,rotationVectors, translationVectors);
    }

    imshow("CSI Camera", frame);
    if(waitKey(30) >= 0) break;

  }

  return 1;


}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients){

  vector<vector<Point2f>> checkerboardImageSpacePoints;
  getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

  vector<vector<Point3f>> worldSpaceCornerPoints(1);

  createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0])

  vector<Mat> rVectors, tVectors;

  distanceCoefficients = Mat::zeros(8, 1, CV_64f);

  calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, rVectors, tVectors);
}


bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients){

  ofstream outStream(name);
  if (outStream){

    uint16_t rows = cameraMatrix.rows;
    uint16_t columns = cameraMatrix.cols;

    outStream << rows << endl;
    outStream << columns << endl;
    for (int r = 0; r < rows; r++){

      for (int c = 0; c < columns; c++){

        double value = cameraMatrix.at<double>(r,c;
          outstream << value << endl;
      }
    }

    rows = distanceCoefficients.rows;
    columns = distanceCoefficients.cols;

    outStream << rows << endl;
    outStream << columns << endl;

    for (int r = 0; r < rows; r++){

      for (int c = 0; c < columns; c++){

        double value = distanceCoefficients.at<double>(r,c;
          outstream << value << endl;
      }
    }

    outStream.close();

    return true;
  }

    return false;
}

bool loadCameraCalibration( string name, Mat& cameraMatrix, Mat& distanceCoefficients){

  ifstream inStream(name);
  if( inStream){

    uint16_t rows;
    uint16_t columns;

    inStream >> rows;
    inStream >> columns;

    cameraMatrix = Mat(Size(columns, rows), CV_64F);

    for(int r = 0; r < rows; r++){

      for ( int c = 0; c < columns, c++){

        double read = 0.0f;
        inStream >> read;
        cameraMatrix.at<double>(r,c) = read;
        cout<< cameraMatrix.at<double>(r,c) << "\n";
      }
    }

    //Distance Coefficients
     inStream >> rows;
     inStream >> columns;

     distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

     for(int r = 0; r < rows; r++){

       for ( int c = 0; c < columns, c++){

         double read = 0.0f;
         inStream >> read;
         distanceCoefficients.at<double>(r,c) = read;
         cout<< distanceCoefficients.at<double>(r,c) << "\n";
       }
     }
     inStream.close();
     return true;

  }

  return false;



}



int main(int argc, char const *argv[]) {

 Mat frame;
 Mat drawToFrame;

 Mat cameraMatrix = Mat :: eye(3,3, CV_64F);

 Mat distanceCoefficients;

 vector<Mat> savedImages;

 vector<vector<Point2f>> markerCorners, rejectedCorners;

 //Initialize camera
 int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
	     capture_height,
	     display_width,
	     display_height,
	     framerate,
	     flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
   if (!vid.isOpened())
 {
   return;
   }

   int framesPerSecond = 20;

   namedWindow("Camera", CV_WINDOW_AUTOSIZE);


   while(true)
   {
     if(!vid.read(frame))
     break;

     vector<Vec2f> foundPoints;
     bool found = false;

     found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

     frame.copyTo(drawToFrame);
     drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
     if(found)
     imshow("Camera", drawToFrame);
     else
     imshow("Camera", frame);

     char character = waitKey(1000/framesPerSecond);

     switch (character) {
       case '':
       if(found){
         Mat temp;
         frame.copyTo(temp);
         savedImages.push_back(temp);
       }
       //saving images
       break;
       case 13:
       //start calibration
       if(savedImages.size() > 15)
       cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients)
       saveCameraCalibration("IlovecameraCalibration", cameraMatrix, distanceCoefficients)
       break;

       case 27:
       //exit
       return 0;

       break;
       }
   }

  return 0;
}
