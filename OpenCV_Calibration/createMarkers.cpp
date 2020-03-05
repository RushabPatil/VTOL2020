#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;


//Create Aruco Markers for 4X4_50 Aruco Markers
void createArucoMarkers(){

  Mat outputMarker;

 Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

 for (int i; i < 50; i++){
    aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
    ostringstream  convert;
    string imageName = "4x4Marker";
    convert << imageName << i << ".jpg";
    imwrite(convert.str(), outputMarker);
  }

}
 int main(int argv, char** argc)
{

	createArucoMarkers();
 
