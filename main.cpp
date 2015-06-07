#include <iostream>

#include "opencv2/opencv.hpp"

#include "internalExploration.cpp"


using namespace std;
using namespace cv;


int main()
{
  double minX,minY;
  double maxX,maxY;
  double resolution;

  Mat map;
  loadMapfromAria(map, minX, minY, maxX, maxY, resolution, "lab_2015.map");
  
  Mat mapFree;
  Mat kernel = Mat::ones(14,14,CV_8UC1);
  dilate(map, mapFree, kernel, Point(-1,-1), 1);
  
  vector <boostPolygonType> polygonsFile;  //Load polygon reads many polygons from a file but we only have one for cone
  loadPolygons(polygonsFile, "visibilityCone.txt");
  boostPolygonType visibilityCone = polygonsFile[0];
  
  vector <boostPolygonType> flatSurfaces;
  loadPolygons(flatSurfaces, "flatSurfaces.txt");
  drawFlatSurfaces(mapFree, flatSurfaces, minX, minY, maxX, maxY, resolution);
  
  vector <boostPolygonType> room;
  //loadPolygons(room, "diningroom.txt");
  loadPolygons(room, "livingroom.txt");
  //loadPolygons(room, "kitchen.txt");
  //loadPolygons(room, "bathroom.txt");
  //loadPolygons(room, "bedroom.txt");
  //loadPolygons(room, "studio.txt");
  
  vector <boostPolygonType> flatSurfacesinRoom;
  flatSurfacesinRoom = selectFlatSurfaces(room[0], flatSurfaces);
  
  poseArrayType poses;
  
  int numPoses=2000;
  generatePoses(mapFree, poses, numPoses, visibilityCone, room[0], minX, minY, maxX, maxY, resolution);
  
  //int maxPoses=100;
  int maxPoses=numPoses*0.10;
  evaluatePoses(poses, flatSurfacesinRoom, visibilityCone, maxPoses);
  
  double obstacleDistance=300;  //Distance to be considered close to obstacles in milimeters
  filterCloseToObstacles(mapFree, poses, obstacleDistance, minX, minY, maxX, maxY, resolution);
  
  double closeDistance=1000;  //Distance to be considered close to other poses in milimeters
  double closeAngle=30;   //Angle to be considered close to other poses in degrees
  filterSimilarPoses(poses, closeDistance, closeAngle);
  
  cout << poses.size() << " poses to be used" << endl;
  drawPoses(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution);
  
  double xInit=5950;
  double yInit=-98;
  selectPath(poses, xInit, yInit);
  drawPath(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution);
  
  return 0;
}



/*

boostPolygonType green, blue, red;

vector <boostPolygonType> polygons;
loadPolygons(polygons, "flatSurfaces.txt");

read_wkt(
  "POLYGON((2 1.3,2.4 1.7,2.8 1.8,3.4 1.2,3.7 1.6,3.4 2,4.1 3,5.3 2.6,5.4 1.2,4.9 0.8,2.9 0.7,2 1.3)"
  "(4.0 2.0, 4.2 1.4, 4.8 1.9, 4.4 2.2, 4.0 2.0))", green);

read_wkt(
  "POLYGON((4.0 -0.5 , 3.5 1.0 , 2.0 1.5 , 3.5 2.0 , 4.0 3.5 , 4.5 2.0 , 6.0 1.5 , 4.5 1.0 , 4.0 -0.5))", blue);

  read_wkt(
   "POLYGON((10.0 10.0 , 10.0 40.0 , 40.0 40.0 , 40.0 10.0))", red);
 
red = polygons[2];

std::deque<boostPolygonType> output;  //The intersection could be more than one polygon
intersection(green, blue, output);

int i = 0;
cout << "green && blue:" << endl;
BOOST_FOREACH(boostPolygonType const& p, output) //Each polygon resulting for intersection
{
  cout << i++ << ": " << area(p) << endl;
}


Mat image = Mat::zeros(300,300,CV_8UC1);


translate_transformer<boostPointType, boostPointType> translate(50.0, 50.0);
rotate_transformer<boostPointType, boostPointType, degree> rotate(45.0);
scale_transformer<boostPointType, boostPointType>  scale(40.0,40.0);

boostPolygonType transf, transf2;
transform(red, transf, rotate);
transform(green, transf2, scale);
//transform(red, transf, translate);
//transform(red, transf, rotate);


cvPolygonType cvPolygon = boostPolygon2cvPoligon(transf);
polylines( image, cvPolygon, true,  Scalar( 255, 255, 255 ) );

cvPolygon = boostPolygon2cvPoligon(transf2);
polylines( image, cvPolygon, true,  Scalar( 255, 255, 255 ) );


cout << "Imagen " << endl;
imshow("Output", image);

Mat kernel = Mat::ones(8,8,CV_8UC1);
dilate(image, image, kernel, Point(-1,-1), 1);
imshow("Dilated", image);

waitKey(0);
*/