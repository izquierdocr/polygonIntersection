#include <iostream>
#include <deque>  //Double ended queues

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/strategies/transform.hpp>

#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

#include <fstream>	//files


#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace boost::geometry;
using namespace boost::geometry::strategy::transform;


typedef model::d2::point_xy<double> boostPointType;
typedef model::polygon<boostPointType> boostPolygonType;   //A polygon with holes defined for polygons

typedef Point cvPointType;
typedef vector < vector< cvPointType > > cvPolygonType;   //A polygon with holes defined for polygons

struct poseType {
  double x;
  double y;
  double theta;
  double flatArea;
};

typedef vector<poseType> poseArrayType;



cvPolygonType boostPolygon2cvPoligon(boostPolygonType boostPolygon) {
  cvPolygonType cvPolygon;
  vector<cvPointType> cvPolygonSingle;
  
  int polygonSize = num_points(boostPolygon.outer());
  vector<boostPointType> const& points = boostPolygon.outer(); //The exterior polygon with no holes
  for (vector<boostPointType>::size_type i = 0; i < points.size(); ++i)
  {
    cvPolygonSingle.push_back( Point ( get<0>(points[i]), get<1>(points[i]) ) );
  }
  cvPolygon.push_back(cvPolygonSingle);
  
  //TODO It need to handle holes. Not clear how to access .inners() polygons
  for (int i=0; i<num_interior_rings (boostPolygon); i++ ) {  //Check all holes
    cvPolygonSingle.clear();
    /*
     *   int holeSize = num_points(boostPolygon.inners());
     *   vector<boostPointType> const& holePoints = boostPolygon.inners(i); //Each hole polygon
     *   for (vector<boostPointType>::size_type i = 0; i < holePoints.size(); ++i)
     *   {
     *     cvPolygonSingle.push_back( Point ( get<0>(holePoints[i]), get<1>(holePoints[i]) ) );
  }
  cvPolygon.push_back(cvPolygonSingle);
  */
  }
  return cvPolygon;
}


void drawPoint(Mat &image, double x, double y, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, Scalar lineColor=(255,255,255), int thickness=1) {
  line( image,Point( (x-minX)/resolution, (y-minY)/resolution ),Point( (x-minX)/resolution, (y-minY)/resolution ), lineColor, thickness );
}

void drawLine(Mat &image, double x1, double y1, double x2, double y2, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, Scalar lineColor=(255,255,255), int thickness=1) {
  line( image,Point( (x1-minX)/resolution, (y1-minY)/resolution ),Point( (x2-minX)/resolution, (y2-minY)/resolution ), lineColor, thickness );
}

void drawPolygon(Mat &image, cvPolygonType cvPolygon, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, Scalar lineColor=(255,255,255), int thickness=1, bool filled=false) {
  for (int i=0; i<cvPolygon.size(); i++ ) {
    for (int j=0; j<cvPolygon[i].size(); j++ ) { //Draw outer and inners (holes) polygons
      cvPolygon[i][j].x = (cvPolygon[i][j].x - minX)/resolution;
      cvPolygon[i][j].y = (cvPolygon[i][j].y - minY)/resolution;
    }
    if (!filled) polylines( image, cvPolygon[i], true, lineColor, thickness );
  }
  if (filled) fillPoly( image, cvPolygon, lineColor );
}

void drawPolygon(Mat &image, boostPolygonType boostPolygon, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, Scalar lineColor=(255,255,255), int thickness=1, bool filled=false) {
  cvPolygonType cvPolygon = boostPolygon2cvPoligon(boostPolygon);
  drawPolygon(image, cvPolygon, minX, minY, maxX, maxY, resolution, lineColor, thickness, filled);
}


void loadMapfromAria(Mat &map, double &minX, double &minY, double &maxX, double &maxY, double &resolution) {
  
  ifstream fileStream;
  fileStream.open ("lab_2015.map", fstream::in);
  if (!fileStream.fail()) {
    string lineSTR;
    int numPoints;
    int numLines;
    
    getline(fileStream,lineSTR);  //File type (Not used)
    fileStream >> lineSTR >> minX >> minY;
    fileStream >> lineSTR >> maxX >> maxY;
    fileStream >> lineSTR >> numPoints;
    getline(fileStream,lineSTR);  //New line after << (Not used)
    getline(fileStream,lineSTR);  //Points sorted (Not used)
    fileStream >> lineSTR >> resolution;
    getline(fileStream,lineSTR);  //New line after << (Not used)
    getline(fileStream,lineSTR);  //Min line (Not used)
    getline(fileStream,lineSTR);  //Max line (Not used)
    fileStream >> lineSTR >> numLines;
    getline(fileStream,lineSTR);  //New line after << (Not used)
    getline(fileStream, lineSTR);
    while ( lineSTR!="DATA"  ) {
      getline(fileStream, lineSTR);
    }

    int xMap = (maxX-minX)/resolution;
    int yMap = (maxY-minY)/resolution;
    map = Mat::zeros(yMap,xMap,CV_8UC1);
    
    int x,y;
    for (int i=0; i<numPoints; i++) {
      fileStream >> x >> y;
      drawPoint(map, x, y, minX, minY, maxX, maxY, resolution, Scalar(255,255,255), 1);
    }
    fileStream.close();
    
    cout << "Min X: " << minX << "  Max X: " << maxX << endl;
    cout << "Min Y: " << minY << "  Max Y: " << maxY << endl;
    cout << "Resolution: " << resolution << endl;
  }
  else {
    cout << "Error: No data file found." << endl;
  }
}

bool freeSpace(Mat map, double x, double y, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1) {
  const int obstaclePoint = 255;
  //Mat uses y,x
  if (map.at<uchar>( int ((y-minY)/resolution), int ((x-minX)/resolution) ) == obstaclePoint ) return false;
  return true;
}

void drawPoses(Mat map, poseArrayType poses, boostPolygonType visibilityCone, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1) {
  Mat mapBackup=map.clone();
  for (int i=0; i<poses.size(); i++) {
    poseType pose;
    double x=poses[i].x;
    double y=poses[i].y;
    double theta=poses[i].theta;
    
    //Draw pose with arrow
    int orientationLength=max( abs(maxX-minX), abs(maxY-minY) )/50;
    drawPoint(mapBackup, x, y, minX, minY, maxX, maxY, resolution, Scalar(100,100,100), 6);
    drawLine(mapBackup, x, y, x+orientationLength*sin(theta*PI/180), y+orientationLength*cos(theta*PI/180), minX, minY, maxX, maxY, resolution, Scalar(100,100,100), 2);
    
    //Draw visibility cone
    translate_transformer<boostPointType, boostPointType> translate(x, y);
    rotate_transformer<boostPointType, boostPointType, degree> rotate(theta);
    boostPolygonType transf, transf2;
    transform(visibilityCone, transf, rotate);
    transform(transf, transf2, translate);
    //scale_transformer<boostPointType, boostPointType>  scale(40.0,40.0);
    cvPolygonType cvPolygon = boostPolygon2cvPoligon(transf2);
    //drawPolygon(mapBackup, cvPolygon, minX, minY, maxX, maxY, resolution, Scalar(50,50,50), 2, true);
  }
  imshow("Poses", mapBackup);
  waitKey(0);
}


void filterSimilarPoses(poseArrayType &poses, double minDistance, double minAngle) {
  //This method supposes poses sorted by flatArea desc
  poseArrayType posesBackup=poses;
  poses.clear();
  for (int i=0; i<posesBackup.size(); i++) {
    bool isClose=false;
    for (int j=0; j<poses.size(); j++) {
      //if ( abs(posesBackup[i].x-poses[j].x) < minDistance && abs(posesBackup[i].y-poses[j].y) < minDistance && abs(posesBackup[i].theta-poses[j].theta && posesBackup[i].x!=poses[j].x) && posesBackup[i].y!=poses[j].y ) {
      if ( abs(posesBackup[i].x-poses[j].x) < minDistance && abs(posesBackup[i].y-poses[j].y) < minDistance && abs(posesBackup[i].theta-poses[j].theta) < minAngle ) {
	isClose=true;
	break;
      }
    }
    if (!isClose) poses.push_back(posesBackup[i]);
  }
  //cout << "Original poses: " << posesBackup.size()-poses.size() << " similar poses" << endl;
  cout << "Deleted " << posesBackup.size()-poses.size() << " similar poses" << endl;
}

bool wayToSortPoses(poseType i, poseType j) {
  return i.flatArea > j.flatArea;
}


void evaluatePoses(poseArrayType &poses, vector <boostPolygonType> flatSurfaces, boostPolygonType visibilityCone, int maxPoses) {
  poseArrayType posesBackup=poses;
  poses.clear();
  int numValidPoses=0;
  for (int i=0; i<posesBackup.size(); i++) {
    translate_transformer<boostPointType, boostPointType> translate(posesBackup[i].x, posesBackup[i].y);
    rotate_transformer<boostPointType, boostPointType, degree> rotate(posesBackup[i].theta);
    boostPolygonType transf, transf2;
    transform(visibilityCone, transf, rotate);
    transform(transf, transf2, translate);
    double totalIntersectionArea=0;
    for (int j=0; j<flatSurfaces.size(); j++) {
      deque<boostPolygonType> flatVisibilityIntersection;  //The intersection could be more than one polygon
      intersection(transf2, flatSurfaces[j], flatVisibilityIntersection);
      BOOST_FOREACH(boostPolygonType const& polygon, flatVisibilityIntersection) { //Each polygon resulting for intersection
	totalIntersectionArea += area(polygon);
      }
    }
    if (totalIntersectionArea>0) {
      posesBackup[i].flatArea = totalIntersectionArea;
      poses.push_back(posesBackup[i]);
      numValidPoses++;
    }
  }
  
  posesBackup.clear();
  cout << "Flat view poses: " << numValidPoses << endl;
  sort(poses.begin(), poses.end(), wayToSortPoses);
  for (int i=0; i<poses.size(); i++) {
    cout << "Pose X: " << poses[i].x << "  Y: " << poses[i].y << "  Area:" << poses[i].flatArea << endl;
    if (i<maxPoses) {   //Only get the first ones maxPoses
      posesBackup.push_back(poses[i]);
    }
  }
  poses=posesBackup;
}
  

void generatePoses(Mat map, poseArrayType &poses, int maxPoses, boostPolygonType visibilityCone, boostPolygonType space, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1) {
  for (int i=0; i<maxPoses;) {
    poseType pose;
    pose.x=(maxX-minX)*( double (rand())/RAND_MAX) + minX;
    pose.y=(maxY-minY)*( double (rand())/RAND_MAX) + minY;
    pose.theta=360*( double (rand())/RAND_MAX);
    
    if ( within(make<boostPointType>(pose.x, pose.y), space) && freeSpace(map, pose.x, pose.y, minX, minY, maxX, maxY, resolution) ) {
      poses.push_back(pose);
      i++;
    }
    else {
      //drawPoint(mapBackup, pose.x, pose.y, minX, minY, maxX, maxY, resolution, Scalar(50,50,50), 8);
      cout << "Pose X: " << pose.x << "  Y: " << pose.y << "  no valid" << endl;
    }
  }
}



void drawFlatSurfaces(Mat &map, vector <boostPolygonType> flatSurfaces, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1) {
  for (int i=0; i<flatSurfaces.size(); i++ ) {
    cvPolygonType cvPolygon = boostPolygon2cvPoligon(flatSurfaces[i]);
    drawPolygon(map, cvPolygon, minX, minY, maxX, maxY, resolution, Scalar(255,255,255), 2, true);
  }
  //imshow("Map", map);
  //waitKey(0);
}

vector <boostPolygonType> selectFlatSurfaces(boostPolygonType space, vector <boostPolygonType> flatSurfaces) {
  vector <boostPolygonType> flatSurfacesinSpace;
  for (int i=0; i<flatSurfaces.size(); i++) {
    bool inSpace=false;
    deque<boostPolygonType> flatinSpaceIntersection;  //The intersection could be more than one polygon
    intersection(space, flatSurfaces[i], flatinSpaceIntersection);
    BOOST_FOREACH(boostPolygonType const& polygon, flatinSpaceIntersection) { //Each polygon resulting for intersection
      if ( area(polygon)>0 ) inSpace=true;
    }
    if (inSpace) {
      flatSurfacesinSpace.push_back(flatSurfaces[i]);
    }
  }
  cout << "Selected " << flatSurfacesinSpace.size() << " flat surfaces in the space" << endl;
  return flatSurfacesinSpace;
}


void loadPolygons(vector <boostPolygonType> &polygons, string fileName) {
  
  ifstream fileStream;
  fileStream.open (fileName.c_str(), fstream::in);
  if (!fileStream.fail()) {
    string polygonSTR;
    boostPolygonType polygon;
    while ( getline(fileStream, polygonSTR) ) {
      read_wkt(polygonSTR, polygon);
      polygons.push_back(polygon);
    }
    fileStream.close();
  }
  else {
    cout << "Error: No data file found." << endl;
  }
}



int main()
{
  double minX,minY;
  double maxX,maxY;
  double resolution;

  Mat map;
  loadMapfromAria(map, minX, minY, maxX, maxY, resolution);
  
  Mat mapFree;
  Mat kernel = Mat::ones(12,12,CV_8UC1);
  dilate(map, mapFree, kernel, Point(-1,-1), 1);
  
  vector <boostPolygonType> polygonsFile;  //Load polygon reads many polygons from a file but we only have one for cone
  loadPolygons(polygonsFile, "visibilityCone.txt");
  boostPolygonType visibilityCone = polygonsFile[0];
  
  vector <boostPolygonType> flatSurfaces;
  loadPolygons(flatSurfaces, "flatSurfaces.txt");
  drawFlatSurfaces(mapFree, flatSurfaces, minX, minY, maxX, maxY, resolution);
  
  vector <boostPolygonType> room;
  //loadPolygons(dinningroom, "dinningroom.txt");
  loadPolygons(room, "livingroom.txt");
  
  vector <boostPolygonType> flatSurfacesinDinning;
  flatSurfacesinDinning = selectFlatSurfaces(room[0], flatSurfaces);
  
  poseArrayType poses;
  int numPoses=1000;
  generatePoses(mapFree, poses, numPoses, visibilityCone, room[0], minX, minY, maxX, maxY, resolution);
  int maxPoses=20;
  evaluatePoses(poses, flatSurfacesinDinning, visibilityCone, maxPoses);
  double minDistance=1000;  //Distance to be considered close in milimeters
  double minAngle=30;   //Angle to be considered close in degrees
  filterSimilarPoses(poses, minDistance, minAngle);
  drawPoses(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution);
  
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