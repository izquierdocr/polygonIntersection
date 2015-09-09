#include <iostream>
#include <deque>  //Double ended queues
#include <algorithm>    // std::copy_if, std::distance
#include <fstream>	//files

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/strategies/transform.hpp>

#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"


#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace boost::geometry;
using namespace boost::geometry::strategy::transform;


typedef model::d2::point_xy<double> boostPointType;
typedef model::polygon<boostPointType> boostPolygonType;   //A polygon with holes defined for polygons in boost
typedef model::multi_polygon<boostPolygonType> boostMultiPolygonType;
//TODO Many methods have not used multi-polygon for not known it before. For easy code, it is needed a change for deque

typedef Point cvPointType;
typedef vector < vector< cvPointType > > cvPolygonType;   //A polygon with holes defined for polygons in OpenCV



struct poseType {
  double x;
  double y;
  double theta;
  double flatArea;
  int flatSurfaceSeen;
  double overlapedArea;
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


void loadMapfromAria(Mat &map, double &minX, double &minY, double &maxX, double &maxY, double &resolution, string fileName) {
  
  ifstream fileStream;
  fileStream.open (fileName.c_str(), fstream::in);
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

bool freeSpace(Mat map, double x1, double y1, double x2, double y2, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1) {
  const int obstaclePoint = 255;
  int x1i=(x1-minX)/resolution;
  int y1i=(y1-minY)/resolution;
  int x2i=(x2-minX)/resolution;
  int y2i=(y2-minY)/resolution;
  if (x1i>x2i) {int tmp=x2i;x2i=x1i;x1i=tmp;}
  if (y1i>y2i) {int tmp=y2i;y2i=y1i;y1i=tmp;}
  for (int x=x1i; x<=x2i; x++)
    for (int y=y1i; y<=y2i; y++)
      //Mat uses y,x
      if (map.at<uchar>( y, x ) == obstaclePoint ) return false;
  return true;
}

void drawPoses(Mat map, poseArrayType poses, boostPolygonType visibilityCone, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, string windowsName="Poses") {
  Mat mapBackup=map.clone();
  for (int i=0; i<poses.size(); i++) {
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
  
  Mat mapFlip;
  flip(mapBackup,mapFlip,0);
  imshow(windowsName, mapFlip);
  waitKey(0);
  destroyWindow(windowsName);
}

void drawOnePose(Mat map, double x, double y, double theta, boostPolygonType visibilityCone, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, string windowsName="Poses") {
  Mat mapBackup=map.clone();
  
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
  drawPolygon(mapBackup, cvPolygon, minX, minY, maxX, maxY, resolution, Scalar(50,50,50), 2, true);
  
  Mat mapFlip;
  flip(mapBackup,mapFlip,0);
  imshow(windowsName, mapFlip);
  waitKey(0);
  destroyWindow(windowsName);
}


void drawPath(Mat map, poseArrayType poses, boostPolygonType visibilityCone, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1, string windowsName="Poses") {
  Mat mapBackup=map.clone();
  
  //Draw the first pose
  int orientationLengthFirst=max( abs(maxX-minX), abs(maxY-minY) )/50;
  drawPoint(mapBackup, poses[0].x, poses[0].y, minX, minY, maxX, maxY, resolution, Scalar(200,200,200), 6);
  drawLine(mapBackup, poses[0].x, poses[0].y, poses[0].x+orientationLengthFirst*sin(poses[0].theta*PI/180), poses[0].y+orientationLengthFirst*cos(poses[0].theta*PI/180), minX, minY, maxX, maxY, resolution, Scalar(200,200,200), 2);
  
  for (int i=1; i<poses.size(); i++) {
    //Draw path
    drawLine(mapBackup, poses[i-1].x, poses[i-1].y, poses[i].x, poses[i].y, minX, minY, maxX, maxY, resolution, Scalar(200,200,200), 1);
    //Draw pose
    int orientationLength=max( abs(maxX-minX), abs(maxY-minY) )/50;
    drawPoint(mapBackup, poses[i].x, poses[i].y, minX, minY, maxX, maxY, resolution, Scalar(200,200,200), 6);
    drawLine(mapBackup, poses[i].x, poses[i].y, poses[i].x+orientationLength*sin(poses[i].theta*PI/180), poses[i].y+orientationLength*cos(poses[i].theta*PI/180), minX, minY, maxX, maxY, resolution, Scalar(200,200,200), 2);
  }
  
  Mat mapFlip;
  flip(mapBackup,mapFlip,0);
  imshow(windowsName, mapFlip);
  waitKey(0);
  destroyWindow(windowsName);
}


double distanceToPoint(double x1, double y1, double x2, double y2) {
  return sqrt( pow(x2-x1,2) + pow(y2-y1,2) );
}


void selectPath(poseArrayType &poses, double xInit, double yInit) {
  //Order poses according the closest pose
  poseArrayType posesBackup=poses;
  poses.clear();
  int numComparatives=posesBackup.size();
  for (int i=0; i<numComparatives; i++) {
  int posMinDistance = 0;
  double minDistance = distanceToPoint(xInit, yInit, posesBackup[posMinDistance].x, posesBackup[posMinDistance].y);
  for (int j=0; j<posesBackup.size(); j++) {
    double d = distanceToPoint(xInit, yInit, posesBackup[j].x, posesBackup[j].y);
    if (d<minDistance) {
      minDistance=d;
      posMinDistance=j;
    }
  }
  poses.push_back(posesBackup[posMinDistance]);
  xInit=posesBackup[posMinDistance].x;
  yInit=posesBackup[posMinDistance].y;
  posesBackup.erase( posesBackup.begin()+posMinDistance );
  }
}


void selectPath2(vector <boostPolygonType> flatSurfacesinRoom, poseArrayType &poses, double xInit, double yInit) { 
  //Order poses according the area of the flat surface
  vector <double> areas;
  vector <int> indexes;
  for (int i=0; i<flatSurfacesinRoom.size(); i++) {
    areas.push_back( area( flatSurfacesinRoom[i] ) );
    indexes.push_back(i);
  }
  sort( indexes.begin(), indexes.end(), [&areas](int i1, int i2) {return areas[i1] > areas[i2];} ); //Using lambdas (C++11) for sorting with indexes
  //And make a path with all the poses of each flat surface seen
  poseArrayType posesBackup=poses;
  poses.clear();
  for (int i=0; i<flatSurfacesinRoom.size(); i++) {
    poseArrayType posesbySurface;
    for (int j=0; j< posesBackup.size(); j++) {
      if (posesBackup[j].flatSurfaceSeen == indexes[i]) {
	posesbySurface.push_back(posesBackup[j]);
      }
    }
    //Intentos por usar librerias de copia condicional
    //poseArrayType posesbySurface ( posesBackup.size() );
    //std::copy_if (posesBackup.begin(), posesBackup.end(), back_inserter(posesbySurface), [](const poseType& pose){return (pose.flatSurfaceSeen==i);} ); //Copy only the analized flat surface
    
    //poseArrayType::iterator it = std::copy_if (posesBackup.begin(), posesBackup.end(), posesbySurface.begin(), [](poseType pose){return (pose.flatSurfaceSeen==i);} ); //Copy only the analized flat surface
    //posesbySurface.resize(std::distance(posesbySurface.begin(),it));  // shrink container to new size
    if (posesbySurface.size() > 0 ) {
      selectPath(posesbySurface, xInit, yInit); //Find shortes path with the path method
      xInit=posesbySurface.back().x; //The last point of the last path will be the first point of the new one
      yInit=posesbySurface.back().y;
      poses.insert( poses.end(), posesbySurface.begin(), posesbySurface.end() ); //Put togheter all the paths by flat surface
    }
  }
}

void filterCloseToObstacles(Mat obstacles, poseArrayType &poses, double minDistance, double minX=0, double minY=0, double maxX=0, double maxY=0, double resolution=1) {
  poseArrayType posesBackup=poses;
  poses.clear();
  for (int i=0; i<posesBackup.size(); i++) {
    if ( freeSpace(obstacles, posesBackup[i].x-minDistance, posesBackup[i].y-minDistance, posesBackup[i].x+minDistance, posesBackup[i].y+minDistance, minX, minY, maxX, maxY, resolution) ) {
      poses.push_back(posesBackup[i]);
    }
  }
}

double angleDiff(double a1, double a2) {
  double angle = fmod ( abs(a2- a1) , 360 );
  if (angle > 180) 
    return 360 - angle;
  else
    return angle;
}


//TODO TODO Podría cambiarse este metodo para que el filtro funcionara con el porcentaje de traslape. Si tienen cierto porcentaje entonces verificamos el angulo entre las poses. Si este angulo es menor que un umbral (porcentaje de 360 grados que a lo mejor puede ser el porcentaje del angulo que forma el cono de visibilidad) entonces la pose es similar. Habria que analizar si todavia se requiere el filtro de poses traslapadas. Ya lo pensé que si se requiere el otro filtro. Si no se tuviera y pongo un porcentaje de angulo muy grande se tendran poses que miran al mismo lugar y con el mismo angulo, lo que nunca se requiere.
//TODO Si reduzco el traslape elimino las poses similares. Pero si reduzco las similares no reduzco el traslape.
void filterSimilarPoses(poseArrayType &poses, double minDistance, double minAngle) {
  //This method supposes poses sorted by flatArea desc
  poseArrayType posesBackup=poses;
  poses.clear();
  for (int i=0; i<posesBackup.size(); i++) {
    bool isClose=false;
    for (int j=0; j<poses.size(); j++) {
      //if ( abs(posesBackup[i].x-poses[j].x) < minDistance && abs(posesBackup[i].y-poses[j].y) < minDistance && abs(posesBackup[i].theta-poses[j].theta) < minAngle ) {
      if ( sqrt( pow(posesBackup[i].x-poses[j].x,2) + pow(posesBackup[i].y-poses[j].y,2) ) < minDistance && angleDiff(posesBackup[i].theta,poses[j].theta) < minAngle ) {
	isClose=true;
	break;
      }
    }
    if (!isClose) poses.push_back(posesBackup[i]);
  }
}



bool wayToSortPoses(poseType i, poseType j) {
  return i.flatArea > j.flatArea;
}

bool wayToSortPosesByOverlapedArea(poseType i, poseType j) {
  return i.overlapedArea < j.overlapedArea;
}


void filterOverlapedViews(poseArrayType &poses, boostPolygonType visibilityCone, double PercentageAreaThreshold) {
  //This method supposes poses sorted by SeenArea desc
  sort(poses.begin(), poses.end(), wayToSortPoses);
  double visibilityConeArea = area(visibilityCone);
  poseArrayType posesNoOverlaped;
  while (poses.size()>0) {
    posesNoOverlaped.push_back(poses[0]);
    translate_transformer<boostPointType, boostPointType> translate(posesNoOverlaped.back().x, posesNoOverlaped.back().y);
    rotate_transformer<boostPointType, boostPointType, degree> rotate(posesNoOverlaped.back().theta);
    boostPolygonType transf, transf2;
    transform(visibilityCone, transf, rotate);
    transform(transf, transf2, translate);
    poseArrayType posesBackup;
    for (int i=1; i<poses.size(); i++) { //The first pose is already saved
      boostMultiPolygonType poseVisibilityIntersection;  //The intersection could be more than one polygon
      translate_transformer<boostPointType, boostPointType> translate2(poses[i].x, poses[i].y);
      rotate_transformer<boostPointType, boostPointType, degree> rotate2(poses[i].theta);
      boostPolygonType transf3, transf4;
      transform(visibilityCone, transf3, rotate2);
      transform(transf3, transf4, translate2);
      intersection(transf2, transf4, poseVisibilityIntersection);
      if ( area(poseVisibilityIntersection)/visibilityConeArea < PercentageAreaThreshold ) {
	posesBackup.push_back(poses[i]);
      }
    }
    poses.clear();
    poses=posesBackup;
  }
  poses.clear();
  poses=posesNoOverlaped;
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
    double maxSurfaceArea=0;
    int indexMaxSurfaceArea=0;
    double totalIntersectionArea=0;
    for (int j=0; j<flatSurfaces.size(); j++) {
      deque<boostPolygonType> flatVisibilityIntersection;  //The intersection could be more than one polygon
      intersection(transf2, flatSurfaces[j], flatVisibilityIntersection);
      double partialIntersectionArea = 0;
      BOOST_FOREACH(boostPolygonType const& polygon, flatVisibilityIntersection) { //Each polygon resulting for intersection
	partialIntersectionArea += area(polygon);
      }
      if (partialIntersectionArea > maxSurfaceArea) {
	maxSurfaceArea = partialIntersectionArea;
	indexMaxSurfaceArea = j;
      }
      totalIntersectionArea += partialIntersectionArea;
    }
    if (totalIntersectionArea>0) {
      posesBackup[i].flatArea = totalIntersectionArea;
      posesBackup[i].flatSurfaceSeen = indexMaxSurfaceArea; //If poses by Surface are created this should be done after calling evaluatePoses because only one surface by room is sent.
      poses.push_back(posesBackup[i]);
      numValidPoses++;
    }
  }
  
  posesBackup.clear();
  cout << "Flat view poses: " << numValidPoses << endl;
  sort(poses.begin(), poses.end(), wayToSortPoses);
  for (int i=0; i<poses.size(); i++) {
    //cout << "Pose X: " << poses[i].x << "  Y: " << poses[i].y << "  Area:" << poses[i].flatArea << endl;
    //TODO TODO TODO No debería de haber un máximo y este procedimiento debe llamarse filterNoSurfaceSeen
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
    
    if ( within(make<boostPointType>(pose.x, pose.y), space) && freeSpace(map, pose.x, pose.y, pose.x, pose.y, minX, minY, maxX, maxY, resolution) ) {
      poses.push_back(pose);
      i++;
    }
    else {
      //drawPoint(mapBackup, pose.x, pose.y, minX, minY, maxX, maxY, resolution, Scalar(50,50,50), 8);
      //cout << "Pose X: " << pose.x << "  Y: " << pose.y << "  no valid" << endl;
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


void measurePosesQuality(vector <boostPolygonType> flatSurfaces, poseArrayType poses, boostPolygonType visibilityCone, double &totalCoverageArea, double &totalOverlapedArea) {
  totalOverlapedArea = 0;
  boostMultiPolygonType coveragePolygon;
  for (int i=0; i<poses.size(); i++) {
    translate_transformer<boostPointType, boostPointType> translate(poses[i].x, poses[i].y);
    rotate_transformer<boostPointType, boostPointType, degree> rotate(poses[i].theta);
    boostPolygonType transf, transf2;
    transform(visibilityCone, transf, rotate);
    transform(transf, transf2, translate);
    
    for (int j=0; j<flatSurfaces.size(); j++) {
      boostMultiPolygonType flatVisibilityIntersection;  //The intersection could be more than one polygon
      intersection(transf2, flatSurfaces[j], flatVisibilityIntersection);
      
      boostMultiPolygonType overlapedPosePolygon;
      intersection(flatVisibilityIntersection, coveragePolygon, overlapedPosePolygon);
      totalOverlapedArea += area(overlapedPosePolygon);
      
      BOOST_FOREACH(boostPolygonType const& polygon, flatVisibilityIntersection) { //Each polygon resulting for intersection
	boostMultiPolygonType tmpCoveragePolygon;
	union_(coveragePolygon,polygon,tmpCoveragePolygon);
	coveragePolygon = tmpCoveragePolygon;
      }
    }
  }
  
  double totalFlatArea=0;
  for (int i=0; i<flatSurfaces.size(); i++) {
    totalFlatArea += area(flatSurfaces[i]);
  }
  totalCoverageArea=area(coveragePolygon);
  cout << "Visibility cone area: " << area(visibilityCone) << endl;
  cout << "Flat area: " << totalFlatArea << endl;
  cout << "Coverage area: " << totalCoverageArea << endl;
  cout << "Coverage percentage: " << 100*totalCoverageArea/totalFlatArea << endl;
  cout << "Overlaped area: " << totalOverlapedArea << endl;
  cout << "Overlaped percentage: " << 100*totalOverlapedArea/totalFlatArea << endl;
}



//TODO Measure the ideal length path (considering no obstacles nor walls in the map). The pose probability of seeing the object is the percentage of flat surface seen by the pose weighted by the probability of the flat surface (the percentage of all areas in the room. It considers that the object recognizer never fails.
double measurePathQuality(vector <boostPolygonType> flatSurfaces, poseArrayType poses, double xInit, double yInit, boostPolygonType visibilityCone) {
  double traveledExpectedDistance = 0;

  double totalFlatArea=0;
  for (int i=0; i<flatSurfaces.size(); i++) {
    totalFlatArea += area(flatSurfaces[i]);
  }
  
  for (int i=0; i<poses.size(); i++) {
    translate_transformer<boostPointType, boostPointType> translate(poses[i].x, poses[i].y);
    rotate_transformer<boostPointType, boostPointType, degree> rotate(poses[i].theta);
    boostPolygonType transf, transf2;
    transform(visibilityCone, transf, rotate);
    transform(transf, transf2, translate);

    double totalIntersectionArea=0;
    for (int j=0; j<flatSurfaces.size(); j++) {
      boostMultiPolygonType flatVisibilityIntersection;  //The intersection could be more than one polygon
      intersection(transf2, flatSurfaces[j], flatVisibilityIntersection);
      totalIntersectionArea += area(flatVisibilityIntersection);
    }
    traveledExpectedDistance += distanceToPoint(xInit, yInit, poses[i].x,poses[i].y) * (totalIntersectionArea/totalFlatArea);
    xInit = poses[i].x;
    yInit = poses[i].y;
  }
  cout << "Traveled expected distance: " << traveledExpectedDistance << endl;
  return traveledExpectedDistance;
}