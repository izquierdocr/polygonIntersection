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

  //Use a different random seed
  srand (time(NULL));
  
  Mat map;
  loadMapfromAria(map, minX, minY, maxX, maxY, resolution, "lab_2015.map");
  //loadMapfromAria(map, minX, minY, maxX, maxY, resolution, "bigroom01.map");
  
  Mat mapFree;
  Mat kernel = Mat::ones(14,14,CV_8UC1);
  dilate(map, mapFree, kernel, Point(-1,-1), 1);
  //mapFree=map.clone();
  
  vector <boostPolygonType> polygonsFile;  //Load polygon reads many polygons from a file but we only have one for cone
  loadPolygons(polygonsFile, "visibilityCone.txt");
  boostPolygonType visibilityCone = polygonsFile[0];
  
  vector <boostPolygonType> flatSurfaces;
  loadPolygons(flatSurfaces, "flatSurfaces.txt");
  drawFlatSurfaces(mapFree, flatSurfaces, minX, minY, maxX, maxY, resolution); //This not put an image in screen. Its to add the flat surfaces in the map
  //Mat mapFlip; flip(mapFree,mapFlip,0); imshow("Planos", mapFlip); waitKey(0);
  
  string roomName="livingroom"; //diningroom livingroom kitchen bathroom bedroom studio bigroom01
  vector <boostPolygonType> room;
  loadPolygons(room, roomName+".txt");
  
  vector <boostPolygonType> flatSurfacesinRoom;
  flatSurfacesinRoom = selectFlatSurfaces(room[0], flatSurfaces);
  
  
  
  
  
  //For making repetitive experiments
  ofstream fileStream("experiment-"+roomName+".txt");
  fileStream << "Room,";
  fileStream << "Overlap Threshold,";
  fileStream << "Repetition,";
  fileStream << "Flat Surfaces,";
  fileStream << "Visibility Cone Area,";
  fileStream << "Poses Generated,";
  fileStream << "Final Poses,";
  fileStream << "Total Flat Surface,";
  fileStream << "Coverage,";
  fileStream << "Overlap,";
  fileStream << "Traveled Expected Distance Path1,";
  fileStream << "Traveled Expected Distance Path2,";
  fileStream << endl;
  
  for (double threshold=0; threshold<=1; threshold+=0.2) {
  for (int repetition=0; repetition<5; repetition++) {
  
  
  poseArrayType poses;
  
  int numPoses=1000;
  generatePoses(mapFree, poses, numPoses, visibilityCone, room[0], minX, minY, maxX, maxY, resolution);
  //drawPoses(map, poses, visibilityCone, minX, minY, maxX, maxY, resolution,"All poses");
  /*
  //Method 1. Select poses by room. NOTE: In rooms with flat surfaces with too much difference in their areas. Small flat surfaces tend to have no poses.
  int maxPoses=numPoses*0.1; //A percentage of the poses (with the biggest seen area)
  evaluatePoses(poses, flatSurfacesinRoom, visibilityCone, maxPoses);
  
  double obstacleDistance=300;  //Distance to be considered close to obstacles in milimeters
  filterCloseToObstacles(mapFree, poses, obstacleDistance, minX, minY, maxX, maxY, resolution);
  cout << poses.size() << " poses after delete poses close to obstacles" << endl;
  
  double closeDistance=1400;  //Distance to be considered close to other poses in milimeters
  double closeAngle=40;   //Angle to be considered close to other poses in degrees
  filterSimilarPoses(poses, closeDistance, closeAngle);
  cout << poses.size() << " poses after delete similar poses" << endl;
  //End Method 1
  */
  
  //Method 2. Select poses by flat surfaces
  int maxPoses=numPoses*0.1; //A percentage of the poses (with the biggest seen area)
  poseArrayType posesTMP=poses;
  poses.clear();
  for (int i=0; i<flatSurfacesinRoom.size(); i++) {
    poseArrayType posesbySurface = posesTMP;
    vector <boostPolygonType> oneFlatSurfacesinRoom;
    oneFlatSurfacesinRoom.push_back( flatSurfacesinRoom[i] );
    evaluatePoses(posesbySurface, oneFlatSurfacesinRoom, visibilityCone, maxPoses);
    
    double obstacleDistance=300;  //Distance to be considered close to obstacles in milimeters
    filterCloseToObstacles(mapFree, posesbySurface, obstacleDistance, minX, minY, maxX, maxY, resolution);
    cout << poses.size() << " poses after delete poses close to obstacles" << endl;
    
    double closeDistance=1400;  //Distance to be considered close to other poses in milimeters
    double closeAngle=40;   //Angle to be considered close to other poses in degrees
    filterSimilarPoses(posesbySurface, closeDistance, closeAngle);
    cout << poses.size() << " poses after delete similar poses" << endl;
    
    for (int j=0; j<posesbySurface.size(); j++) {posesbySurface[j].flatSurfaceSeen=i;}   //Group poses by surface seen to find a path. This code overwrite one inside evaluate poses because only one surface is sent each time
    poses.insert( poses.end(), posesbySurface.begin(), posesbySurface.end() );
    cout << "Flat " << i << " with " << posesbySurface.size() << endl;
  }
  double closeDistance=1400;  //Distance to be considered close to other poses in milimeters
  double closeAngle=30;   //Angle to be considered close to other poses in degrees
  filterSimilarPoses(poses, closeDistance, closeAngle);
  cout << poses.size() << " poses after delete similar poses" << endl;
  //End Method 2
    
  //drawPoses(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution, "Similar poses filtered");
  
  
  double overlapedPercentageThreshold=threshold;  //Porcentage to evaluate overlaped versus visibility cone
  //overlapedPercentageThreshold can't be 0 because it means delete poses without intersection
  //if (threshold==0.0) overlapedPercentageThreshold=0.05;
  filterOverlapedViews(poses, visibilityCone, overlapedPercentageThreshold);
  cout << poses.size() << " poses after delete redundant (overlaped) poses" << endl;
  
  //drawPoses(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution,"Overlaped poses filtered");
                 //AQUI map
  double xInit=5950;
  double yInit=-98;
  selectPath(poses, xInit, yInit);
  //drawPath(map, poses, visibilityCone, minX, minY, maxX, maxY, resolution, "Closest pose path");
  double distancePath1 = measurePathQuality(flatSurfacesinRoom, poses, xInit, yInit, visibilityCone);
  
  xInit=5950;
  yInit=-98;
  selectPath2(flatSurfacesinRoom, poses, xInit, yInit);
  //drawPath(map, poses, visibilityCone, minX, minY, maxX, maxY, resolution, "Biggest surface path");
  double distancePath2 = measurePathQuality(flatSurfacesinRoom, poses, xInit, yInit, visibilityCone);
  
  double coverage, overlap;
  measurePosesQuality(flatSurfacesinRoom, poses, visibilityCone, coverage, overlap);
  
  
  
  fileStream << roomName << ",";
  fileStream << overlapedPercentageThreshold << ",";
  fileStream << repetition << ",";
  fileStream << flatSurfacesinRoom.size() << ",";
  fileStream << area(visibilityCone) << ",";
  fileStream << numPoses << ",";
  fileStream << poses.size() << ",";

  double totalFlatArea=0;
  for (int i=0; i<flatSurfacesinRoom.size(); i++) {
    totalFlatArea += area(flatSurfacesinRoom[i]);
  }  
  fileStream << totalFlatArea << ",";
  fileStream << coverage << ",";
  fileStream << overlap << ",";
  fileStream << distancePath1 << ",";
  fileStream << distancePath2;
  fileStream << endl;
  
  
  }
  }
  fileStream.close();
  cout << endl << "End of test!" << endl;
  
  return 0;
}
