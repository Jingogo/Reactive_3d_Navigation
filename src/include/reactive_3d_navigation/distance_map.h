#ifndef _Distance_Map_H_
#define _Distance_Map_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include "dynamicvoronoi/bucketedqueue.h"

//! A Distance_Map object computes and updates a distance map and Voronoi diagram.
class Distance_Map {
  
public:
  
  Distance_Map();
  ~Distance_Map();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT> newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist=true);

  //! returns the obstacle distance at the specified location
  float getDistance ( int x, int y ) const;

  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);


  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() const {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() const {return sizeY;}


private:  
  struct dataCell {
    float dist;
    char queueing;
    int obstX;
    int obstY;
    bool needsRaise;
    int sqdist;
  };


  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;


  
  // methods
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
 
  void commitAndColorize(bool updateRealDist=true);
  
  inline bool isOccupied(int &x, int &y, dataCell &c);

  // queues

  BucketPrioQueue open;

  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  // maps
  int sizeY;
  int sizeX;
  dataCell** data;
  bool** gridMap;

  // parameters
  int padding;
  double doubleThreshold;
  int maxDistSquared;
  double sqrt2;

  //  dataCell** getData(){ return data; }
};


#endif

