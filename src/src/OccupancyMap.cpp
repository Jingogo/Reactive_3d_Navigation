#include <stdlib.h>  // abs

#include "reactive_3d_navigation/OccupancyMap.h"

#define UNKNOWN_VAL false

OccupancyMap::OccupancyMap(int radius, int resolution) {
  _mapSize = radius * 2 + 1;
  _map = new bool*[_mapSize];
  
  for (int i = 0; i < _mapSize; i++) {
    _map[i] = new bool[_mapSize];
    for (int j = 0; j < _mapSize; j++) {
      _map[i][j] = false;
    }
  }
  
  _offsx = 0;
  _offsy = 0;
  
  _radius = radius;
  _resolution = resolution;
}

void OccupancyMap::occupy(double x, double y) {
  int idx = toIndex(x);
  int idy = toIndex(y);
  if (idx < -_radius || idx > _radius || idy < -_radius || idy > _radius) {
    // Do nothing, this coordinate is out of range.
    return;
  }

  int map_idx = toMapIndex(idx, _offsx);
  int map_idy = toMapIndex(idy, _offsy);
  
  setOccupied(map_idx, map_idy);
}

bool OccupancyMap::isOccupied(double x, double y) const {
  return isOccupiedIndex(toIndex(x), toIndex(y));
}

void OccupancyMap::updatePos(double x, double y) {
  int newOffsx = (int)(x * _resolution + 0.5);
  int newOffsy = (int)(y * _resolution + 0.5);
  
  // Cleanup array
  deleteRows(_offsx, newOffsx);
  deleteRows(newOffsx, _offsx);
  
  deleteColums(_offsy, newOffsy);
  deleteColums(newOffsy, _offsy);
  
  _offsx = (int)(x * _resolution + 0.5);
  _offsy = (int)(y * _resolution + 0.5);
  _offsxf = x;
  _offsyf = y;
}

#ifdef TRACK_CHANGES
ChangeSet OccupancyMap::popAddedPositions() const {
  ChangeSet result = _addSet;
  _addSet.clear();
  return _addSet;
}

ChangeSet OccupancyMap::popDeletedPositions() const {
  ChangeSet result = _deleteSet;
  _deleteSet.clear();
  return _deleteSet;
}
#endif

void OccupancyMap::deleteRows(int from, int to) {
  for (int x = from; x < (to); x++) {
    int map_idx = EUCMOD(x, _mapSize);
    for (int y = -_radius; y < _radius; y++) {
      int map_idy = toMapIndex(y, _offsy);
      if (UNKNOWN_VAL) {
        setOccupied(map_idx, map_idy); 
      } else {
        setFree(map_idx, map_idy); 
      }
    }
  }
}

void OccupancyMap::deleteColums(int from, int to) {
  for (int x = -_radius; x < _radius; x++) {
    int map_idx = toMapIndex(x, _offsy);
    for (int y = from; y < (to); y++) {
      int map_idy = EUCMOD(y, _mapSize);
      if (UNKNOWN_VAL) {
        setOccupied(map_idx, map_idy); 
      } else {
        setFree(map_idx, map_idy); 
      } 
    }
  }
}

#ifdef TRACK_CHANGES
void OccupancyMap::setFree(int map_idx, int map_idy) {
  if (_map[map_idx][map_idy]) {
    _map[map_idx][map_idy] = false;
    _deleteSet.insert(make_pair(map_idx, map_idy));
  }
  _addSet.erase(make_pair(map_idx, map_idy));
}
#else
void OccupancyMap::setFree(int map_idx, int map_idy) {
  _map[map_idx][map_idy] = false;
}
#endif

#ifdef TRACK_CHANGES
void OccupancyMap::setOccupied(int map_idx, int map_idy) {
  if (!_map[map_idx][map_idy]) {
    _map[map_idx][map_idy] = true;
    _addSet.insert(make_pair(map_idx, map_idy));
  }
  _deleteSet.erase(make_pair(map_idx, map_idy));
}
#else
void OccupancyMap::setOccupied(int map_idx, int map_idy) {
  _map[map_idx][map_idy] = true;
}
#endif

void OccupancyMap::freeInBetween(double toX, double toY) {
  int idx = toIndex(toX);
  int idy = toIndex(toY);
  _freeInBetween(0, 0, idx, idy);
}

void OccupancyMap::_freeInBetween(int fromX, int fromY, int toX, int toY) {
  // Bresenheim's algorithm
  int dx =  abs(toX - fromX), sx = fromX < toX ? 1 : -1;
  int dy = -abs(toY - fromY), sy = fromY < toY ? 1 : -1;
  int err = dx + dy; // error value e_xy

  while(true) {
    if (fromX < -_radius || fromX > _radius || fromY < -_radius || fromY > _radius) {
      // Break, this coordinate is out of range.
      break;
    }  
    int map_idx = toMapIndex(fromX, _offsx);
    int map_idy = toMapIndex(fromY, _offsy);
    setFree(map_idx, map_idy);
    if (fromX == toX && fromY == toY) break;
    int e2 = 2*err;
    if (e2 > dy) { err += dy; fromX += sx; } // e_xy + e_x > 0
    if (e2 < dx) { err += dx; fromY += sy; } // e_xy + e_y < 0
  }
}

bool OccupancyMap::isOccupiedIndex(int idx, int idy) const {
  if (idx < -_radius || idx > _radius || idy < -_radius || idy > _radius) {
    return false;
  }
  int map_idx = toMapIndex(idx, _offsx);
  int map_idy = toMapIndex(idy, _offsy);
  return _map[map_idx][map_idy];
}
