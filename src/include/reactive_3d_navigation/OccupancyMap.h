#ifndef OCCUPANCYMAP_H
#define OCCUPANCYMAP_H

#include <unordered_set>
#include <utility>

using namespace std;

struct IntPairHash
{
 public:
  // adapted from boost::hash_combine
  size_t operator()(const pair<int, int>& v) const
  {
      //return std::hash<int>()(v.first) ^ std::hash<int>()(v.second);
      size_t result = v.first;
      result ^= v.second + 0x9e3779b9 + (v.first << 6) + (v.first >> 2);
      return result;
  }
};


typedef unordered_set<pair<int, int>, IntPairHash> ChangeSet;

class OccupancyMap {
 public:
  OccupancyMap(int radius, int resolution);
  
  // Occupy a position
  void occupy(double x, double y);
  
  // Returns whether a position is occupied
  bool isOccupied(double x, double y) const;
  
  // Returns whether an index in the map in [-radius, radius] is occupied.
  // Use indexToCoord() to convert that index into a coordinate.
  bool isOccupiedIndex(int x, int y) const;
  
  // Update robot position
  void updatePos(double x, double y);
  
  // Free all positions along the line of two points.
  void freeInBetween(double toX, double toY);

  // Convert index to coordinate.
  double indexToCoord(int idx) const {
    return ((double)idx) / _resolution;
  }
  
  // Get the robot position.
  double getOffsX() const { return _offsxf; }
  
  // Get robot position
  double getOffsY() const { return _offsyf; }
  
  int getRadius() const { return _radius; }
  
#ifdef TRACK_CHANGES
  ChangeSet popAddedPositions() const;
  ChangeSet popDeletedPositions() const;
#endif

 private:
  // Map.
  bool** _map;

  // How many cells the map has.
  int _mapSize;
  
  // How many index position we have to shift the array
  int _offsx;
  int _offsy;
  
  double _offsxf;
  double _offsyf;
  
  int _radius;
  int _resolution;
  
#ifdef TRACK_CHANGES
  mutable ChangeSet _addSet;
  mutable ChangeSet _deleteSet;
#endif

  // Set a position at a given index to "free".
  // Updates add and delete set.
  void setFree(int map_idx, int map_idy);

  // Set a position at a given index to "occupied".
  // Updates add and delete set.
  void setOccupied(int map_idx, int map_idy);
  
  void deleteRows(int from, int to);
  
  void deleteColums(int from, int to);
  
  // Free all positions along the line of two points.
  void _freeInBetween(int fromX, int fromY, int toX, int toY);
  
  int EUCMOD(int a, int mod) const {
    return ((a % mod) + mod) % mod;
  }
  
  // Convert a coordinate value into a virtual index in the occupancy map.
  int toIndex(double x) const {
    return (int)(x * _resolution + 0.5);
  }
  
  // Convert a virtual index index into an actual position in the array.
  int toMapIndex(int idx, int offs) const {
    return EUCMOD(idx + offs + _radius, _mapSize);
  }

};

#endif  // OCCUPANCYMAP_H
