#ifndef RASTERIZATION_HELPER_H
#define RASTERIZATION_HELPER_H

#include <vector>

#include "ros/ros.h"


struct intPoint {
  intPoint(int _x, int _y) {
    x = _x;
    y = _y;
  }
  intPoint() {
    x = 0;
    y = 0;
  }
  int x;
  int y;
};

intPoint toPoint(int x, int y) {
  intPoint p;
  p.x = x;
  p.y = y;
  return p;
}

std::vector<intPoint> rasterCircle(int x0, int y0, int radius)
{
  int f = 1 - radius;
  int ddF_x = 0;
  int ddF_y = -2 * radius;
  int x = 0;
  int y = radius;
  
  std::vector<intPoint> result;

  result.push_back(toPoint(x0, y0 + radius));

  while(x <= radius)
  {
    if(f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x + 1;

    result.push_back(toPoint(x0 + x, y0 + y));
  }
  
  return result;
}

std::vector<intPoint> rasterLine(int fromX, int fromY, int toX, int toY) {
  // Bresenheim's algorithm
  int dx =  abs(toX - fromX), sx = fromX < toX ? 1 : -1;
  int dy = -abs(toY - fromY), sy = fromY < toY ? 1 : -1;
  int err = dx + dy; // error value e_xy

  std::vector<intPoint> result;

  while(true) {
    result.push_back(toPoint(fromX, fromY));
    if (fromX == toX && fromY == toY) break;
    int e2 = 2*err;
    if (e2 > dy) { err += dy; fromX += sx; } // e_xy + e_x > 0
    if (e2 < dx) { err += dx; fromY += sy; } // e_xy + e_y < 0
  }
  
  return result;
}

#endif  // RASTERIZATION_HELPER_H

