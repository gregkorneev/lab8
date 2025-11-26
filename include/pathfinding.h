#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <vector>
#include <string>
#include "grid.h"

// Классический A*
bool aStar(const std::vector<std::string> &grid,
           Cell start, Cell goal,
           std::vector<Cell> &path,
           int &closedCount);

// Theta* (any-angle поиск)
bool thetaStar(const std::vector<std::string> &grid,
               Cell start, Cell goal,
               std::vector<Cell> &path,
               int &closedCount);

// Сглаживание пути (A*PS)
void smoothPath(const std::vector<std::string> &grid,
                const std::vector<Cell> &in,
                std::vector<Cell> &out);

// Длина пути в "геометрическом" смысле
double pathLength(const std::vector<Cell> &path);

#endif // PATHFINDING_H
