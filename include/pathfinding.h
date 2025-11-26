#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <vector>
#include <string>
#include "grid.h"

// Метрики пути для анализа по ТЗ
struct PathMetrics {
    double L_found;       // длина найденного пути
    double L_opt;         // оптимальная длина (по Dijkstra)
    double KO;            // коэффициент оптимальности
    double OO_percent;    // отклонение от оптимального, %

    int closedCount;      // количество раскрытых узлов

    double EP;            // вычислительная эффективность (L_opt / closedCount)
    double FV;            // фактор ветвления

    double SUP;           // суммарный угол поворотов
    double GP;            // гладкость пути

    double minObsDist;    // минимальное расстояние до препятствия
    double avgObsDist;    // среднее расстояние до препятствий
};

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

// --------- Доп. функции для метрик ---------

// Посчитать оптимальную длину пути (Dijkstra)
// Возвращает true, если путь найден, и пишет длину в L_opt.
bool computeOptimalPathLength(const std::vector<std::string> &grid,
                              Cell start, Cell goal,
                              double &L_opt);

// Посчитать гладкость (SUP и GP) по ломаной траектории
void computeSmoothness(const std::vector<Cell> &path,
                       double &SUP, double &GP);

// Посчитать минимальное и среднее расстояние до препятствий вдоль пути
void computeObstacleDistances(const std::vector<std::string> &grid,
                              const std::vector<Cell> &path,
                              double &minDist, double &avgDist);

// Заполнить все метрики по ТЗ
void fillMetrics(const std::vector<std::string> &grid,
                 Cell start, Cell goal,
                 const std::vector<Cell> &finalPath,
                 int closedCount,
                 PathMetrics &m);

// Сохранить метрики в CSV (data/csv/metrics.csv)
void saveMetricsToCsv(const std::string &algorithmName,
                      int h, int w,
                      const PathMetrics &m);

#endif // PATHFINDING_H
