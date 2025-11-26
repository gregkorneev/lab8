#pragma once
#include <vector>
#include <string>
#include "grid.h"

struct PathMetrics {
    double L_opt      = 0.0;
    double L_found    = 0.0;
    double KO         = 0.0;
    double OO_percent = 0.0;
    int    closedCount = 0;

    double EP         = 0.0;
    double FV         = 0.0;
    double SUP        = 0.0;
    double GP         = 0.0;
    double curvature  = 0.0;

    double minObsDist = 0.0;
    double avgObsDist = 0.0;

    double heurError  = 0.0;
};

// Алгоритмы поиска
bool aStar(const std::vector<std::string>& grid,
           Cell start, Cell goal,
           std::vector<Cell>& outPath,
           int& outClosedCount);

bool thetaStar(const std::vector<std::string>& grid,
               Cell start, Cell goal,
               std::vector<Cell>& outPath,
               int& outClosedCount);

// Пост-сглаживание
void smoothPath(const std::vector<std::string>& grid,
                const std::vector<Cell>& inPath,
                std::vector<Cell>& outSmooth);

// Длина пути
double pathLength(const std::vector<Cell>& path);

// Оценка эвристики (используется только в режиме 1)
double computeHeuristicError(const std::vector<std::string>& grid,
                             Cell goal);

// Расчёт всех метрик
void fillMetrics(const std::vector<std::string>& grid,
                 Cell start, Cell goal,
                 const std::vector<Cell>& finalPath,
                 int closedCount,
                 double heurError,
                 PathMetrics& m);

// Сохранение метрик в CSV
void saveMetricsToCsv(const std::string& name,
                      int h, int w,
                      const PathMetrics& m,
                      int runIndex);

// Отрисовка карты с путём
void printMapWithPath(std::vector<std::string> grid,
                      const std::vector<Cell>& path);
