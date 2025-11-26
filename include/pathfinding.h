#pragma once

#include <vector>
#include <string>

#include "grid.h"  // здесь уже объявлены struct Cell и printMapWithPath

// Метрики качества маршрута
struct PathMetrics {
    double L_opt      = 0.0;  // оптимальная длина (Dijkstra)
    double L_found    = 0.0;  // найденная длина
    double KO         = 0.0;  // коэффициент оптимальности
    double OO_percent = 0.0;  // отклонение от оптимального, %
    int    closedCount = 0;   // число раскрытых узлов

    double EP         = 0.0;  // эффективность поиска
    double FV         = 0.0;  // фактор ветвления
    double SUP        = 0.0;  // суммарный угол поворотов
    double GP         = 0.0;  // гладкость пути
    double curvature  = 0.0;  // кривизна траектории

    double minObsDist = 0.0;  // минимальное расстояние до препятствий
    double avgObsDist = 0.0;  // среднее расстояние до препятствий

    double heurError  = 0.0;  // RMSE эвристики (точность эвристики)
};

// --- Алгоритмы поиска пути ---

bool aStar(const std::vector<std::string>& grid,
           Cell start, Cell goal,
           std::vector<Cell>& outPath,
           int& outClosedCount);

bool thetaStar(const std::vector<std::string>& grid,
               Cell start, Cell goal,
               std::vector<Cell>& outPath,
               int& outClosedCount);

// Пост-сглаживание пути
void smoothPath(const std::vector<std::string>& grid,
                const std::vector<Cell>& inPath,
                std::vector<Cell>& outSmooth);

// Длина пути
double pathLength(const std::vector<Cell>& path);

// Сохранение карты и пути в CSV
void saveMapAndPathToCsv(const std::vector<std::string>& grid,
                         const std::vector<Cell>& path,
                         const std::string& filename);

// Расчёт ошибки эвристики (RMSE) для одной карты
double computeHeuristicError(const std::vector<std::string>& grid,
                             Cell goal);

// Заполнение структуры метрик
void fillMetrics(const std::vector<std::string>& grid,
                 Cell start, Cell goal,
                 const std::vector<Cell>& finalPath,
                 int closedCount,
                 double heurError,
                 PathMetrics& m);

// Сохранение метрик в CSV
void saveMetricsToCsv(const std::string& algorithmName,
                      int height, int width,
                      const PathMetrics& m,
                      int runIndex);
