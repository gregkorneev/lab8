#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <vector>
#include <string>
#include "grid.h"

// Метрики пути по ТЗ
struct PathMetrics {
    double L_found;        // длина найденного пути
    double L_opt;          // оптимальная длина (Dijkstra)
    double KO;             // коэффициент оптимальности
    double OO_percent;     // отклонение от оптимального, %

    int    closedCount;    // количество раскрытых узлов

    double EP;             // эффективность поиска (L_opt / N_раскрытых)
    double FV;             // фактор ветвления

    double SUP;            // суммарный угол поворотов
    double GP;             // гладкость пути

    double minObsDist;     // минимальное расстояние до препятствия
    double avgObsDist;     // среднее расстояние до препятствий

    double curvature;      // суммарная кривизна пути
    double heurError;      // точность эвристики (RMSE)
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

// Длина пути в геометрическом смысле
double pathLength(const std::vector<Cell> &path);

// --- Вспомогательные функции для метрик ---

// Оптимальная длина пути (Dijkstra), true если путь найден
bool computeOptimalPathLength(const std::vector<std::string> &grid,
                              Cell start, Cell goal,
                              double &L_opt);

// Гладкость, суммарный угол поворотов и кривизна
void computeSmoothness(const std::vector<Cell> &path,
                       double &SUP, double &GP,
                       double &curvature);

// Расстояния до препятствий
void computeObstacleDistances(const std::vector<std::string> &grid,
                              const std::vector<Cell> &path,
                              double &minDist, double &avgDist);

// Заполнить все метрики
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
