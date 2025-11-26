#include <iostream>
#include <vector>
#include <string>

#include "grid.h"
#include "pathfinding.h"

// Прототипы
void run_single();
void run_batch();

int main() {
    std::cout << "Лабораторная работа 8.\n";
    std::cout << "Поиск безопасного пути через лес во время пожара.\n\n";

    std::cout << "Режим работы:\n";
    std::cout << "1 — Один запуск (интерактивный)\n";
    std::cout << "2 — Серия запусков (статистика по 4–5+ картам)\n";
    std::cout << "Выберите режим: ";

    int mode = 0;
    if (!(std::cin >> mode)) {
        std::cout << "Ошибка ввода.\n";
        return 0;
    }

    if (mode == 1) {
        run_single();
    } else if (mode == 2) {
        run_batch();
    } else {
        std::cout << "Неизвестный режим.\n";
    }

    return 0;
}

// ------------------ ОДИНОЧНЫЙ ЗАПУСК ------------------

void run_single() {
    int h, w;
    std::cout << "Введите высоту и ширину карты (например, 10 15): ";
    if (!(std::cin >> h >> w)) {
        std::cout << "Ошибка ввода.\n";
        return;
    }
    if (h <= 1 || w <= 1) {
        std::cout << "Размеры должны быть больше 1.\n";
        return;
    }

    std::vector<std::string> grid;
    Cell start;
    Cell goal;
    generateRandomMap(h, w, grid, start, goal);

    std::cout << "Сгенерированная карта леса (S - вы, G - безопасная зона, # - огонь/заросли):\n";
    for (const auto& row : grid) {
        std::cout << row << "\n";
    }
    std::cout << "\n";

    std::cout << "Выберите алгоритм поиска пути:\n";
    std::cout << "1 — A*\n";
    std::cout << "2 — A* с пост-сглаживанием (A*PS)\n";
    std::cout << "3 — Theta*\n";

    int choice = 0;
    std::cin >> choice;

    std::vector<Cell> path;
    std::vector<Cell> smooth;
    int closedCount = 0;
    bool ok = false;
    std::string algorithmName;

    if (choice == 1) {
        algorithmName = "A*";
        ok = aStar(grid, start, goal, path, closedCount);
    } else if (choice == 2) {
        algorithmName = "A*PS";
        ok = aStar(grid, start, goal, path, closedCount);
        if (ok) {
            smoothPath(grid, path, smooth);
        }
    } else if (choice == 3) {
        algorithmName = "Theta*";
        ok = thetaStar(grid, start, goal, path, closedCount);
    } else {
        std::cout << "Неизвестный вариант.\n";
        return;
    }

    if (!ok) {
        std::cout << "Путь не найден. Огонь и заросли блокируют проход.\n";
        return;
    }

    const std::vector<Cell>& finalPath = (choice == 2 ? smooth : path);
    double L_found = pathLength(finalPath);

    if (choice == 2) {
        std::cout << "Исходный путь: " << path.size()
                  << " точек, длина = " << pathLength(path) << "\n";
        std::cout << "После сглаживания: " << smooth.size()
                  << " точек, длина = " << L_found << "\n";
        std::cout << "Количество раскрытых узлов (A*): " << closedCount << "\n\n";
    } else {
        std::cout << "Длина пути (клетки): " << finalPath.size()
                  << ", геометрическая длина = " << L_found << "\n";
        std::cout << "Количество раскрытых узлов: " << closedCount << "\n\n";
    }

    std::cout << "Карта ( '*' — найденный безопасный путь через лес ):\n";
    printMapWithPath(grid, finalPath);

    // CSV для python-визуализации
    saveMapAndPathToCsv(grid, finalPath, "forest_path.csv");

    // Точность эвристики считаем только в одиночном режиме
    double heurErr = computeHeuristicError(grid, goal);

    PathMetrics metrics{};
    fillMetrics(grid, start, goal, finalPath, closedCount, heurErr, metrics);

    std::cout << "\n=== Метрики маршрута ===\n";
    std::cout << "Алгоритм: " << algorithmName << "\n";
    std::cout << "Оптимальная длина пути (Dijkstra): " << metrics.L_opt << "\n";
    std::cout << "Найденная длина пути:            " << metrics.L_found << "\n";
    std::cout << "Коэффициент оптимальности KO:    " << metrics.KO << "\n";
    std::cout << "Отклонение от оптимального OO%:  " << metrics.OO_percent << " %\n";
    std::cout << "Раскрытых узлов:                 " << metrics.closedCount << "\n";
    std::cout << "Эффективность EP:                " << metrics.EP << "\n";
    std::cout << "Фактор ветвления FV:             " << metrics.FV << "\n";
    std::cout << "Суммарный угол поворотов SUP:    " << metrics.SUP << "\n";
    std::cout << "Гладкость пути GP:               " << metrics.GP << "\n";
    std::cout << "Суммарная кривизна пути:         " << metrics.curvature << "\n";
    std::cout << "Мин. расстояние до препятствий:  " << metrics.minObsDist << "\n";
    std::cout << "Сред. расстояние до препятствий: " << metrics.avgObsDist << "\n";
    std::cout << "RMSE эвристики (точность ТЭ):    " << metrics.heurError << "\n";

    saveMetricsToCsv(algorithmName, h, w, metrics, /*runIndex=*/1);
    std::cout << "\nМетрики сохранены в data/csv/metrics.csv\n";
}

// ------------------ СЕРИЯ ЗАПУСКОВ ------------------

void run_batch() {
    int h, w;
    std::cout << "Введите высоту и ширину карт для серии (например, 20 20): ";
    if (!(std::cin >> h >> w)) {
        std::cout << "Ошибка ввода.\n";
        return;
    }
    if (h <= 1 || w <= 1) {
        std::cout << "Размеры должны быть больше 1.\n";
        return;
    }

    int numRuns;
    std::cout << "Сколько разных карт сгенерировать (например, 5): ";
    if (!(std::cin >> numRuns) || numRuns <= 0) {
        std::cout << "Некорректное число, возьмём 5.\n";
        numRuns = 5;
    }

    std::cout << "\n=== Серия запусков: " << numRuns
              << " карт, алгоритмы: A*, A*PS, Theta* ===\n\n";

    try {
        for (int run = 1; run <= numRuns; ++run) {
            std::vector<std::string> grid;
            Cell start;
            Cell goal;

            generateRandomMap(h, w, grid, start, goal);

            std::cout << "---- Карта #" << run << " ----\n";
            for (const auto &row : grid) {
                std::cout << row << "\n";
            }
            std::cout << "\n";

            double baseLen = -1.0; // "оптимальная" длина от A* для этой карты

            for (int alg = 1; alg <= 3; ++alg) {
                std::string algorithmName;
                std::vector<Cell> path;
                std::vector<Cell> smooth;
                int closedCount = 0;
                bool ok = false;

                if (alg == 1) {
                    algorithmName = "A*";
                    ok = aStar(grid, start, goal, path, closedCount);
                } else if (alg == 2) {
                    algorithmName = "A*PS";
                    ok = aStar(grid, start, goal, path, closedCount);
                    if (ok) {
                        smoothPath(grid, path, smooth);
                    }
                } else { // alg == 3
                    algorithmName = "Theta*";
                    ok = thetaStar(grid, start, goal, path, closedCount);
                }

                if (!ok) {
                    std::cout << "  [" << algorithmName << "] путь не найден\n";
                    continue;
                }

                const std::vector<Cell> &finalPath = (alg == 2 ? smooth : path);
                double L_found = pathLength(finalPath);

                PathMetrics m{};
                m.L_found     = L_found;
                m.closedCount = closedCount;

                if (alg == 1) {
                    baseLen      = L_found;
                    m.L_opt      = L_found;
                    m.KO         = 1.0;
                    m.OO_percent = 0.0;
                } else {
                    if (baseLen > 0.0) {
                        m.L_opt      = baseLen;
                        m.KO         = L_found / baseLen;
                        m.OO_percent = (L_found - baseLen) / baseLen * 100.0;
                    } else {
                        m.L_opt      = L_found;
                        m.KO         = 1.0;
                        m.OO_percent = 0.0;
                    }
                }

                if (closedCount > 0) {
                    m.EP = L_found / closedCount;
                }

                // Остальные метрики в серии не считаем
                m.FV         = 0.0;
                m.SUP        = 0.0;
                m.GP         = 0.0;
                m.curvature  = 0.0;
                m.minObsDist = 0.0;
                m.avgObsDist = 0.0;
                m.heurError  = 0.0;

                std::cout << "  [" << algorithmName << "] "
                          << "L_found=" << m.L_found
                          << ", KO=" << m.KO
                          << ", OO%=" << m.OO_percent
                          << ", closed=" << m.closedCount
                          << ", EP=" << m.EP
                          << "\n";

                saveMetricsToCsv(algorithmName, h, w, m, run);
            }

            std::cout << "\n";
        }

        std::cout << "Серия запусков завершена. Все метрики добавлены в data/csv/metrics.csv\n";
    }
    catch (const std::bad_alloc &) {
        std::cout << "ОШИБКА: недостаточно памяти в режиме серии запусков.\n"
                  << "Попробуй уменьшить размеры карты или количество карт.\n";
    }
}
