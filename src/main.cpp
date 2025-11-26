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
    std::cin >> h >> w;

    if (h <= 1 || w <= 1) {
        std::cout << "Размеры должны быть больше 1.\n";
        return;
    }

    std::vector<std::string> grid;
    Cell start, goal;

    generateRandomMap(h, w, grid, start, goal);

    std::cout << "\nСгенерированная карта:\n";
    for (auto &r : grid) std::cout << r << "\n";
    std::cout << "\n";

    std::cout << "Выберите алгоритм:\n";
    std::cout << "1 — A*\n";
    std::cout << "2 — A*PS\n";
    std::cout << "3 — Theta*\n";

    int choice;
    std::cin >> choice;

    std::vector<Cell> path, smooth;
    int closed = 0;
    bool ok = false;
    std::string name;

    if (choice == 1) {
        name = "A*";
        ok = aStar(grid, start, goal, path, closed);
    } else if (choice == 2) {
        name = "A*PS";
        ok = aStar(grid, start, goal, path, closed);
        if (ok) smoothPath(grid, path, smooth);
    } else if (choice == 3) {
        name = "Theta*";
        ok = thetaStar(grid, start, goal, path, closed);
    }

    if (!ok) {
        std::cout << "Путь не найден.\n";
        return;
    }

    const auto &finalPath = (choice == 2 ? smooth : path);

    std::cout << "Карта с путём:\n";
    printMapWithPath(grid, finalPath);

    saveMapAndPathToCsv(grid, finalPath, "forest_path.csv");

    double heurErr = computeHeuristicError(grid, goal);

    PathMetrics m{};
    fillMetrics(grid, start, goal, finalPath, closed, heurErr, m);

    std::cout << "\n=== Метрики ===\n";
    std::cout << "L_opt=" << m.L_opt << "\n";
    std::cout << "L_found=" << m.L_found << "\n";
    std::cout << "KO=" << m.KO << "\n";
    std::cout << "OO%=" << m.OO_percent << "\n";
    std::cout << "Closed=" << m.closedCount << "\n";
    std::cout << "EP=" << m.EP << "\n";

    saveMetricsToCsv(name, h, w, m, 1);
}

// ------------------ СЕРИЯ ЗАПУСКОВ ------------------

void run_batch() {
    int h, w;
    std::cout << "Введите высоту и ширину карт: ";
    std::cin >> h >> w;

    int numRuns;
    std::cout << "Сколько разных карт сгенерировать: ";
    std::cin >> numRuns;

    std::cout << "\n=== Серия запусков (" << numRuns << " карт) ===\n\n";

    for (int run = 1; run <= numRuns; ++run) {

        std::vector<std::string> grid;
        Cell start, goal;

        generateRandomMap(h, w, grid, start, goal);

        std::cout << "---- Карта #" << run << " ----\n";
        for (auto &r : grid) std::cout << r << "\n";
        std::cout << "\n";

        for (int alg = 1; alg <= 3; ++alg) {
            std::string name;
            std::vector<Cell> path, smooth;
            int closed = 0;
            bool ok = false;

            if (alg == 1) { name = "A*"; ok = aStar(grid, start, goal, path, closed); }
            if (alg == 2) { name = "A*PS"; ok = aStar(grid, start, goal, path, closed); if (ok) smoothPath(grid, path, smooth); }
            if (alg == 3) { name = "Theta*"; ok = thetaStar(grid, start, goal, path, closed); }

            if (!ok) {
                std::cout << "  [" << name << "] путь не найден\n";
                continue;
            }

            const auto &finalPath = (alg == 2 ? smooth : path);

            PathMetrics m{};
            double heurErr = 0.0; // В СЕРИИ НЕ СЧИТАЕМ RMSE — ТОЛЬКО ОСНОВНЫЕ МЕТРИКИ
            fillMetrics(grid, start, goal, finalPath, closed, heurErr, m);

            std::cout << "  [" << name << "] "
                      << "L=" << m.L_found
                      << ", KO=" << m.KO
                      << ", OO%=" << m.OO_percent
                      << ", closed=" << m.closedCount
                      << ", EP=" << m.EP
                      << "\n";

            saveMetricsToCsv(name, h, w, m, run);
        }

        std::cout << "\n";
    }

    std::cout << "Серия завершена. Метрики сохранены.\n";
}
