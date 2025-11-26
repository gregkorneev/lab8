#include <iostream>
#include <vector>
#include <string>

#include "grid.h"
#include "pathfinding.h"

int main() {
    std::cout << "Лабораторная работа 8.\n";
    std::cout << "Поиск безопасного пути через лес во время пожара.\n\n";

    int h, w;
    std::cout << "Введите высоту и ширину карты (например, 10 15): ";
    if (!(std::cin >> h >> w)) {
        std::cout << "Ошибка ввода.\n";
        return 1;
    }

    if (h <= 1 || w <= 1) {
        std::cout << "Размеры должны быть больше 1.\n";
        return 1;
    }

    std::vector<std::string> grid;
    Cell start;
    Cell goal;

    // Случайная генерация леса с очагами пожара и зарослями
    generateRandomMap(h, w, grid, start, goal);

    std::cout << "Сгенерированная карта леса (S - вы, G - безопасная зона, # - огонь/заросли):\n";
    for (const auto &row : grid) {
        std::cout << row << "\n";
    }
    std::cout << "\n";

    std::cout << "Выберите алгоритм поиска пути:\n";
    std::cout << "1 — A*\n";
    std::cout << "2 — A* с пост-сглаживанием (A*PS)\n";
    std::cout << "3 — Theta*\n";
    int choice;
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
        return 1;
    }

    if (!ok) {
        std::cout << "Путь не найден. Огонь и заросли блокируют проход.\n";
        return 0;
    }

    const std::vector<Cell> &finalPath = (choice == 2 ? smooth : path);
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

    // Сохраняем карту и путь в CSV
    saveMapAndPathToCsv(grid, finalPath, "forest_path.csv");

    // Считаем все метрики по ТЗ
    PathMetrics metrics{};
    fillMetrics(grid, start, goal, finalPath, closedCount, metrics);

    // Выводим метрики в консоль
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

    // Сохраняем метрики в CSV
    saveMetricsToCsv(algorithmName, h, w, metrics);

    std::cout << "\nМетрики сохранены в data/csv/metrics.csv\n";

    return 0;
}
