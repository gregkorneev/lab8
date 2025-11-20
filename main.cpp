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

    if (choice == 1) {
        ok = aStar(grid, start, goal, path, closedCount);
    } else if (choice == 2) {
        ok = aStar(grid, start, goal, path, closedCount);
        if (ok) {
            smoothPath(grid, path, smooth);
        }
    } else if (choice == 3) {
        ok = thetaStar(grid, start, goal, path, closedCount);
    } else {
        std::cout << "Неизвестный вариант.\n";
        return 1;
    }

    if (!ok) {
        std::cout << "Путь не найден. Огонь и заросли блокируют проход.\n";
        return 0;
    }

    if (choice == 2) {
        std::cout << "Исходный путь: " << path.size()
                  << " точек, длина = " << pathLength(path) << "\n";
        std::cout << "После сглаживания: " << smooth.size()
                  << " точек, длина = " << pathLength(smooth) << "\n";
        std::cout << "Количество раскрытых узлов (A*): " << closedCount << "\n\n";
        printMapWithPath(grid, smooth);
    } else {
        std::cout << "Длина пути (клетки): " << path.size()
                  << ", геометрическая длина = " << pathLength(path) << "\n";
        std::cout << "Количество раскрытых узлов: " << closedCount << "\n\n";
        printMapWithPath(grid, path);
    }

    return 0;
}
