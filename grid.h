#ifndef GRID_H
#define GRID_H

#include <vector>
#include <string>

// Клетка на карте
struct Cell {
    int y;
    int x;
};

// Проверка выхода за границы
bool inBounds(int y, int x, int h, int w);

// Проходима ли клетка (нет огня/зарослей)
bool isFree(const std::vector<std::string> &grid, int y, int x);

// Случайная генерация карты леса во время пожара
// S - старт, G - безопасная зона, # - огонь/заросли, . - свободно
void generateRandomMap(int h, int w,
                       std::vector<std::string> &grid,
                       Cell &start, Cell &goal);

// Вывод карты с отмеченным путём (*)
void printMapWithPath(std::vector<std::string> grid,
                      const std::vector<Cell> &path);

#endif // GRID_H
