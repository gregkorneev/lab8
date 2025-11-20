#include "grid.h"
#include <iostream>
#include <random>

bool inBounds(int y, int x, int h, int w) {
    return y >= 0 && y < h && x >= 0 && x < w;
}

bool isFree(const std::vector<std::string> &grid, int y, int x) {
    char c = grid[y][x];
    return c == '.' || c == 'S' || c == 'G';
}

// Генерация случайной карты леса при пожаре
void generateRandomMap(int h, int w,
                       std::vector<std::string> &grid,
                       Cell &start, Cell &goal) {
    grid.assign(h, std::string(w, '.'));

    // Старт и цель фиксируем в углах (чтобы точно были на карте)
    start.y = 0;
    start.x = 0;
    goal.y = h - 1;
    goal.x = w - 1;

    grid[start.y][start.x] = 'S';
    grid[goal.y][goal.x] = 'G';

    // Настройки "сложности" карты
    double fireProb = 0.15;   // вероятность очага пожара
    double bushProb = 0.10;   // вероятность непроходимых зарослей
    double blockProb = fireProb + bushProb;

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if ((y == start.y && x == start.x) ||
                (y == goal.y && x == goal.x)) {
                continue; // не блокируем старт и цель
            }

            double r = dist(rng);
            if (r < blockProb) {
                grid[y][x] = '#'; // огонь/заросли
            }
        }
    }

    std::cout << "Сгенерированная карта леса (S - вы, G - безопасная зона, # - огонь/заросли):\n";
    for (int y = 0; y < h; ++y) {
        std::cout << grid[y] << "\n";
    }
    std::cout << "\n";
}

void printMapWithPath(std::vector<std::string> grid,
                      const std::vector<Cell> &path) {
    for (size_t i = 0; i < path.size(); ++i) {
        int y = path[i].y;
        int x = path[i].x;
        if (grid[y][x] == 'S' || grid[y][x] == 'G')
            continue;
        grid[y][x] = '*';
    }

    std::cout << "Карта ( '*' — найденный безопасный путь через лес ):\n";
    for (size_t i = 0; i < grid.size(); ++i) {
        std::cout << grid[i] << "\n";
    }
}
