#include "grid.h"
#include <iostream>
#include <random>
#include <fstream>
#include <filesystem>
#include <chrono>

// проверка выхода за границы
bool inBounds(int y, int x, int h, int w) {
    return y >= 0 && y < h && x >= 0 && x < w;
}

// клетка проходима?
bool isFree(const std::vector<std::string> &grid, int y, int x) {
    char c = grid[y][x];
    return c == '.' || c == 'S' || c == 'G';
}

// генерация карты
void generateRandomMap(int h, int w,
                       std::vector<std::string> &grid,
                       Cell &start, Cell &goal)
{
    grid.assign(h, std::string(w, '.'));

    start.y = 0; start.x = 0;
    goal.y  = h - 1; goal.x = w - 1;

    grid[start.y][start.x] = 'S';
    grid[goal.y][goal.x]   = 'G';

    double fireProb  = 0.15;
    double bushProb  = 0.10;
    double blockProb = fireProb + bushProb;

    auto seed = static_cast<unsigned long long>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count()
    );
    std::mt19937 rng(seed);

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (!(y == start.y && x == start.x) &&
                !(y == goal.y  && x == goal.x))
                if (dist(rng) < blockProb)
                    grid[y][x] = '#';
}

void printMapWithPath(std::vector<std::string> grid,
                      const std::vector<Cell> &path)
{
    for (auto &c : path) {
        if (grid[c.y][c.x] == 'S' || grid[c.y][c.x] == 'G') continue;
        grid[c.y][c.x] = '*';
    }

    for (auto &r : grid) std::cout << r << "\n";
}

void saveMapAndPathToCsv(const std::vector<std::string> &grid,
                         const std::vector<Cell> &path,
                         const std::string &filename)
{
    namespace fs = std::filesystem;
    fs::create_directories("data/csv");

    int h = grid.size(), w = grid[0].size();
    std::vector<std::vector<int>> mark(h, std::vector<int>(w, 0));

    for (auto &c : path)
        if (c.y >= 0 && c.y < h && c.x >= 0 && c.x < w)
            mark[c.y][c.x] = 1;

    std::ofstream out("data/csv/" + filename);
    out << "row,col,cell,is_path\n";

    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            out << y << "," << x << "," << grid[y][x] << "," << mark[y][x] << "\n";
}
