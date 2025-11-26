#include "grid.h"
#include <iostream>
#include <random>
#include <fstream>
#include <filesystem>

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

    // Старт и цель в углах
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
    for (const auto &c : path) {
        int y = c.y;
        int x = c.x;
        if (grid[y][x] == 'S' || grid[y][x] == 'G')
            continue;
        grid[y][x] = '*';
    }

    std::cout << "Карта ( '*' — найденный безопасный путь через лес ):\n";
    for (size_t i = 0; i < grid.size(); ++i) {
        std::cout << grid[i] << "\n";
    }
}

// Сохранение карты и пути в CSV
void saveMapAndPathToCsv(const std::vector<std::string> &grid,
                         const std::vector<Cell> &path,
                         const std::string &filename)
{
    namespace fs = std::filesystem;

    int h = (int)grid.size();
    if (h == 0) return;
    int w = (int)grid[0].size();

    // Помечаем клетки, которые лежат на пути
    std::vector<std::vector<int>> onPath(h, std::vector<int>(w, 0));
    for (const auto &c : path) {
        if (c.y >= 0 && c.y < h && c.x >= 0 && c.x < w) {
            onPath[c.y][c.x] = 1;
        }
    }

    // Создаём каталоги data/csv
    fs::create_directories("data/csv");

    std::string fullName = "data/csv/" + filename;
    std::ofstream out(fullName);
    if (!out.is_open()) {
        std::cerr << "Не удалось открыть файл для записи: " << fullName << "\n";
        return;
    }

    // Заголовок
    out << "row,col,cell,is_path\n";

    // Строки таблицы
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            char cell = grid[y][x]; // '.', '#', 'S', 'G'
            int isP = onPath[y][x]; // 0 или 1
            out << y << "," << x << "," << cell << "," << isP << "\n";
        }
    }

    std::cout << "\nРезультат сохранён в CSV: " << fullName << "\n";
}
