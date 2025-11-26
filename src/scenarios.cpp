#include "scenarios.h"
#include "pathfinding.h"
#include <iostream>
#include <stdexcept>

// Вспомогательная функция: ищем S и G в карте
static Scenario makeScenario(const std::string& id,
                             const std::string& title,
                             std::vector<std::string> grid)
{
    Scenario sc;
    sc.id = id;
    sc.title = title;
    sc.grid = std::move(grid);

    const int h = static_cast<int>(sc.grid.size());
    if (h == 0) {
        throw std::runtime_error("Scenario '" + id + "': empty grid");
    }
    const int w = static_cast<int>(sc.grid[0].size());

    bool foundS = false, foundG = false;

    for (int y = 0; y < h; ++y) {
        if (static_cast<int>(sc.grid[y].size()) != w) {
            throw std::runtime_error("Scenario '" + id + "': inconsistent row width");
        }
        for (int x = 0; x < w; ++x) {
            char c = sc.grid[y][x];
            if (c == 'S') {
                sc.start = {y, x};
                foundS = true;
            } else if (c == 'G') {
                sc.goal = {y, x};
                foundG = true;
            }
        }
    }

    if (!foundS) {
        sc.start = {0, 0};
        sc.grid[0][0] = 'S';
    }
    if (!foundG) {
        sc.goal = {h - 1, w - 1};
        sc.grid[h - 1][w - 1] = 'G';
    }

    return sc;
}

std::vector<Scenario> getPredefinedScenarios()
{
    std::vector<Scenario> scenarios;

    // 1. Открытое поле (почти без препятствий)
    scenarios.push_back(makeScenario(
        "open_field",
        "Открытое поле",
        {
            "S........",
            ".........",
            ".........",
            ".........",
            ".......G."
        }
    ));

    // 2. Лабиринт
    scenarios.push_back(makeScenario(
        "maze",
        "Лабиринт с коридорами",
        {
            "S.#.....",
            ".#.#.###",
            ".#.#...#",
            ".###.#.#",
            ".....#G#"
        }
    ));

    // 3. Плотная застройка (много препятствий)
    scenarios.push_back(makeScenario(
        "dense_obstacles",
        "Плотная застройка (много препятствий)",
        {
            "S#.#.#G#",
            ".#.#.#.#",
            "#.#.#.#.",
            ".#.#.#.#",
            "#.#.#.#."
        }
    ));

    // 4. Узкие коридоры
    scenarios.push_back(makeScenario(
        "narrow_corridors",
        "Узкие коридоры",
        {
            "S#......",
            ".#.#.###",
            ".#.#...#",
            ".###.#.#",
            ".....#G#"
        }
    ));

    return scenarios;
}

void run_predefined_scenarios()
{
    auto scenarios = getPredefinedScenarios();

    std::cout << "\n=== Тестирование на заранее заданных картах ===\n\n";

    int scenarioIndex = 1;

    for (const auto& sc : scenarios) {
        const int h = static_cast<int>(sc.grid.size());
        const int w = static_cast<int>(sc.grid[0].size());

        std::cout << "=== Сценарий #" << scenarioIndex << " ===\n";
        std::cout << "ID:    " << sc.id << "\n";
        std::cout << "Название: " << sc.title << "\n";
        std::cout << "Размер: " << h << " x " << w << "\n";
        std::cout << "Карта:\n";
        for (const auto& row : sc.grid) {
            std::cout << row << "\n";
        }
        std::cout << "\n";

        for (int alg = 1; alg <= 3; ++alg) {
            std::string algName;
            std::vector<Cell> path, smooth;
            int closed = 0;
            bool ok = false;

            if (alg == 1) {
                algName = "A*";
                ok = aStar(sc.grid, sc.start, sc.goal, path, closed);
            } else if (alg == 2) {
                algName = "A*PS";
                ok = aStar(sc.grid, sc.start, sc.goal, path, closed);
                if (ok) {
                    smoothPath(sc.grid, path, smooth);
                }
            } else if (alg == 3) {
                algName = "Theta*";
                ok = thetaStar(sc.grid, sc.start, sc.goal, path, closed);
            }

            if (!ok) {
                std::cout << "  [" << algName << "] путь не найден\n";
                continue;
            }

            const auto& finalPath = (alg == 2 ? smooth : path);

            // Метрики
            PathMetrics m{};
            double heurErr = computeHeuristicError(sc.grid, sc.goal);
            fillMetrics(sc.grid, sc.start, sc.goal, finalPath, closed, heurErr, m);

            // Человекочитаемый вывод
            std::cout << "  [" << algName << "] "
                      << "L=" << m.L_found
                      << ", KO=" << m.KO
                      << ", OO%=" << m.OO_percent
                      << ", closed=" << m.closedCount
                      << ", EP=" << m.EP
                      << "\n";

            // Сохранение в общий metrics.csv:
            // имя алгоритма дополняем ID сценария, чтобы легко фильтровать
            std::string fullName = algName + "_" + sc.id;
            saveMetricsToCsv(fullName, h, w, m, scenarioIndex);
        }

        std::cout << "\n";
        ++scenarioIndex;
    }

    std::cout << "Тестирование сценариев завершено. Метрики добавлены в data/csv/metrics.csv\n";
}
