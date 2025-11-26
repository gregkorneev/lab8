#pragma once

#include <string>
#include <vector>
#include "grid.h"

// Описание одного сценария
struct Scenario {
    std::string id;              // короткий идентификатор (open_field, maze, …)
    std::string title;           // человекочитаемое название
    std::vector<std::string> grid;
    Cell start;
    Cell goal;
};

// Набор заранее заданных сценариев
std::vector<Scenario> getPredefinedScenarios();

// Запуск всех сценариев для трёх алгоритмов (A*, A*PS, Theta*)
void run_predefined_scenarios();
