import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

CSV_FILE = "data/csv/metrics.csv"


def load_scenario_metrics():
    if not os.path.exists(CSV_FILE):
        print("Файл metrics.csv не найден. Сначала запусти режим 3 (сценарии).")
        return None

    df = pd.read_csv(CSV_FILE, header=None)
    df.columns = [
        "run", "alg", "h", "w",
        "L_opt", "L_found", "KO", "OO_percent",
        "closed", "EP", "heurErr",
    ]

    # Берём только строки, где в названии алгоритма есть суффикс сценария: A*_open_field и т.п.
    mask = df["alg"].str.contains("_")
    scen_df = df[mask].copy()
    if scen_df.empty:
        print("В metrics.csv нет данных по сценариям (alg с суффиксом _scenario).")
        return None

    # ⬇️ исправленная строка
    scen_df[["alg_base", "scenario"]] = scen_df["alg"].str.split("_", n=1, expand=True)
    return scen_df



def plot_scenario_bars(df):
    os.makedirs("data/png", exist_ok=True)

    scenario_order = sorted(df["scenario"].unique().tolist())
    algs = sorted(df["alg_base"].unique().tolist())
    n_algs = len(algs)

    # Небольшие сдвиги по X, чтобы бары не слипались
    offsets = {
        alg: (i - (n_algs - 1) / 2) * 0.25
        for i, alg in enumerate(algs)
    }

    scen_name_map = {
        "open_field": "Открытое поле",
        "maze": "Лабиринт",
        "dense_obstacles": "Плотная застройка",
        "narrow_corridors": "Узкие коридоры",
    }

    x = np.arange(len(scenario_order))

    # Удобный помощник: берём значения метрики в нужном порядке сценариев
    def values_for(alg, col):
        vals = []
        sub = df[df["alg_base"] == alg]
        for scen in scenario_order:
            row = sub[sub["scenario"] == scen]
            if row.empty:
                vals.append(np.nan)
            else:
                vals.append(row.iloc[0][col])
        return np.array(vals)

    # 1) Длина пути
    plt.figure(figsize=(10, 6))
    width = 0.25
    for alg in algs:
        vals = values_for(alg, "L_found")
        plt.bar(x + offsets[alg], vals, width=width, label=alg)
    plt.title("Длина найденного пути по сценариям")
    plt.xticks(x, [scen_name_map.get(s, s) for s in scenario_order], rotation=15)
    plt.ylabel("L_found")
    plt.grid(axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig("data/png/scenarios_L_found.png")
    plt.close()

    # 2) Эффективность EP
    plt.figure(figsize=(10, 6))
    for alg in algs:
        vals = values_for(alg, "EP")
        plt.bar(x + offsets[alg], vals, width=width, label=alg)
    plt.title("Эффективность поиска (EP) по сценариям")
    plt.xticks(x, [scen_name_map.get(s, s) for s in scenario_order], rotation=15)
    plt.ylabel("EP")
    plt.grid(axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig("data/png/scenarios_EP.png")
    plt.close()

    # 3) Количество раскрытых узлов
    plt.figure(figsize=(10, 6))
    for alg in algs:
        vals = values_for(alg, "closed")
        plt.bar(x + offsets[alg], vals, width=width, label=alg)
    plt.title("Количество раскрытых узлов по сценариям")
    plt.xticks(x, [scen_name_map.get(s, s) for s in scenario_order], rotation=15)
    plt.ylabel("Closed nodes")
    plt.grid(axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig("data/png/scenarios_closed.png")
    plt.close()

    print("Столбчатые графики сохранены:")
    print(" - data/png/scenarios_L_found.png")
    print(" - data/png/scenarios_EP.png")
    print(" - data/png/scenarios_closed.png")


def get_scenario_grids():
    """
    Карты сценариев — копия того, что в scenarios.cpp.
    """
    return {
        "open_field": [
            "S........",
            ".........",
            ".........",
            ".........",
            ".......G.",
        ],
        "maze": [
            "S.#.....",
            ".#.#.###",
            ".#.#...#",
            ".###.#.#",
            ".....#G#",
        ],
        "dense_obstacles": [
            "S#.#.#G#",
            ".#.#.#.#",
            "#.#.#.#.",
            ".#.#.#.#",
            "#.#.#.#.",
        ],
        "narrow_corridors": [
            "S#......",
            ".#.#.###",
            ".#.#...#",
            ".###.#.#",
            ".....#G#",
        ],
    }


def plot_scenario_maps():
    os.makedirs("data/png", exist_ok=True)
    grids = get_scenario_grids()

    scen_order = ["open_field", "maze", "dense_obstacles", "narrow_corridors"]
    titles = {
        "open_field": "Открытое поле",
        "maze": "Лабиринт",
        "dense_obstacles": "Плотная застройка",
        "narrow_corridors": "Узкие коридоры",
    }

    fig, axes = plt.subplots(1, len(scen_order), figsize=(4 * len(scen_order), 4))

    if len(scen_order) == 1:
        axes = [axes]

    for ax, scen_id in zip(axes, scen_order):
        grid = grids[scen_id]
        h = len(grid)
        w = len(grid[0])
        arr = np.zeros((h, w), dtype=int)

        for y in range(h):
            for x in range(w):
                c = grid[y][x]
                if c == "#":
                    arr[y, x] = 1
                elif c == "S":
                    arr[y, x] = 2
                elif c == "G":
                    arr[y, x] = 3
                else:
                    arr[y, x] = 0

        im = ax.imshow(arr)
        ax.set_title(titles.get(scen_id, scen_id))
        ax.set_xticks([])
        ax.set_yticks([])

        # Подпишем символы поверх клеток для наглядности
        for y in range(h):
            for x in range(w):
                ch = grid[y][x]
                if ch != ".":
                    ax.text(x, y, ch, ha="center", va="center")

    plt.tight_layout()
    plt.savefig("data/png/scenarios_maps.png")
    plt.close()

    print("Карта сценариев сохранена: data/png/scenarios_maps.png")


def main():
    df = load_scenario_metrics()
    if df is None:
        return

    plot_scenario_bars(df)
    plot_scenario_maps()


if __name__ == "__main__":
    main()
