import csv
import os
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

CSV_PATH = "data/csv/forest_path.csv"
OUT_DIR = "data/png"
OUT_PATH = os.path.join(OUT_DIR, "forest_path.png")


def load_grid_from_csv(path):
    rows = []
    cols = []
    cells = []
    path_flags = []

    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for line in reader:
            r = int(line["row"])
            c = int(line["col"])
            cell = line["cell"]
            is_path = int(line["is_path"])

            rows.append(r)
            cols.append(c)
            cells.append(cell)
            path_flags.append(is_path)

    h = max(rows) + 1
    w = max(cols) + 1

    # 0 - свободный лес, 1 - препятствие, 2 - старт, 3 - цель, 4 - путь
    grid_code = [[0 for _ in range(w)] for _ in range(h)]

    for r, c, cell, is_path in zip(rows, cols, cells, path_flags):
        if cell == '#':
            code = 1
        elif cell == 'S':
            code = 2
        elif cell == 'G':
            code = 3
        else:  # '.'
            code = 0

        # если эта клетка входит в путь и это не S/G — помечаем как путь
        if is_path == 1 and cell not in ("S", "G"):
            code = 4

        grid_code[r][c] = code

    return grid_code


def compute_stats(grid_code):
    h = len(grid_code)
    w = len(grid_code[0]) if h > 0 else 0

    obstacles = 0
    path_cells = 0
    start = None
    goal = None

    for i in range(h):
        for j in range(w):
            v = grid_code[i][j]
            if v == 1:
                obstacles += 1
            if v == 4:
                path_cells += 1
            if v == 2:
                start = (i, j)
            if v == 3:
                goal = (i, j)

    # путь в клетках (включая S и G)
    if start is not None and goal is not None:
        path_len_cells = path_cells + 2  # старт и цель
    else:
        path_len_cells = path_cells

    return {
        "h": h,
        "w": w,
        "obstacles": obstacles,
        "path_len_cells": path_len_cells,
    }


def plot_grid(grid_code, out_path):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    stats = compute_stats(grid_code)

    # цвета: фон, препятствия, старт, цель, путь
    cmap = ListedColormap(
        [
            "#e0ffe0",  # 0 - свободный лес
            "#ff6666",  # 1 - огонь/заросли
            "#0066ff",  # 2 - старт
            "#ffd700",  # 3 - цель
            "#ffff00",  # 4 - путь
        ]
    )

    plt.figure(figsize=(6, 6))
    plt.imshow(grid_code, cmap=cmap, origin="upper")

    h = len(grid_code)
    w = len(grid_code[0]) if h > 0 else 0

    # сетка по клеткам
    plt.xticks(range(w))
    plt.yticks(range(h))
    plt.grid(which="both", color="black", linewidth=0.3)
    plt.tick_params(labelsize=6)

    # легенда
    legend_elements = [
        Patch(facecolor="#e0ffe0", edgecolor="black", label="Свободный лес"),
        Patch(facecolor="#ff6666", edgecolor="black", label="Огонь / заросли"),
        Patch(facecolor="#0066ff", edgecolor="black", label="Старт (S)"),
        Patch(facecolor="#ffd700", edgecolor="black", label="Цель (G)"),
        Patch(facecolor="#ffff00", edgecolor="black", label="Безопасный путь"),
    ]
    plt.legend(
        handles=legend_elements,
        loc="upper right",
        fontsize=7,
        framealpha=0.9,
    )

    # подписи S и G поверх клетки
    for i in range(h):
        for j in range(w):
            v = grid_code[i][j]
            if v == 2:
                plt.text(j, i, "S", ha="center", va="center", fontsize=8, fontweight="bold")
            elif v == 3:
                plt.text(j, i, "G", ha="center", va="center", fontsize=8, fontweight="bold")

    # аннотация с краткими характеристиками
    annotation_text = (
        f"Размер карты: {stats['h']} x {stats['w']} клеток\n"
        f"Препятствий (огонь/заросли): {stats['obstacles']}\n"
        f"Длина безопасного пути: {stats['path_len_cells']} клеток"
    )

    plt.gcf().text(
        0.02,
        0.02,
        annotation_text,
        fontsize=7,
        va="bottom",
        ha="left",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    plt.title("Безопасный путь через лес при пожаре")
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Картинка сохранена в {out_path}")


def main():
    if not os.path.exists(CSV_PATH):
        print(f"Файл {CSV_PATH} не найден. Сначала запусти C++ программу.")
        return

    grid_code = load_grid_from_csv(CSV_PATH)
    plot_grid(grid_code, OUT_PATH)


if __name__ == "__main__":
    main()
