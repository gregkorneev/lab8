import pandas as pd
import matplotlib.pyplot as plt
import os

CSV_FILE = "data/csv/metrics.csv"


def main():
    if not os.path.exists(CSV_FILE):
        print("Файл metrics.csv не найден. Сначала запустите режим 2 (серия).")
        return

    df = pd.read_csv(CSV_FILE, header=None)
    df.columns = [
        "run", "alg", "h", "w",
        "L_opt", "L_found", "KO", "OO_percent",
        "closed", "EP", "heurErr"
    ]

    if df.empty:
        print("В metrics.csv нет данных.")
        return

    print("Загружено строк:", len(df))

    os.makedirs("data/png", exist_ok=True)

    # Упорядочим алгоритмы, чтобы сдвиги по X были стабильными
    algs = sorted(df["alg"].unique().tolist())
    n_algs = len(algs)

    # Сдвиги по X, чтобы точки не лежали друг на друге
    # Например, для 3 алгоритмов: -0.15, 0, +0.15
    if n_algs > 1:
        offsets = {
            alg: (i - (n_algs - 1) / 2) * 0.15
            for i, alg in enumerate(algs)
        }
    else:
        offsets = {algs[0]: 0.0}

    # Диапазон карт для аккуратных тиков
    runs = sorted(df["run"].unique())
    x_min, x_max = min(runs) - 0.5, max(runs) + 0.5

    # 1. Длина пути по картам
    plt.figure(figsize=(10, 6))
    for alg in algs:
        sub = df[df["alg"] == alg]
        x = sub["run"] + offsets[alg]
        plt.plot(x, sub["L_found"], marker="o", linestyle="-", label=alg)
    plt.title("Длина найденного пути по картам")
    plt.xlabel("Номер карты")
    plt.ylabel("L_found")
    plt.xlim(x_min, x_max)
    plt.xticks(runs)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("data/png/series_L_found.png")
    plt.close()

    # 2. EP по картам
    plt.figure(figsize=(10, 6))
    for alg in algs:
        sub = df[df["alg"] == alg]
        x = sub["run"] + offsets[alg]
        plt.plot(x, sub["EP"], marker="o", linestyle="-", label=alg)
    plt.title("Эффективность (EP) по картам")
    plt.xlabel("Номер карты")
    plt.ylabel("EP")
    plt.xlim(x_min, x_max)
    plt.xticks(runs)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("data/png/series_EP.png")
    plt.close()

    # 3. Кол-во раскрытых узлов по картам
    plt.figure(figsize=(10, 6))
    for alg in algs:
        sub = df[df["alg"] == alg]
        x = sub["run"] + offsets[alg]
        plt.plot(x, sub["closed"], marker="o", linestyle="-", label=alg)
    plt.title("Количество раскрытых узлов по картам")
    plt.xlabel("Номер карты")
    plt.ylabel("Closed nodes")
    plt.xlim(x_min, x_max)
    plt.xticks(runs)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("data/png/series_closed.png")
    plt.close()

    # 4. Сводка по алгоритмам: успехи / средние значения

    summary = df.groupby("alg").agg(
        runs_count=("run", "count"),
        L_mean=("L_found", "mean"),
        closed_mean=("closed", "mean"),
        EP_mean=("EP", "mean")
    ).reset_index()

    # 4.1. Количество успешных запусков (сколько раз алгоритм вообще нашёл путь)
    plt.figure(figsize=(8, 5))
    plt.bar(summary["alg"], summary["runs_count"])
    plt.title("Количество успешных запусков по алгоритмам")
    plt.ylabel("Число успешных карт")
    plt.tight_layout()
    plt.savefig("data/png/series_success_counts.png")
    plt.close()

    # 4.2. Средняя длина пути
    plt.figure(figsize=(8, 5))
    plt.bar(summary["alg"], summary["L_mean"])
    plt.title("Средняя длина пути по алгоритмам")
    plt.ylabel("L_found (среднее)")
    plt.tight_layout()
    plt.savefig("data/png/series_L_mean.png")
    plt.close()

    # 4.3. Среднее число раскрытых узлов
    plt.figure(figsize=(8, 5))
    plt.bar(summary["alg"], summary["closed_mean"])
    plt.title("Среднее число раскрытых узлов по алгоритмам")
    plt.ylabel("Closed nodes (среднее)")
    plt.tight_layout()
    plt.savefig("data/png/series_closed_mean.png")
    plt.close()

    print("Графики сохранены в data/png/:")
    print(" - series_L_found.png")
    print(" - series_EP.png")
    print(" - series_closed.png")
    print(" - series_success_counts.png")
    print(" - series_L_mean.png")
    print(" - series_closed_mean.png")


if __name__ == "__main__":
    main()
