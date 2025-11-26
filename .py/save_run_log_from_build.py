import os
from datetime import datetime

RAW_LOG_PATH = "build/run_output.txt"  # откуда берем сырой вывод
LOG_DIR = "data"
LOG_PATH = os.path.join(LOG_DIR, "run_log.md")


def main():
    if not os.path.exists(RAW_LOG_PATH):
        print(f"Файл {RAW_LOG_PATH} не найден. Сначала запусти ./build/lab8 c перенаправлением вывода.")
        print(f"Например:\n  ./build/lab8 | tee {RAW_LOG_PATH}")
        return

    with open(RAW_LOG_PATH, "r", encoding="utf-8") as f:
        output = f.read()

    if not output.strip():
        print(f"Файл {RAW_LOG_PATH} пустой, нечего сохранять.")
        return

    os.makedirs(LOG_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Если файла ещё нет — пишем заголовок
    header = "# Лог запусков lab8\n\n" if not os.path.exists(LOG_PATH) else ""
    block_header = f"\n\n## Запуск от {timestamp}\n\n```text\n"
    block_footer = "\n```\n"

    with open(LOG_PATH, "a", encoding="utf-8") as f:
        if header:
            f.write(header)
        f.write(block_header)
        f.write(output)
        f.write(block_footer)

    print(f"Лог из {RAW_LOG_PATH} добавлен в {LOG_PATH}")


if __name__ == "__main__":
    main()
