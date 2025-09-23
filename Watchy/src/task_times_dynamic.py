import random
import pandas as pd
import json

# Definicije taskova (kolone) i komponenti (vrste)
tasks = ["Task1", "Task2", "Task3", "Rbt"]
components = ["Hrdwr", "Sftwr", "Frmwr"]

# Generiši nasumična vremena (1–10 ms) za svaku komponentu i task
task_times = {
    comp: {task: random.randint(1, 10) for task in tasks}
    for comp in components
}

# Sačuvaj kao Excel (opciono)
df = pd.DataFrame(task_times).T
df.to_excel("task_data.xlsx")

# Sačuvaj i kao .json fajl za Watchy
def save_to_json(data, filename="task_data.json"):
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)

# Poziv funkcije
save_to_json(task_times)

print("✅ task_data.xlsx i task_data.json su uspešno generisani!")

def save_json_as_header(data, filename="task_data_json.h"):
    with open(filename, "w") as f:
        f.write("#pragma once\n\n")
        f.write('const char json_task_data[] PROGMEM = R"====(\n')
        json.dump(data, f, indent=2)
        f.write('\n)====";\n')


#poziv
save_json_as_header(task_times)
