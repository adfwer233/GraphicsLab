import glob
import os
import subprocess
import threading

print("running clang format")

file_extends = ["*.hpp", "*.cpp", "*.cuh", "*.cu", "*.hpp.impl"]

target_dirs = [os.path.join(".", "src"), os.path.join(".", "include")]

source_file_list = []

for dir in target_dirs:
    for extend in file_extends:
        res = glob.glob(os.path.join(dir, "**", extend), recursive=True)
        source_file_list += res


def run_clang_format(file_list):
    for file_path in file_list:
        print(file_list)
        subprocess.call(["clang-format", "-i", file_path, "--style=file"])


def split_list_generator(listTemp, n):
    for i in range(0, len(listTemp), n):
        yield listTemp[i:i + n]


def run_clang_format_dispatcher(total_list):
    ts = []
    for cur_file_list in split_list_generator(total_list, 10):
        thread = threading.Thread(target=run_clang_format, args=(cur_file_list,))
        ts.append(thread)
        thread.start()

    for t in ts:
        t.join()

print(source_file_list)

run_clang_format_dispatcher(source_file_list)