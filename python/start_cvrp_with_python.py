import os
import subprocess
import multiprocessing
import time

# Define the directory path and command components
directory_path = r"C:\Users\mahu123a\Documents\3l-cvrp"
command_base =  r"C:\Users\mahu123a\Documents\3l-cvrp/build/Release/bin/Release/3L-VehicleRoutingApplication.exe"
output_folder = r"C:\Users\mahu123a\Documents\3l-cvrp/data/output/3l-cvrp/all-constraints-train-data-classifier/krebs/"
input_folder =r"C:\Users\mahu123a\Documents\3l-cvrp/data/input/3l-cvrp/krebs/"
parameter_file = r"C:\Users\mahu123a\Documents\3l-cvrp/data/input/3l-cvrp/parameters/BenchmarkParameters_AllConstraints.json"
number_of_processes = 3

# Navigate to the directorys
os.chdir(directory_path)

def add_tasks(task_queue, folder_path=input_folder):
    # List all files in the folder
    file_list = os.listdir(folder_path)

    for file_name in file_list:
        full_path = os.path.join(folder_path, file_name)
        if not os.path.isfile(full_path):
            continue
        task_queue.put(file_name)

    return task_queue

def run_foster_exe(filename, counter, lock):

    command = f'"{command_base}" -i "{input_folder}" -f "{filename}" -o "{output_folder}" -p "{parameter_file}"'
    
    try:
        #subprocess.run(f"start cmd /c {command} & exit", shell=True, check=True, capture_output=True, cwd = directory_path)
        print(f"Starting: {filename} \n")
        print(f"Command: {command} \n")

        subprocess.run(command, check=True, shell = True, capture_output=True,  cwd = directory_path)
        print(f"Finished: {filename} \n")

        # Warten, bevor der nächste Task gestartet wird
        time.sleep(5)

        # Safely increment the counter
        with lock:
            counter.value += 1

    except subprocess.CalledProcessError as e:
        print(f"Failed: {filename}")
        print(e)

def process_tasks(task_queue, counter, lock):
    while not task_queue.empty():
        try:
            filename = task_queue.get_nowait()
        except:
            break  # In case of race condition
        run_foster_exe(filename, counter, lock)
    return True

def run():
    empty_task_queue = multiprocessing.Queue()
    full_task_queue = add_tasks(empty_task_queue)

    #Create shared counter and lock
    counter = multiprocessing.Value('i', 0)
    lock = multiprocessing.Lock()

    processes = []
    for _ in range(number_of_processes):
        p = multiprocessing.Process(target=process_tasks, args=(full_task_queue,counter, lock))
        processes.append(p)
        p.start()

        time.sleep(10)

    for p in processes:
        p.join()

    print(f"\nTotal subprocesses executed: {counter.value}")

if __name__ == "__main__":
    run()
