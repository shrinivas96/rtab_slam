import os

projectDir = r"/home/ivengar/workspace/rtab_slam/"

# get the list of files in the current directory
files = os.listdir(projectDir)

# filter the files with .dat extension
dat_files = [projectDir + file for file in files if file.endswith(".dat")]

file_times = {}

# print the list of .dat files
print("List of .dat files:")
for file in dat_files:
    count = 0
    total_time = 0
    with open(file) as whole:
        for line in whole:
            count += 1
            total_time += float(line.strip().split()[-1])
    file_times[file] = total_time/count
print(file_times)

sortedFT = dict(sorted(file_times.items(), key=lambda item: item[1]))

print(sortedFT)

for key in sortedFT.values():
    print(key)