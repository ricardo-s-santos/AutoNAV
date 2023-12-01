import csv
from numpy import array


def _readpathfile(filename: str):
    positions = []
    try:
        with open(filename, 'r') as file:
            csv_file = csv.reader(file)
            for line in csv_file:
                positions.append([float(x) for x in line])
    except:
        print("File not found, please check path.")
        exit()
    return array(positions)
