import time
from numpy import array

from .WLS import wls
from .GTRS import gtrs
from .fileHandlers import _readpathfile


def _main():
    print("##################")
    print("Welcome to Autonav")
    print("##################")
    print("Which algorithm you want to use (Insert the corresponding number)?")
    print("(1) GTRS")
    print("(2) WLS")
    algorithm = input()
    if algorithm == '1' or algorithm == '2':
        default = input("Do you want to use the default anchor number and positions (y/n)?\n")
        if default == 'y' or 'Y':
            B = 200
            N = 8
            a_i = array(
                [
                    [0, 0, 0],
                    [0, B, 0],
                    [B / 2, 0, 0],
                    [B / 2, B, 0],
                    [0, 0, B / 8],
                    [0, B, B / 8],
                    [B / 2, 0, B / 8],
                    [B / 2, B, B / 8],
                ]).T
            K = int(input("Please insert the number of Measurement Samples (K)\n"))
            sigma = float(input("Please insert the noise level in meters (sigma)\n"))
            filename = input("Please input the path to the Waypoints file\n")
            destinations = _readpathfile(filename)
            if algorithm == '1':
                print("Running GTRS...")
                start_time = time.time()
                gtrs(a_i, N, K, sigma, destinations, [10, 10, 5])
                exec_time = (time.time() - start_time)
                print(f"GTRS finished in {exec_time:0,.2f} seconds.")
            elif algorithm == '2':
                print("Running WLS...")
                start_time = time.time()
                wls(a_i, N, K, sigma, destinations, [10, 10, 5])
                exec_time = (time.time() - start_time)
                print(f"WLS finished in {exec_time:0,.2f} seconds.")
        elif default == 'n' or 'N':
            aux = input(
                "Please insert the positions of the anchors in the following format: [x1, y1, z1]; [x2, y2; z2];....")
        else:
            print("Please insert a valid option!")
    else:
        print("Please choose one of the available algorithms!")
