"""This module contains the command line application."""
import time

import typer
from numpy import array, fromstring, insert
from rich import print

from .file_handlers import _readpathfile
from .GTRS import gtrs
from .metrics import armse
from .plots import plot_trajectories
from .WLS import wls

"""
    Todo: Verify user input
    Ver text line
https://typer.tiangolo.com/tutorial/first-steps/
https://github.com/Textualize/rich
https://rich.readthedocs.io/en/latest/introduction.html#quick-start
"""
app = typer.Typer()


@app.command()
def _main():
    """This is the CLI application function of the AutoNAV package.

    Args:
        None.

    Returns:
        Nothing.
    """
    print("[green]###################[/green]")
    print("[bold red]Welcome to Autonav! [/bold red]")
    print("[bold blue]© Copelabs - UL [/bold blue]")
    print("[green]###################[/green]")
    print("Which algorithm you want to use (Insert the corresponding number)?")
    print("(1) GTRS")
    print("(2) WLS")
    algorithm = input()
    if algorithm == "1" or algorithm == "2":
        default = input("Do you want to use the default anchor number and positions (y/n)?\n")
        if default == "y" or default == "Y":
            b = 200
            n = 8
            a_i = array(
                [
                    [0, 0, 0],
                    [0, b, 0],
                    [b / 2, 0, 0],
                    [b / 2, b, 0],
                    [0, 0, b / 8],
                    [0, b, b / 8],
                    [b / 2, 0, b / 8],
                    [b / 2, b, b / 8],
                ]
            ).T
        elif default == "n" or default == "N":
            n = int(input("Please insert the number of Anchors (N)\n"))
            a_i = []
            for i in range(0, n):
                aux = input(f"Please insert the positions of anchor {i + 1} in the following format: x1, y1, z1\n")
                a_i.append(fromstring(aux, dtype=int, sep=","))
            a_i = array(a_i).T
        else:
            print("Please insert a valid option!")
        k = int(input("Please insert the number of Measurement Samples (K)\n"))
        sigma = float(input("Please insert the noise level in meters (sigma)\n"))
        initial_uav_position = [10, 10, 5]
        filename = input("Please input the path to the Waypoints file\n")
        destinations = _readpathfile(filename)
        if algorithm == "1":
            print("Running GTRS...")
            start_time = time.time()
            trajectories = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
            exec_time = time.time() - start_time
            print(f"GTRS finished in {exec_time:0,.2f} seconds.")
            estimated_trajectory = trajectories[0]
            true_trajectory = trajectories[1]
            # Add initial position of the UAV to the plot
            destinations = insert(destinations, 0, initial_uav_position, axis=0)
            # Compute metrics
            print(f"Average RMSE: {armse(estimated_trajectory, true_trajectory):0,.2f} (m)")
            # Plot trajectories
            plot_trajectories(destinations, estimated_trajectory)
        elif algorithm == "2":
            print("Running WLS...")
            start_time = time.time()
            trajectories = wls(a_i, n, k, sigma, destinations, initial_uav_position)
            exec_time = time.time() - start_time
            print(f"WLS finished in {exec_time:0,.2f} seconds.")
            estimated_trajectory = trajectories[0]
            true_trajectory = trajectories[1]
            # Add initial position of the UAV to the plot
            destinations = insert(destinations, 0, initial_uav_position, axis=0)
            # Compute metrics
            print(f"Average RMSE: {armse(estimated_trajectory, true_trajectory):0,.2f} (m)")
            # Plot trajectories
            plot_trajectories(destinations, estimated_trajectory)
    else:
        print("Please choose one of the available algorithms!")
