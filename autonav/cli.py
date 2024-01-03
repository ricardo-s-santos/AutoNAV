"""This module contains the command line application."""
import time

import typer
from numpy import array, fromstring, insert
from rich import print
from rich.progress import Progress, SpinnerColumn, TextColumn
from rich.prompt import FloatPrompt, IntPrompt, Prompt

from .file_handlers import _readpathfile
from .GTRS import gtrs
from .metrics import compute_armse
from .plots import plot_comparison_between_algorithms, plot_rmse, plot_trajectories
from .WLS import wls

"""
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
    print("[bold blue]© Copelabs - UL CUL [/bold blue]")
    print("[green]###################[/green]")
    algorithm = IntPrompt.ask(
        "Which algorithm you want to use? \n [bold blue](1) GTRS [/bold blue]\n [bold green](2) WLS[/bold green]\n \
        [bold red](3) Both[/bold red]\n"
    )
    default = Prompt.ask("Do you want to use the default anchor number and positions?\n", choices=["y", "n"])
    if default == "y":
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
    else:
        # width = IntPrompt.ask("Please insert the width of the scenario (x,y)")
        # length = IntPrompt.ask("Please insert the length of the scenario (z)")
        n = IntPrompt.ask("Please insert the number of Anchors (N)")
        a_i = []
        for i in range(0, n):
            aux = Prompt.ask(f"Please insert the positions of anchor {i + 1} in the following format: x1, y1, z1")
            # Aqui verificar se as âncoras estão dentro do cenário
            a_i.append(fromstring(aux, dtype=int, sep=","))
        a_i = array(a_i).T
    k = IntPrompt.ask("Please insert the number of Measurement Samples (K)")
    sigma = FloatPrompt.ask("Please insert the noise level in meters (sigma)")
    initial_uav_position = [10, 10, 5]
    filename = Prompt.ask("Please input the path to the Waypoints file")
    destinations = _readpathfile(filename)
    if algorithm == 1:
        start_time = time.time()
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Running GTRS...", total=None)
            trajectories = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
        exec_time = time.time() - start_time
        print(f"GTRS finished in {exec_time:0,.2f} seconds.")
        estimated_trajectory = trajectories[0]
        true_trajectory = trajectories[1]
        # Add initial position of the UAV to the plot
        destinations = insert(destinations, 0, initial_uav_position, axis=0)
        # Compute metrics
        print(f"Average RMSE: {compute_armse(estimated_trajectory, true_trajectory):0,.2f} (m)")
        # Plot trajectories
        plot_trajectories(destinations, estimated_trajectory, a_i)
        # Plot metrics
        plot_rmse(true_trajectory, estimated_trajectory)
    elif algorithm == 2:
        start_time = time.time()
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Running WLS...", total=None)
            trajectories = wls(a_i, n, k, sigma, destinations, initial_uav_position)
        exec_time = time.time() - start_time
        print(f"WLS finished in {exec_time:0,.2f} seconds.")
        estimated_trajectory = trajectories[0]
        true_trajectory = trajectories[1]
        # Add initial position of the UAV to the plot
        destinations = insert(destinations, 0, initial_uav_position, axis=0)
        # Compute metrics
        print(f"Average RMSE: {compute_armse(estimated_trajectory, true_trajectory):0,.2f} (m)")
        # Plot trajectories
        plot_trajectories(destinations, estimated_trajectory, a_i)
        # Plot metrics
        plot_rmse(true_trajectory, estimated_trajectory)
    elif algorithm == 3:
        # GTRS
        start_time = time.time()
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Running GTRS...", total=None)
            trajectories_GTRS = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
        exec_time = time.time() - start_time
        print(f"GTRS finished in {exec_time:0,.2f} seconds.")
        estimated_trajectory_GTRS = trajectories_GTRS[0]
        true_trajectory_GTRS = trajectories_GTRS[1]
        # WLS
        start_time = time.time()
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Running WLS...", total=None)
            trajectories_WLS = wls(a_i, n, k, sigma, destinations, initial_uav_position)
        exec_time = time.time() - start_time
        print(f"WLS finished in {exec_time:0,.2f} seconds.")
        estimated_trajectory_WLS = trajectories_WLS[0]
        true_trajectory_WLS = trajectories_WLS[1]
        # Add initial position of the UAV to the plot
        destinations = insert(destinations, 0, initial_uav_position, axis=0)
        # Compute metrics
        print(f"Average RMSE GTRS: {compute_armse(estimated_trajectory_GTRS, true_trajectory_GTRS):0,.2f} (m)")
        print(f"Average RMSE WLS: {compute_armse(estimated_trajectory_WLS, true_trajectory_WLS):0,.2f} (m)")
        # Plot trajectories
        plot_comparison_between_algorithms(destinations, estimated_trajectory_GTRS, estimated_trajectory_WLS, a_i)
