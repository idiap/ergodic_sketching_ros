# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import pathlib
import argparse

import numpy
import rich.console
import rich.table
import rich.progress
import matplotlib.pyplot as plt

def display_action_message(console: rich.console.Console)->str:
    console.print("[bold] What to do next?")
    console.print("\t * [italic]h[/italic] display this message")
    console.print("\t * [italic]t[/italic] display the experiment table")
    console.print("\t * [italic]q[/italic] terminate this analyze")
    console.print("\t * [italic]Idx[/italic] Investigate further a given experiment")
    console.print("[bold] >")

def get_scalar_txt(val:float,max_val:float,min_val:float=0):
    color = "green" if val > min_val and val < max_val else "red"
    return f"[{color}]{val:.3f}[/{color}]  "

def print_array_info(console:rich.console.Console,array:numpy.ndarray,max_limit,min_limit):

    max_val = array.max(axis=0)

    if type(max_val) is numpy.float64:
        console.print(f"\t * max: {get_scalar_txt(max_val,max_limit,min_limit)}")
    else:
        msg = "\t * max: ["
        for i,val in enumerate(max_val):
            msg += get_scalar_txt(val,max_limit[i],min_limit[i])
        msg += "]"
        console.print(msg)

    min_val = array.min(axis=0)

    if type(min_val) is numpy.float64:
        console.print(f"\t * min: {get_scalar_txt(min_val,max_limit,min_limit)}")
    else:
        msg = "\t * min: ["
        for i,val in enumerate(min_val):
            msg += get_scalar_txt(val,max_limit[i],min_limit[i])
        msg += "]"
        console.print(msg)

    mean_val = array.mean(axis=0)

    if type(mean_val) is numpy.float64:
        console.print(f"\t * mean: {get_scalar_txt(mean_val,max_limit,min_limit)}")
    else:
        msg = "\t * mean: ["
        for i,val in enumerate(mean_val):
            msg += get_scalar_txt(val,max_limit[i],min_limit[i])
        msg += "]"
        console.print(msg)

    std_val = array.std(axis=0)
    console.print(f"\t * std: {std_val}")

    num_quantile = 10
    issue_quantile = [False] * num_quantile
    for i,elem in enumerate(array):
        max_issue = numpy.greater_equal(elem,max_limit).any()
        min_issue = numpy.less_equal(elem,min_limit).any()
        if max_issue or min_issue:
            issue_idx = int(i/array.shape[0] * 10)
            issue_quantile[issue_idx] = True

    msg = "\t * where [bold]["
    msg += " ".join([ "[red]*[/red]" if issue else "[green]*[/green]" for issue in issue_quantile ])
    msg += "]"
    console.print(msg)

def invest_experiment(exp_id: int, exp_info: dict, qmax:list[float], qmin:list[float],dqmax:list[float],cost_max:float,console:rich.console.Console):
    console.print(f"[bold][underline]Experiment {exp_id}")
    console.print("[italic] Joint positions")
    print_array_info(console,exp_info["q"],qmax,qmin)
    console.print("[italic] Joint velocities")
    print_array_info(console,exp_info["dq"],dqmax,-1*dqmax)
    console.print("[italic] Cost")
    print_array_info(console,exp_info["cost"],cost_max,0)

def analyze():
    parser = argparse.ArgumentParser()
    parser.add_argument("-l","--logdir",dest="log_dir",type=str, default="./ergodic_sketching_ros/ergodic_sketching_ros/scripts/planner_second_order_log/", help="log dir")
    parser.add_argument("--qmax",dest="qmax",nargs="+", type=list, default=[2.961,2.091,2.961,2.091,2.961,2.901,3.001],help="joint position upper limit")
    parser.add_argument("--qmin",dest="qmin",nargs="+", type=list, default=[-2.961,-2.091,-2.961,-2.091,-2.961,-2.901,-3.001],help="joint position lower limit")
    parser.add_argument("--dqmax",dest="dqmax",nargs="+", type=list, default=[1,1,1,1,1,1,1],help="command limit")
    parser.add_argument("--cost-max",dest="cost_max",type=float, default=0.0005, help="maximum accepted value for cost")
    args = parser.parse_args()

    log_dir = pathlib.Path(args.log_dir)
    qmax = numpy.array(args.qmax)
    qmin = numpy.array(args.qmin)
    dqmax = numpy.array(args.dqmax)
    cost_max = args.cost_max

    if not log_dir.exists():
        print(f"Specified log dir ({str(log_dir.resolve())}) does not exist")
        return

    console = rich.console.Console()
    console.print(f"[bold] Analyzing {str(log_dir)}")

    experiment_table = rich.table.Table(title="Experiments overview")
    experiment_table.add_column("Idx", style="cyan", no_wrap=True)
    experiment_table.add_column("Name")
    experiment_table.add_column("Joint positions")
    experiment_table.add_column("Joint velocities")
    experiment_table.add_column("Cost")
    experiment_table.add_column("overall")

    experiments_info = []
    exp_idx = 0

    for experiment_dir in rich.progress.track(log_dir.glob("*/"),description="[green] Gathering experiment informations"):
        if experiment_dir.is_file():
            continue

        exp_name = experiment_dir.stem

        exp_info = dict()
        exp_info["q"] = numpy.loadtxt(str(experiment_dir/"joint_positions.csv"),delimiter=",")
        exp_info["dq"] = numpy.loadtxt(str(experiment_dir/"joint_velocities.csv"),delimiter=",")
        exp_info["cost"] = numpy.loadtxt(str(experiment_dir/"costs.csv"),delimiter=",")
        experiments_info += [exp_info]

        qmax_ok = not numpy.greater_equal(exp_info["q"],qmax).any()
        qmin_ok = not numpy.less_equal(exp_info["q"],qmin).any()
        q_ok = "✅" if qmax_ok and qmin_ok else "❌"
        dq_ok = "✅" if not numpy.greater_equal(numpy.abs(exp_info["dq"]),dqmax).any() else "❌"
        cost_ok = "✅" if not numpy.greater_equal(exp_info["cost"],cost_max).any() else "❌"
        overall_ok = "✅" if "❌" not in [q_ok,dq_ok,cost_ok] else "❌"

        experiment_table.add_row(str(exp_idx),exp_name,q_ok,dq_ok,cost_ok,overall_ok)

        exp_idx += 1

    console.print(experiment_table)

    action = "h"

    while action!="q":

        if action=="h":
            display_action_message(console)

        if action=="t":
            console.print(experiment_table)

        if action.isnumeric():
            exp_id = int(action)
            if exp_id >= exp_idx:
                console.print(f"[bold] Experiment number should be between 0 and {exp_idx-1}")
                continue

            invest_experiment(exp_id,experiments_info[exp_id],qmax,qmin,dqmax,cost_max,console)

        action = input()

if __name__ == "__main__":
    analyze()
