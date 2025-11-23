# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.ticker import MaxNLocator
# import time
import latency_calculation as plc
import latency_line_chart_plot as lcp

channel_num = 6
channel_index = 0 # Configure to calculate upper bound and observation for different channels [0,5]
c_bound=100

#############################################
# Get results of seam_latency for changed C
#############################################
def C_results():
    passing_latencies = []
    reaction_latencies = []
    c_bounds = [60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140]
    for c in c_bounds:
        path="src/seam_analysis/results/C/C_"+str(c)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index,c)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index,c)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(c_bounds)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed C")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(c_bounds)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {c_bounds[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {c_bounds[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    lcp.plot_passing_latency_different_C(passing_latencies)
    lcp.plot_reaction_latency_different_C(reaction_latencies)

#############################################
# Get results of seam_latency for changed channel_num
#############################################
def channel_num_results():
    passing_latencies = []
    reaction_latencies = []

    channel_nums = [3, 4, 5, 6, 7, 8, 9]
    for n in channel_nums:
        path="src/seam_analysis/results/channel_num/channel_" + str(n)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, n, channel_index,c_bound)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, n, channel_index,c_bound)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(channel_nums)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed channel_num")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(channel_nums)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {channel_nums[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {channel_nums[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    lcp.plot_passing_latency_changed_channel_num(passing_latencies)
    lcp.plot_reaction_latency_changed_channel_num(reaction_latencies)


#############################################
# Get results of seam_latency for lower period
#############################################
def lower_period_results():
    passing_latencies = []
    reaction_latencies = []
    periods = [10, 20, 30, 40, 50]  
    for period in periods:
        path="src/seam_analysis/results/lower_period/period_" + str(period)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index,c_bound)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index,c_bound)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(periods)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed period")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(periods)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {periods[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {periods[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    lcp.plot_passing_latency_changed_period(passing_latencies)
    lcp.plot_reaction_latency_changed_period(reaction_latencies)

#############################################
# Get results of seam_latency for upper period
#############################################
def upper_period_results():
    passing_latencies = []
    reaction_latencies = []
    periods = [60, 70, 80, 90, 100]  
    for period in periods:
        path="src/seam_analysis/results/upper_period/period_" + str(period)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index,c_bound)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index,c_bound)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(periods)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed period")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(periods)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {periods[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {periods[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    lcp.plot_passing_latency_changed_period(passing_latencies)
    lcp.plot_reaction_latency_changed_period(reaction_latencies)

#############################################
# Get results of seam_latency for changed jitter
#############################################
def jitter_results():
    passing_latencies = []
    reaction_latencies = []
    jitters = [0, 2, 4, 6, 8]  
    for jitter in jitters:
        path="src/seam_analysis/results/J/J_" + str(jitter)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index,c_bound)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index,c_bound)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(jitters)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed jitter")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(jitters)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {jitters[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {jitters[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    lcp.plot_passing_latency_changed_jitter(passing_latencies)
    lcp.plot_reaction_latency_changed_jitter(reaction_latencies)

#############################################
# Get results of seam_latency for changed delay
#############################################
def delay_results():
    passing_latencies = []
    reaction_latencies = []
    delays = [0, 10, 20, 30, 40]
    for delay in delays:
        path="src/seam_analysis/results/D/D_" + str(delay)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index,c_bound)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index,c_bound)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(delays)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed delay")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(delays)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {delays[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {delays[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    lcp.plot_passing_latency_changed_delay(passing_latencies)
    lcp.plot_reaction_latency_changed_delay(reaction_latencies)


#############################################
# Get results of seam_latency for different channel
#############################################
def channel_results():
    passing_latencies = []
    reaction_latencies = []
    channels = [1, 2, 3, 4, 5, 6]
    for channel in channels:
        path="src/seam_analysis/results/channel_num/channel_" + str(channel_num)

        passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index,c_bound)
        passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
        reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index,c_bound)
        reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(channels)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])

    print("#### SEAM Latency: changed channel")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(channels)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {channels[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {channels[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")

    lcp.plot_passing_latency_different_channel(passing_latencies)
    lcp.plot_reaction_latency_different_channel(reaction_latencies)


if __name__ == "__main__":
    lower_period_results()
    upper_period_results()