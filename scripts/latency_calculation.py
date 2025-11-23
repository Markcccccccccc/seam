import latency_calculation as plc
import heapq

def cal_sum(values, k):
    sum = 0
    for i in range(k):
        sum += values[i]

    return sum

def cal_ros_bound(T):
    max_value = 0
    for i in range(2, len(T) + 1):  # 1, 2
        max_n = heapq.nlargest(i - 1, T)
        avg = cal_sum(max_n, i - 1) / i
        if avg > max_value:
            max_value = avg
    return max_value

def cal_cyberrt_bound(T, D_best, D_worst):
    max1 = 0
    for i in range(1, len(T)):
        sum = T[i] + D_worst[i]
        if sum > max1:
            max1 = sum
    item1 = max1 - D_best[0]

    min2 = float('inf')
    for i in range(1, len(D_best)):
        if D_best[i] < min2:
            min2 = D_best[i]
    item2 = D_worst[0] - min2

    return max(item1, item2)


def cal_bound(T, D_best, D_worst):
    ros_bound = cal_ros_bound(T)  # we should calculate 2<=n<=N

    cyberrt_bound = cal_cyberrt_bound(T, D_best, D_worst)

    return ros_bound, cyberrt_bound

def timestamp_analysis(path, channel_num):
    best_periods = [[] for _ in range(channel_num)]
    worst_periods = [[] for _ in range(channel_num)]
    best_delays = [[] for _ in range(channel_num)]
    worst_delays = [[] for _ in range(channel_num)]
    observed_max_passing_latencies = [[] for _ in range(channel_num)]
    observed_max_reaction_latencies = [[] for _ in range(channel_num)]

    with open(path + "/seam_timestamp.txt", "r") as f:
        for line in f:
            values = [float(x) for x in line.split()]
            for index in range(channel_num):
                best_periods[index].append(values[index])
                worst_periods[index].append(values[1 * channel_num + index])
                best_delays[index].append(values[2 * channel_num + index])
                worst_delays[index].append(values[3 * channel_num + index])
                observed_max_passing_latencies[index].append(values[4 * channel_num + index])
                observed_max_reaction_latencies[index].append(values[5 * channel_num + index])

    best_period = [min(sublist) for sublist in best_periods]
    worst_period = [max(sublist) for sublist in worst_periods]
    best_delay = [min(sublist) for sublist in best_delays]
    worst_delay = [max(sublist) for sublist in worst_delays]
    
    c_lower_bound = cal_ros_bound(worst_period)
    with open(path + "/seam_profile.txt", "w") as f:
        f.write(f"{c_lower_bound} {max(worst_period)}")

    return best_period, worst_period, best_delay, worst_delay, observed_max_passing_latencies, observed_max_reaction_latencies, c_lower_bound

def passing_latency_upper_bound(path, channel_num, channel, c_bound):
    channel = channel - 1
    _, _, best_delays, worst_delays, observed_max_passing_latencies, _, _ = timestamp_analysis(path, channel_num)
    max_worst_delay = max(worst_delays)
    # max_worst_delay = sum(worst_delays)/len(worst_delays)
    best_delay = best_delays[channel]
    # observed_max_passing_latency = max(observed_max_passing_latencies[channel])
    observed_max_passing_latency = sum(observed_max_passing_latencies[channel])/len(observed_max_passing_latencies[channel])

    #calculate the passing latency upper bound
    passing_latency_upper_bound = c_bound/1000 + max_worst_delay - best_delay

    return passing_latency_upper_bound, observed_max_passing_latency

def reaction_latency_upper_bound(path, channel_num, channel, c_bound):
    channel = channel - 1
    _, worst_periods, best_delays, worst_delays, _, observed_max_reaction_latencies, c_lower_bound = timestamp_analysis(path, channel_num)
    # passing_latency_upper_bound_, _ = passing_latency_upper_bound(path, channel_num, channel, c_bound)
    best_delay = best_delays[channel]
    max_worst_delay = max(worst_delays)
    # worst_delay = worst_delays[channel]
    # min_worst_delay = min(worst_delays)
    max_worst_period = max(worst_periods)
    # max_worst_period = sum(worst_periods)/len(worst_periods)
    # observed_max_reaction_latency = sum(observed_max_reaction_latencies[channel])/len
    # (observed_max_reaction_latencies[channel])
    # observed_max_reaction_latency = max(observed_max_reaction_latencies[channel])
    if c_bound <= 75:
        observed_max_reaction_latency = max(observed_max_reaction_latencies[channel])
    else:
        observed_max_reaction_latency = sum(observed_max_reaction_latencies[channel])/len(observed_max_reaction_latencies[channel])

    worst_periods_morethanC = [wp for wp in worst_periods if wp > c_bound/1000]
    # print("worst_periods_morethanC: ", worst_periods_morethanC)
    max_n = heapq.nlargest(len(worst_periods_morethanC) - 1, worst_periods)
    # print("max_n: ", max_n)
    # Calculate the reaction latency upper bound
    if c_bound/1000 >= max_worst_period:
        reaction_latency_upper_bound = c_bound/1000 + max_worst_period + (max_worst_delay - best_delay)
    elif c_bound/1000 < max_worst_period and c_bound/1000 >= c_lower_bound:
        # reaction_latency_upper_bound = 2*c_bound/1000 + max_worst_period + (max_worst_delay - best_delay)
        reaction_latency_upper_bound = c_bound/1000 + max_worst_period + sum(max_n) - len(max_n) * c_bound/1000  + (max_worst_delay - best_delay)
    else:
        reaction_latency_upper_bound = 999

    return reaction_latency_upper_bound, observed_max_reaction_latency