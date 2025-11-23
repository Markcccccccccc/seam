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


def c_bound_calculation(path, vars):
    for var in vars:
        print(f"Calculating C_BOUND for {path} with variable {var}")
        timestamp_path = path + str(var)
        param = var if "channel_num" in path else 6
        _, worst_periods, _, _, _, _ = plc.timestamp_analysis(timestamp_path, param)
        c_bound = cal_ros_bound(worst_periods)
        with open(timestamp_path + "/seam_profile.txt", "w") as f:
            f.write(f"{c_bound} {max(worst_periods)}")

if __name__ == '__main__':
    results_dir = "results"
    experiments = [
        ["src/seam_analysis/"+results_dir+"/channel_num/channel_",[3, 4, 5, 6, 7, 8, 9]],
        ["src/seam_analysis/"+results_dir+"/period/period_",[10, 20, 30, 40, 50]],
        ["src/seam_analysis/"+results_dir+"/J/J_",[0, 2, 4, 6, 8]],
        ["src/seam_analysis/"+results_dir+"/D/D_",[0, 10, 20, 30, 40]],
    ]

    for experiment in experiments:
        timestamp_path = experiment[0]
        vars = experiment[1]
        c_bound_calculation(timestamp_path, vars)
