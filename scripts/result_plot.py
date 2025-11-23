import numpy as np
import matplotlib.pyplot as plt
import latency_calculation as plc

channel_num = 6
channel_index = 0 # Configure to calculate upper bound and observation for different channels [0,5]
c_bound=110
results_dir = "results"

def plot_passing_latency(param, xlable, vars, results):
    # 横轴标签
    x = np.array([str(var) for var in vars])
    
    # 提取上界、观测值和差值数组
    upper_bound_array = [r[0] for r in results]
    observed_array = [r[1] for r in results]
    diff_array = [ub - ob for ub, ob in zip(upper_bound_array, observed_array)]

    # 创建图形和主坐标轴
    fig, ax1 = plt.subplots(figsize=(5, 4))

    # 绘制主 y 轴两条曲线：Upper Bound 和 Observed
    ax1.plot(x, upper_bound_array, marker="o", color="#e54d4c",
             markersize=10, linewidth=2, label='UPPER BOUND 1')
    ax1.plot(x, observed_array, marker="d", color="#2b9fc9",
             markersize=10, linewidth=2, label='OBSERVED')
    xtick_labels = [label if i % 1 == 0 else "" for i, label in enumerate(x)]
    ax1.set_xticks(np.arange(len(x)))
    ax1.set_xticklabels(xtick_labels, fontsize=22)
    # 设置主 y 轴属性
    ax1.set_xlabel(xlable, fontsize=22)
    ax1.set_ylabel("Passing Latency", fontsize=22)
    ax1.grid(axis="y", linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', which='major', labelsize=22)
    ax1.set_ylim(bottom=0, top=1.6)

    # # 创建右侧 y 轴并绘制差值曲线
    # ax2 = ax1.twinx()
    # ax2.plot(x, diff_array, marker="^", linestyle='--', color="green",
    #          linewidth=2, label='Difference')
    # ax2.set_ylabel("Latency Difference (x" + r'$10^2$' + " ms)", fontsize=16)
    # ax2.tick_params(axis='y')
    # ax2.set_ylim(bottom=0)

    # 合并两个 y 轴图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    # lines2, labels2 = ax2.get_legend_handles_labels()
    # ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=16, loc='lower right')
    ax1.legend(lines1, labels1, fontsize=18, loc='lower right')

    # 自动调整布局并保存图像
    plt.tight_layout()
    plt.savefig(f'src/seam_analysis/{results_dir}/passing_latency_different_{param}.pdf', format='pdf')


def plot_reaction_latency(param, xlable, vars, results, threshold=6):
    # 横轴标签
    x = np.array([str(var) for var in vars])
    
    # 处理数据：超过阈值的部分截断为阈值
    upper_bound_array = [r[0] if r[0] <= threshold else threshold for r in results]
    observed_array = [r[1] if r[1] <= threshold else threshold for r in results]
    # 差值可根据需要开启
    # diff_array = [ub - ob if ub < threshold else threshold for ub, ob in zip(upper_bound_array, observed_array)]

    # 创建图形和主坐标轴
    fig, ax1 = plt.subplots(figsize=(5, 4))

    # 绘制主 y 轴两条曲线：Upper Bound 和 Observed
    ax1.plot(x, upper_bound_array, marker="o", color="#e54d4c",
             markersize=10, linewidth=2, label='UPPER BOUND 2')
    ax1.plot(x, observed_array, marker="d", color="#2b9fc9",
             markersize=10, linewidth=2, label='OBSERVED')

    # 设置主 x 轴属性（每两个横坐标标签显示一个）
    xtick_labels = [label if i % 1 == 0 else "" for i, label in enumerate(x)]
    ax1.set_xticks(np.arange(len(x)))
    ax1.set_xticklabels(xtick_labels, fontsize=22)
    ax1.set_xlabel(xlable, fontsize=22)

    # 设置 y 轴最大值略高于阈值
    # y_max = threshold * 1.05  # 比阈值高5%
    ax1.set_ylim(0, 5.8)

    # y 轴标签
    ax1.set_ylabel("Reaction Latency", fontsize=22)
    ax1.grid(axis="y", linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', which='major', labelsize=22)

    # 获取当前的 yticks
    yticks = ax1.get_yticks()
    # 将等于阈值的刻度替换成 ∞，其余正常显示（取整）
    if threshold not in yticks:
        yticks = np.append(yticks, threshold)
        yticks = np.sort(yticks)
    yticks = yticks[yticks < 6]
# 替换阈值对应的刻度标签为 ∞
    ytick_labels = [r'$\infty$' if np.isclose(t, threshold) else str(int(t)) for t in yticks]
    # ytick_labels = [r'$\infty$' if np.isclose(t, threshold) or t > threshold else str(int(t)) for t in yticks]
    ax1.set_yticks(yticks)
    ax1.set_yticklabels(ytick_labels, fontsize=22)

    # 合并图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    ax1.legend(lines1, labels1, fontsize=18, loc='lower right')

    # 自动调整布局并保存
    plt.tight_layout()
    plt.savefig(f'src/seam_analysis/{results_dir}/reaction_latency_different_{param}.pdf', format='pdf')

def result_plot(param, xlabel, vars):
    passing_latencies = []
    reaction_latencies = []
    if param == 'c_bound':
        for c in vars:
            path = f"src/seam_analysis/{results_dir}/c_bound/{c}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    elif param == 'channel_num':
        for n in vars:
            path = f"src/seam_analysis/{results_dir}/channel_num/{n}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, n, channel_index, c_bound)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, n, channel_index, c_bound)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    elif param == 'jitter':
        for j in vars:
            path = f"src/seam_analysis/{results_dir}/jitter/{j}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c_bound)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c_bound)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    elif param == 'delay':
        for d in vars:
            path = f"src/seam_analysis/{results_dir}/delay/{d}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c_bound)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c_bound)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    elif param == 'lower_period':
        for p in vars:
            path = f"src/seam_analysis/{results_dir}/lower_period/{p}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c_bound)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c_bound)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    elif param == 'upper_period':
        for p in vars:
            path = f"src/seam_analysis/{results_dir}/upper_period/{p}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c_bound)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c_bound)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    elif param == 'channel':
        for i in vars:
            path = f"src/seam_analysis/{results_dir}/channel/{i}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, i, c_bound)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, i, c_bound)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    # elif param == 'config':
    #     passing_latenct_list = []
    #     reaction_latenct_list = []
    #     for i in vars:
    #         for c in [60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140]:
    #             path = f"src/seam_analysis/{results_dir}/config_{i}/{c}"
    #             passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c)
    #             passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
    #             reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c)
    #             reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
    #         passing_latenct_list.append(passing_latencies)
    #         reaction_latenct_list.append(reaction_latencies)
    #     plot_multiple_passing_latency(param, xlabel, vars, passing_latenct_list, ['60', '65', '70', '75', '80', '85', '90', '95', '100', '105', '110', '115', '120', '125', '130', '135', '140'], ['#e54d4c','#2b9fc9'], results_dir)
    else:
        for c in vars:
            path = f"src/seam_analysis/{results_dir}/{param}/{c}"
            passing_latency_upper_bound, observed_max_passing_latency = plc.passing_latency_upper_bound(path, channel_num, channel_index, c)
            passing_latencies.append([passing_latency_upper_bound * 10, observed_max_passing_latency * 10])
            reaction_latency_upper_bound, observed_max_reaction_latency = plc.reaction_latency_upper_bound(path, channel_num, channel_index, c)
            reaction_latencies.append([reaction_latency_upper_bound * 10, observed_max_reaction_latency * 10])
        print("Invalid parameter. Please choose from 'channel_num', 'lower_period', 'upper_period', 'jitter', 'delay', 'c_bound' or 'channel_index'.")

    passing_latency_upper_bound_array=[]
    observed_max_passing_latency_array=[]
    reaction_latency_upper_bound_array=[]
    observed_max_reaction_latency_array=[]
    for k in range(len(vars)):
        passing_latency_upper_bound_array.append(passing_latencies[k][0])
        observed_max_passing_latency_array.append(passing_latencies[k][1])
        reaction_latency_upper_bound_array.append(reaction_latencies[k][0])
        observed_max_reaction_latency_array.append(reaction_latencies[k][1])
    print(f"#### SEAM Latency: changed {param}")
    print("Passing Latency Upper Bound:", passing_latency_upper_bound_array)
    print("Observed Max Passing Latency: ", observed_max_passing_latency_array)
    print("Reaction Latency Upper Bound:", reaction_latency_upper_bound_array)
    print("Observed Max Reaction Latency: ", observed_max_reaction_latency_array)

    # 简单地逐项比较观察值是否超过上界
    print("\nChecking if observed latency exceeds upper bound:")
    for i in range(len(vars)):
        if observed_max_passing_latency_array[i] > passing_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {vars[i]}] Passing latency exceeded: Observed = {observed_max_passing_latency_array[i]}, UB = {passing_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")
        if observed_max_reaction_latency_array[i] > reaction_latency_upper_bound_array[i]:
            print(f"-------------------------------------------------------------------------------------------------------------------------")
            print(f"[Channel {vars[i]}] Reaction latency exceeded: Observed = {observed_max_reaction_latency_array[i]}, UB = {reaction_latency_upper_bound_array[i]}")
            print(f"-------------------------------------------------------------------------------------------------------------------------")


    plot_passing_latency(param, xlabel, vars, passing_latencies)
    plot_reaction_latency(param, xlabel, vars, reaction_latencies, threshold=5.6)
channel_num = 6
if __name__ == "__main__":

    experiments = [
        ['c_bound', "Time Disparity Bound " + r'$\mathrm{C}$', 
            [60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140]],

        ['channel_num', "Number of Channels " + r'$\mathrm{N}$', 
            [3, 4, 5, 6, 7, 8, 9]],

        ['jitter', "Jitter " + r'$\mathrm{T}_\mathrm{i}^\mathrm{W} / \mathrm{T}_\mathrm{i}^\mathrm{B}$', 
            [1.0, 1.2, 1.4, 1.6, 1.8]],

        ['delay', "Random Delay " + r'$\mathrm{D}_\mathrm{i}^\mathrm{W}$', 
            [0, 10, 20, 30, 40]],

        ['lower_period', "Lower Period " + r'$\mathrm{T}_\mathrm{i}^\mathrm{B}$', 
            [10, 20, 30, 40, 50]],

        ['channel', "Index of Channel", 
            [1, 2, 3, 4, 5, 6]]
    ]
    # experiments = [
        # ['c_bound', "" + r'$C$' + "",[32, 63, 94, 126, 157]],
        # ['c_bound',"Time Disparity Bound" + "" + r'$C$' + "",[60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140]],
        # ['channel_num', "Number of Channels" + "" + r'$N$' + "",[3, 4, 5, 6, 7, 8, 9]],
        # ['jitter',"Jitter" + ""+r'$T_i^W/T_i^B$'+"",[1.0, 1.2, 1.4, 1.6, 1.8]],
        # ['delay', "Ramdom Delay" + ""+r'$D_i^W$'+"",[0, 10, 20, 30, 40]],
        # ['lower_period', "Lower Period"+ ""+r'$T_i^B$'+"",[10, 20, 30, 40, 50]],
        # # # ['upper_period', 'Upper Period',[80, 90, 100, 110, 120, 130, 140, 150]],
        # ['channel', 'Index of Channel',[1, 2, 3, 4, 5, 6]],
        # ['config_2_N4_T10070_J2', "" + r'$C$' + "",[62, 81 ,100, 120, 139]],
        # ['config_2', "" + r'$C$' + "",[25, 53, 75, 100, 125]],
        # ['config_3_N3_T100_J2', "" + r'$C$' + "",[30, 60 , 90, 120, 150]],
        # ['config_11', "" + r'$C$' + "",[32, 62 ,92, 123, 153]],
        # ['config_5', "" + r'$C$' + "",[32, 63 ,94, 126, 157]],
        # ['config_6', "" + r'$C$' + "",[55, 66, 77, 88, 99]],
    # ]

    for experiment in experiments:
        param = experiment[0]
        xlabel = experiment[1]
        vars = experiment[2]
        result_plot(param, xlabel, vars)