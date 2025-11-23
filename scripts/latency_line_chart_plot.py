import numpy as np
import matplotlib.pyplot as plt

def plot_passing_latency_different_C(results):
    # 横轴标签（C 值）
    x = np.array(['60', '65', '70', '75', '80', '85', '90', '95', '100', '105', 
                  '110', '115', '120', '125', '130', '135', '140'])
    
    # 提取上界、观测值和差值数组
    upper_bound_array = [r[0] for r in results]
    observed_array = [r[1] for r in results]
    diff_array = [ub - ob for ub, ob in zip(upper_bound_array, observed_array)]

    # 创建图形和主坐标轴
    fig, ax1 = plt.subplots(figsize=(10, 4))

    # 绘制主 y 轴两条曲线：Upper Bound 和 Observed
    ax1.plot(x, upper_bound_array, marker="o", color="#e54d4c",
             markersize=12, linewidth=2, label='Upper Bound')
    ax1.plot(x, observed_array, marker="d", color="#2b9fc9",
             markersize=12, linewidth=2, label='Observed')

    # 设置主 y 轴属性
    ax1.set_xlabel('C', fontsize=20)
    ax1.set_ylabel("Passing Latency (x" + r'$10^2$' + " ms)", fontsize=20)
    ax1.grid(axis="y", linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', which='major', labelsize=16)
    ax1.set_ylim(bottom=0)

    # 创建右侧 y 轴并绘制差值曲线
    ax2 = ax1.twinx()
    ax2.plot(x, diff_array, marker="^", linestyle='--', color="green",
             linewidth=2, label='Difference')
    ax2.set_ylabel("Latency Difference", fontsize=16)
    ax2.tick_params(axis='y')
    ax2.set_ylim(bottom=0)

    # 合并两个 y 轴图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=16, loc='upper left')

    # 自动调整布局并保存图像
    plt.tight_layout()
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_different_C.svg', format='svg')

def plot_reaction_latency_different_C(results):
    # X 轴标签
    x = np.array(['60', '65', '70', '75', '80', '85', '90', '95', 
                  '100', '105', '110', '115', '120', '125', '130', '135', '140'])

    # 提取上界、观测值和差值数组
    upper_bound_array = [r[0] for r in results]
    observed_array = [r[1] for r in results]
    diff_array = [ub - ob for ub, ob in zip(upper_bound_array, observed_array)]

    fig, ax1 = plt.subplots(figsize=(10, 4))

    # 绘制主 y 轴两条曲线：Upper Bound 和 Observed
    ax1.plot(x, upper_bound_array, marker="o", color="#e54d4c",
                markersize=12, linewidth=2, label='Upper Bound')
    ax1.plot(x, observed_array, marker="d", color="#2b9fc9",
                markersize=12, linewidth=2, label='Observed')
    
    # 设置主 y 轴属性
    ax1.set_xlabel('C', fontsize=20)
    ax1.set_ylabel("Reaction Latency (x" + r'$10^2$' + " ms)", fontsize=20)
    ax1.grid(axis="y", linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', which='major', labelsize=16)

    upper_bound_array = []
    observed_array = []

    for k in range(len(x)):
        ub = results[k][0]
        ob = results[k][1]

        upper_bound_array.append(ub if ub <= 6 else np.inf)
        observed_array.append(ob if ob <= 6 else np.inf)

    # 绘图
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')

    ax.set_xlabel("" + r'$C$' + "", fontsize=20)
    ax.set_ylabel("Reaction Latency (x" + r'$10^2$' + " ms)", fontsize=20)

    ax.grid(axis="y", linestyle='--', alpha=0.7)
    ax.tick_params(axis='both', which='major', labelsize=16)

    # 设置 y 轴最大值和替换 tick label
    ax.set_ylim(0, 6)
    yticks = ax.get_yticks()
    ytick_labels = [r'$\infty$' if t == 6 else str(int(t)) for t in yticks]
    ax.set_yticks(yticks)
    ax.set_yticklabels(ytick_labels, fontsize=16)


    ax.legend(fontsize=20, loc='lower right')

    plt.savefig('src/seam_analysis/scripts/svg/reaction_latency_different_C.svg', format='svg')

def plot_passing_latency_changed_channel_num(results):
    ## Line Chart
    x = np.array(['3', '4', '5', '6', '7', '8', '9'])
    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Number of Channels', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_changed_channel_num.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_passing_latency_changed_channel_num(results):
    # 横轴标签：通道数量
    x = np.array(['3', '4', '5', '6', '7', '8', '9'])

    # 提取上界、观测值和差值数组
    upper_bound_array = [r[0] for r in results]
    observed_array = [r[1] for r in results]
    diff_array = [ub - ob for ub, ob in zip(upper_bound_array, observed_array)]

    # 创建图形和主坐标轴
    fig, ax1 = plt.subplots(figsize=(7, 4))

    # 绘制主 y 轴两条曲线：Upper Bound 和 Observed
    ax1.plot(x, upper_bound_array, marker="o", color="#e54d4c",
             markersize=12, linewidth=2, label='Upper Bound')
    ax1.plot(x, observed_array, marker="d", color="#2b9fc9",
             markersize=12, linewidth=2, label='Observed')

    # 设置主 y 轴属性
    ax1.set_xlabel('Number of Channels', fontsize=20)
    ax1.set_ylabel("Passing Latency (x" + r'$10^2$' + " ms)", fontsize=20)
    ax1.grid(axis="y", linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', which='major', labelsize=16)
    ax1.set_ylim(bottom=1.35)

    # 创建右侧 y 轴并绘制差值曲线
    ax2 = ax1.twinx()
    ax2.plot(x, diff_array, marker="^", linestyle='--', color="green",
             linewidth=2, label='Difference')
    ax2.set_ylabel("Latency Difference", fontsize=16)
    ax2.tick_params(axis='y')
    ax2.set_ylim(bottom=0)

    # 合并两个 y 轴图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=16, loc='upper left')

    # 自动调整布局并保存图像
    plt.tight_layout()
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_changed_channel_num_with_diff.svg', format='svg')

def plot_reaction_latency_changed_channel_num(results):
    ## Line Chart
    x = np.array(['3', '4', '5', '6', '7', '8', '9'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Number of Channels', fontsize=20)
    ax.set_ylabel("Reaction Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/reaction_latency_changed_channel_num.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_passing_latency_changed_period(results):
    ## Line Chart
    x = np.array(['10', '20', '30', '40', '50'])
    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Period', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_changed_period.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_reaction_latency_changed_period(results):
    ## Line Chart
    x = np.array(['10', '20', '30', '40', '50'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Period', fontsize=20)
    ax.set_ylabel("Reaction Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/reaction_latency_changed_period.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_passing_latency_changed_jitter(results):
    ## Line Chart
    x = np.array(['1.0', '1.2', '1.4', '1.6', '1.8'])
    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel(""+r'$T_i^W/T_i^B$'+"", fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_changed_jitter.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_reaction_latency_changed_jitter(results):
    ## Line Chart
    x = np.array(['1.0', '1.2', '1.4', '1.6', '1.8'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    
    for k in range(len(x)):
        ub = results[k][0]
        ob = results[k][1]

        upper_bound_array.append(ub if ub <= 6 else np.inf)
        observed_array.append(ob if ob <= 6 else np.inf)

    # 绘图
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')

    ax.set_xlabel(""+r'$T_i^W/T_i^B$'+"", fontsize=20)
    ax.set_ylabel("Reaction Latency (x" + r'$10^2$' + " ms)", fontsize=20)

    ax.grid(axis="y", linestyle='--', alpha=0.7)
    ax.tick_params(axis='both', which='major', labelsize=16)

    # 设置 y 轴最大值和替换 tick label
    ax.set_ylim(0, 6)
    yticks = ax.get_yticks()
    ytick_labels = [r'$\infty$' if t == 6 else str(int(t)) for t in yticks]
    ax.set_yticks(yticks)
    ax.set_yticklabels(ytick_labels, fontsize=16)
    
    # for k in range(len(x)):
    #     upper_bound_array.append(results[k][0])
    #     observed_array.append(results[k][1])
    
    # #print(upper_bound_array)
    # #print(observed_array)
    # # return
    # ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # # Plot the observed line
    # ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # # Set the axis labels and title
    # ax.set_xlabel(""+r'$T_i^W/T_i^B$'+"", fontsize=20)
    # ax.set_ylabel("Reaction Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # # Add a grid to the chart
    # ax.grid(axis="y", linestyle='--', alpha=0.7)

    # # Customize the tick font sizes
    # ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/reaction_latency_changed_jitter.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_passing_latency_changed_delay(results):
    ## Line Chart
    x = np.array(['0', '[0,10]', '[0,20]', '[0,30]', '[0,40]'])
    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Ramdom Delay(ms)', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_changed_delay.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_reaction_latency_changed_delay(results):
    ## Line Chart
    x = np.array(['0', '[0,10]', '[0,20]', '[0,30]', '[0,40]'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Ramdom Delay(ms)', fontsize=20)
    ax.set_ylabel("Reaction Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/reaction_latency_changed_delay.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_passing_latency_different_channel(results):
    ## Line Chart
    x = np.array(['1', '2', '3', '4', '5', '6'])
    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Index of Channel', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/passing_latency_different_channel.svg', format='svg')

    # Show the plot
    # plt.show() 

def plot_reaction_latency_different_channel(results):
    ## Line Chart
    x = np.array(['1', '2', '3', '4', '5', '6'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound_array=[]
    observed_array=[]
    for k in range(len(x)):
        upper_bound_array.append(results[k][0])
        observed_array.append(results[k][1])
    
    #print(upper_bound_array)
    #print(observed_array)
    # return
    ax.plot(x, upper_bound_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound')
    # ax.scatter(x, upper_bound_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Index of Channel', fontsize=20)
    ax.set_ylabel("Reaction Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('src/seam_analysis/scripts/svg/reaction_latency_different_channel.svg', format='svg')

    # Show the plot
    # plt.show() 
