import numpy as np
import matplotlib.pyplot as plt

# 读取数据
data = np.loadtxt('lio_livox_slam.txt')

# 提取时间戳和值
timestamps = data[:, 0]
values = data[:, 3]

# 绘制轨迹
plt.plot(timestamps, values)
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Trajectory')
plt.show()