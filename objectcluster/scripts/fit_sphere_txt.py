# -*- coding: UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from scipy.optimize import leastsq
import math


def plot_sphere(x, y, z, r, dense):
    """
        圆心坐标 半径 稠密程度
    """
    t = np.linspace(0, np.pi * 2, dense)
    s = np.linspace(0, np.pi * 2, dense)
    t, s = np.meshgrid(t, s)             # 生成稠密网格点
    x = x + r * np.sin(s) * np.cos(t)    # 球面坐标公式
    y = y + r * np.sin(s) * np.sin(t)
    z = z + r * np.cos(s)
    return x, y, z


def fit_sphere(data_path):
    f = open(data_path, mode='r')
    x = []
    y = []
    z = []
    for line in f:
        data = line.split(' ')
        if float(data[3]) == 4:
            x.append(float(data[0]))
            y.append(float(data[1]))
            z.append(float(data[2]))
    Ax = -2 * np.array(x).reshape(-1,1)
    Ay = -2 * np.array(y).reshape(-1,1)
    Az = -2 * np.array(z).reshape(-1,1)
    Ad = np.ones((len(x),1),dtype=float)
    A = np.concatenate([Ax,Ay,Az,Ad],axis=1)
    r = np.ones((len(x),1), dtype=float).reshape(-1,1) * 0.235
    b = (-np.array(x)*np.array(x) -np.array(y)*np.array(y) -np.array(z)*np.array(z)).reshape(-1,1) + r*r
    
    # X = inv(A'*A)*A'*b
    ATA = np.dot(A.T, A)
    X = np.dot(np.dot(np.linalg.inv(ATA), A.T), b)
    
    print("func: "+str(X[0])+"^2+"+str(X[1])+"^2+"+str(X[2])+"=0.235^2")

    print(X[0])
    print(X[1])
    print(X[2])

    error_sum = 0
    for i in range(len(x)):
        error2 = math.sqrt(pow((x[i]-X[0]), 2) + pow((y[i]-X[1]), 2) + pow((z[i]-X[2]), 2)) - 0.235
        error_sum += error2
    
    mean_error = error_sum / len(x)
    print("error: "+str(mean_error))

    fig = plt.figure()
    ax = fig.gca(projection="3d")
    ax.scatter(np.array(x), np.array(y), np.array(z))
    xx, yy, zz = plot_sphere(x=X[0], y=X[1], z=X[2], r=0.235, dense=50)
    ax.plot_surface(xx, yy, zz, rstride=1, cstride=1, cmap='gray', alpha=0.5)
    plt.show()


if __name__ == "__main__":
    data_path = '/home/nc/bag/0612_dynamic/error_test/1.txt'
    fit_sphere(data_path)