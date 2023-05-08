from scipy.interpolate import InterpolatedUnivariateSpline
import pdb
import matplotlib.pyplot as plt
import numpy as np
import csv
import math

file_name = "trailer_mpc_wu_chen_2_obs1"
folder_path = "/home/aadith/Desktop/f1_tenth/workspace/src/project/waypoints/"

def save_as_csv(x,y, fname):
    with open(fname, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        v = [2]*len(x);
        yaw = []
        for i in range(1,len(x)):
            yaw += [math.atan2(y[i] - y[i-1], x[i] - x[i-1])];
        yaw += [math.atan2(y[0] - y[-1], x[0] - x[-1])];


        for i in range(len(x)):
            writer.writerow([x[i], y[i], v[i], yaw[i]])

with open(folder_path + file_name + ".csv") as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    x = []
    y = []
    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))
    # x.append(x[0]);
    # y.append(y[0]);
axes = np.linspace(1,len(x), len(x));
k = 1
splx = InterpolatedUnivariateSpline(axes, x, k = k)
sply = InterpolatedUnivariateSpline(axes, y, k = k)

sampling_density = 2;
axes_sampling = np.linspace(1,len(x), sampling_density*len(x));

save_as_csv(splx(axes_sampling), sply(axes_sampling), folder_path+file_name+"_spline_"+str(sampling_density)+".csv")




