from scipy.interpolate import InterpolatedUnivariateSpline
import pdb
import matplotlib.pyplot as plt
import numpy as np
import csv

file_name = "physical3"
folder_path = "/home/orlando21/f1tenth_ws/src/single_trailer_controller/waypoints/"

def save_as_csv(x,y, fname):
    with open(fname, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for i in range(len(x)):
            writer.writerow([x[i], y[i]])

with open(folder_path + file_name + ".csv") as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    x = []
    y = []
    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))

axes = np.linspace(1,len(x), len(x));
k = 3
splx = InterpolatedUnivariateSpline(axes, x, k = k)
sply = InterpolatedUnivariateSpline(axes, y, k = k)

#plt.plot(x, y, 'ro', ms=5)
# plt.plot(splx(axes), sply(axes), 'go', lw=3, alpha=0.7)
# plt.show()

axes_sampling = np.linspace(1,len(x), 3*len(x));

save_as_csv(splx(axes_sampling), sply(axes_sampling), folder_path+file_name+"_spline.csv")




