import matplotlib.pyplot as plt
import csv


plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13


def readPts(filename):
    x = []
    y = []
    with open(filename) as file:
        lines = csv.reader(file)
        for line in lines:
            x.append(float(line[0]))
            y.append(float(line[1]))
    return (x, y)


(xOutliers, yOutliers) = readPts("./data/outliers.txt")
(xInliers, yInliers) = readPts("./data/inliers.txt")


plt.scatter(xOutliers, yOutliers,  label='outliers',
            c='r', alpha=0.75, marker='+', s=60)
plt.scatter(xInliers, yInliers, label='inliers',
            c='g', alpha=0.75, marker='+', s=60)

plt.legend()
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.title('Points Data')
plt.grid(ls='--', alpha=0.5)

# plt.show()
plt.savefig("./img/data.png", dpi=600)
