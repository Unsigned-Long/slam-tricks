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


(x, y) = readPts("./data/pts.txt")
(xRansac, yRansac) = readPts("./data/ransac.txt")
(xLsq, yLsq) = readPts("./data/lsq.txt")


plt.scatter(x, y,  label='points',
            alpha=0.75, marker='+', s=60)

plt.scatter(xRansac, yRansac, label='ransac',
            c='orange', alpha=0.75, marker='X', s=80, edgecolors='black')
plt.scatter(xLsq, yLsq, label='LSQ',
            c='red', alpha=0.75, marker='X', s=80, edgecolors='black')

plt.legend()
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.title('Points Data')
plt.grid(ls='--', alpha=0.5)

# plt.show()
plt.savefig("./img/result.png", dpi=600)
