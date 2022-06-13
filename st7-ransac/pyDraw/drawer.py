from turtle import color, width
from isort import file
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

# filename = "./data/good.csv"
filename = "./data/bad.csv"

x = []
y = []

with open(filename) as f:
    reader1 = csv.reader(f)
    for line in reader1:
        x.append(float(line[0]))
        y.append(float(line[1]))

plt.scatter(x, y, label='points',
            c='r', alpha=0.75, marker='+', s=60)
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Bad Points')
plt.grid(ls='--', alpha=0.5)
# plt.savefig("./img/bad.png", dpi=600)
plt.show()
