from turtle import color, width
from isort import file
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

# pc1_filename = "./log/nobinding/pc1.csv"
pc1_filename = "./log/nobinding/pc1_prime_4.csv"
pc2_filename = "./log/nobinding/pc2.csv"

pc1_x = []
pc1_y = []
pc2_x = []
pc2_y = []

with open(pc1_filename) as f1:
    reader1 = csv.reader(f1)
    for line in reader1:
        pc1_x.append(float(line[0]))
        pc1_y.append(float(line[1]))

with open(pc2_filename) as f2:
    reader2 = csv.reader(f2)
    for line in reader2:
        pc2_x.append(float(line[0]))
        pc2_y.append(float(line[1]))


plt.scatter(pc1_x, pc1_y, label='PC1',
            c='r', alpha=0.75, marker='+', s=60)
plt.scatter(pc2_x, pc2_y, label='PC2',
            c='g', alpha=0.75, marker='+', s=60)
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Point Clouds [src]')
plt.grid(ls='--', alpha=0.5)
# plt.savefig("./img/nobinding/src.png", dpi=600)
plt.show()
