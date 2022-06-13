from turtle import color, width
from isort import file
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

# filename = "./data/good.csv"
filename = "./data/bad.csv"

# -- ns_st7::Solver::leastSquare(good): [0.9645, 1.93589, 3.0065]
# -- ns_st7::Solver::leastSquare(bad): [0.509129, 1.0617, 3.61135]

# -- ns_st7::Solver::gaussNewton(good, 10): [0.964502, 1.93589, 3.0065]
# -- ns_st7::Solver::gaussNewton(bad, 10): [0.509132, 1.06171, 3.61135] 

# -- ns_st7::Solver::ransac(good, 10, 0.3f): [1.08974, 2.31455, 3.13733]
# -- ns_st7::Solver::ransac(bad, 10, 0.3f): [0.973135, 1.88441, 2.99881]
a = 0.973135
b = 1.88441
c = 2.99881

x = []
y = []

with open(filename) as f:
    reader1 = csv.reader(f)
    for line in reader1:
        x.append(float(line[0]))
        y.append(float(line[1]))

plt.scatter(x, y, label='points', c='r',
            alpha=0.75, marker='+', s=60)

para_x = []
para_y = []
mid = -b/(2.0*a)
temp_x = mid-2.0
while temp_x < mid+2.0:
    para_x.append(temp_x)
    para_y.append(a*temp_x*temp_x+b*temp_x+c)
    temp_x += 0.01

plt.plot(para_x, para_y)

plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Bad Points [RANSAC]')
plt.grid(ls='--', alpha=0.5)
plt.savefig("./img/bpransac.png", dpi=600)
plt.show()
