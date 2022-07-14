from isort import file
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13


bins = []

filename = "../pyDrawer/hist.csv"

with open(filename) as file:
    reader = csv.reader(file)
    for line in reader:
        for elem in line:
            bins.append(float(elem))


plt.bar([i for i in range(0, len(bins))],
        bins, label="value", color="lightgreen")

plt.title("hist")
plt.legend()

plt.xticks([i for i in range(0, len(bins))])

plt.show()
