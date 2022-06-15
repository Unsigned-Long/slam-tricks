from cProfile import label
import matplotlib.pyplot as plt

# plt.xkcd()
plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

# hws = 2
vals1 = [0.0467707, 0.1054, 0.209616, 0.367905, 0.569906, 0.77932, 0.941344, 1.00633, 0.957729, 0.825571,
         0.675182, 0.577491, 0.577491, 0.675182, 0.825571, 0.957729, 1.00633, 0.941344, 0.77932, 0.569906]
midx1 = [7, 16]

vals2 = [7.8259e-06, 0.131538, 0.755605, 0.45865, 0.532767, 0.218959, 0.0470446, 0.678865, 0.679296, 0.934693,
         0.383502, 0.519416, 0.830965, 0.0345721, 0.0534616, 0.5297, 0.671149, 0.00769819, 0.383416, 0.0668422]
midx2 = [2, 9, 12, 16]

# hws = 3
vals3 = [0.0467707, 0.1054, 0.209616, 0.367905, 0.569906, 0.77932, 0.941344, 1.00633, 0.957729, 0.825571,
         0.675182, 0.577491, 0.577491, 0.675182, 0.825571, 0.957729, 1.00633, 0.941344, 0.77932, 0.569906]
midx3 = [7, 16]
vals4 = [7.8259e-06, 0.131538, 0.755605, 0.45865, 0.532767, 0.218959, 0.0470446, 0.678865, 0.679296, 0.934693,
         0.383502, 0.519416, 0.830965, 0.0345721, 0.0534616, 0.5297, 0.671149, 0.00769819, 0.383416, 0.0668422]
midx4 = [9, 16]


def foo(vals, midx, hws):
    plt.bar([i for i in range(0, len(vals))],
            vals, label="value", color="lightgreen")
    plt.bar(midx, [vals[i] for i in midx], label="local max",  color="red")

    plt.title("NMS 1D [ WinSize:"+str(hws)+" ]")
    plt.legend()

    plt.xticks([i for i in range(0, len(vals))])
    plt.savefig("./img/norm_nms1d_"+str(hws)+".png", dpi=600)

    plt.show()


foo(vals1, midx1, 2)
# foo(vals2, midx2, 2)
foo(vals3, midx3, 3)
# foo(vals4, midx4, 3)
