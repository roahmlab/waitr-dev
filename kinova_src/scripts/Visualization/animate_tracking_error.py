#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.io import savemat

data_path: str = "~\Downloads\HardwareVideoROSData.csv"
# number of measurements to skip
skip_n = 100
# font size
ft_size = 15
# not sure
start_frame = 10
if __name__ == "__main__":
    # loading data
    data = pd.read_csv(data_path)
    time = np.array(data["__time"])
    e = np.array([data[f"/debug/e.{i}"] for i in range(7)])
    ed = np.array([data[f"/debug/ed.{i}"] for i in range(7)])
    time = time[::skip_n]
    e = e[:, ::skip_n]
    ed = ed[:, ::skip_n]

    time = time[start_frame:]
    e = e[:, start_frame:]
    ed = ed[:, start_frame:]
    time -= time[0]

    # cut_idx = np.argmax(time > 220)
    # e = e[:cut_idx, :]
    # ed = ed[:cut_idx, :]
    # time = time[:cut_idx]

    # cut_l = np.argmin(time < 100)
    # cut_h = np.argmax(time > 150)
    # e = np.hstack([e[:, :cut_l], e[:, cut_h:]])
    # ed = np.hstack([ed[:, :cut_l], ed[:, cut_h:]])
    # time[cut_h:] -= 50
    # time = np.concatenate([time[:cut_l], time[cut_h:]])

    fig, ax = plt.subplots(2, 1, sharex=True)
    plt.xlabel("time (s)", fontsize=ft_size)
    ax[0].set_ylabel("Position Error (rad)", fontsize=ft_size)
    ax[1].set_ylabel("Velocity Error (rad/s)", fontsize=ft_size)
    fig.set_figwidth(10)
    fig.set_figheight(7)
    ax[0].set_title("Position Tracking Error", fontsize=ft_size + 5)
    ax[1].set_title("Velocity Tracking Error", fontsize=ft_size + 5)
    ln: list = [ax[0].plot([], [])[0] for i in range(7)]
    ln.extend([ax[1].plot([], [])[0] for i in range(7)])
    ax[0].plot([0, max(time)], [0.0132, 0.0132], "k--")
    ax[0].plot([0, max(time)], [-0.0132, -0.0132], "k--")
    ax[1].plot([0, max(time)], [0.132, 0.132], "k--")
    ax[1].plot([0, max(time)], [-0.132, -0.132], "k--")
    legends = [f"joint {i+1}" for i in range(7)]
    legends.append("ultimate bound")

    # save to matlab
    savemat("data.mat", {"t": time, "e": e, "ed": ed})

    # ax[0].legend(legends, fontsize=18)

    # ax[1].legend(legends)


    def init():
        ax[0].set_xlim(0, max(time))
        ax[0].set_ylim(-0.014, 0.014)
        ax[1].set_xlim(0, max(time))
        ax[1].set_ylim(-0.14, 0.14)
        return ln

    def update(frame):
        for i in range(7):
            ln[i].set_data(time[:frame], e[i, :frame])

        for i in range(7):
            ln[i + 7].set_data(time[:frame], ed[i, :frame])
        return ln

    ani = FuncAnimation(fig,
                        update,
                        frames=len(time),
                        init_func=init,
                        blit=True,
                        interval=100)

    # plt.show()
    ani.save("HardwareVideoTrackingError.mp4", dpi=300)