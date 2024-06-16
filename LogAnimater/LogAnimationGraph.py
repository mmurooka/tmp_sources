#!/usr/bin/env python

import subprocess
import mc_log_ui
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams["font.size"] = 18

class LogAnimationGraph(object):
    """Class to generate animation graph from mc_rtc log."""

    def __init__(self, log, start_time=None, end_time=None, hz=30):
        """Constructor.

        Args:
            log: mc_rtc log instance
            start_time: Start time of plot [sec]
            end_time: End time of plot [sec]
            hz: Video frequency [Hz]
        """
        self.log = log

        self.start_time = start_time
        self.end_time = end_time
        if self.start_time is None:
            self.start_time = self.log["t"][0]
        if self.end_time is None:
            self.end_time = self.log["t"][-1]
        self.start_idx = (self.log["t"] >= self.start_time).argmax()
        self.end_idx = (self.log["t"] >= self.end_time).argmax()

        self.dt = self.log["t"][1] - self.log["t"][0]
        self.skip_num = int(1. / (self.dt * hz))
        self.writer = animation.writers["ffmpeg"](fps=hz)

    def run(self, output_filename, plot_info_list, **kwargs):
        """Generate an animation graph video.

        The plot target is specified by the `plot_info_list` argument, which is a list of plot information.
        A plot information consists of `key` in mc_rtc log and `kwargs` passed as arguments to matplotlib plot method.

        Args:
            output_filename: file name of output video
            plot_info_list: list of plot information
            title (str): Graph title
            xlabel (str): X label
            ylabel (str): Y label
            ylim (list): Y range
        """
        fig, ax = plt.subplots()
        fig.canvas.manager.set_window_title("LogAnimationGraph")
        fig.set_size_inches((20.0, 6.0))
        if "title" in kwargs:
            ax.set_title(kwargs["title"])
        ax.grid()
        if "xlabel" in kwargs:
            ax.set_xlabel(kwargs["xlabel"])
        if "ylabel" in kwargs:
            ax.set_ylabel(kwargs["ylabel"], labelpad=-2)
        if "ylim" in kwargs:
            ax.set_ylim(kwargs["ylim"])

        line_list = []
        ax.set_xlim(0, self.end_time - self.start_time)
        for plot_info in plot_info_list:
            plot_info["xdata"] = []
            plot_info["ydata"] = []
            plot_info["line"], = ax.plot([], [], **plot_info["kwargs"])
            line_list.append(plot_info["line"])
        ax.legend()

        def frames():
            for frame in xrange(self.start_idx, self.end_idx, self.skip_num):
                t = self.log["t"][frame] - self.start_time
                data_list = []
                for plot_info in plot_info_list:
                    if "func" in plot_info:
                        x = plot_info["func"](self.log[plot_info["key"]][frame])
                    else:
                        x = self.log[plot_info["key"]][frame]
                    data_list.append([t, x])
                yield data_list

        def func(data_list):
            for plot_info, data in zip(plot_info_list, data_list):
                plot_info["xdata"].append(data[0])
                plot_info["ydata"].append(data[1])
                plot_info["line"].set_data(plot_info["xdata"], plot_info["ydata"])
            return [plot_info["line"] for plot_info in plot_info_list]

        print("[LogAnimationGraph] Start to generate a video to {}".format(output_filename))
        self.ani = animation.FuncAnimation(
            fig, func, frames,
            interval=int(1e3*self.dt*self.skip_num),
            repeat=False, blit=True,
            save_count=self.end_idx-self.start_idx)
        self.ani.save(output_filename, writer="ffmpeg")
        subprocess.Popen(["notify-send", "[LogAnimationGraph] run finished."])

if __name__ == "__main__":
    input_filename = "/tmp/log.bin"
    output_filename = "/tmp/log.mp4"

    log = mc_log_ui.read_log(input_filename)
    log_animation_graph = LogAnimationGraph(log, 10.0, 30.0)
    log_animation_graph.run(
        output_filename,
        [{"key": "CentroidalManager_ZMP_ref_y",
          "kwargs": {"label": "ref ZMP",
                     "linewidth": 4.0,
                     "color": "blue"}},
         {"key": "CentroidalManager_ZMP_control_y",
          "kwargs": {"label": "desired ZMP",
                     "linewidth": 4.0,
                     "color": "green"}},
         {"key": "CentroidalManager_ZMP_measured_y",
          "kwargs": {"label": "measured ZMP",
                     "linewidth": 4.0,
                     "color": "red"}},
         {"key": "CentroidalManager_CoM_controlRobot_y",
          "kwargs": {"label": "control robot CoM",
                     "linestyle": "--",
                     "linewidth": 4.0,
                     "color": "green"}},
         {"key": "CentroidalManager_CoM_realRobot_y",
          "kwargs": {"label": "real robot CoM",
                     "linestyle": "--",
                     "linewidth": 4.0,
                     "color": "red"}},
         {"key": "CentroidalManager_ZMP_SupportRegion_max_y",
          "kwargs": {"label": "support region",
                     "linewidth": 2.0,
                     "color": "black"}},
         {"key": "CentroidalManager_ZMP_SupportRegion_min_y",
          "kwargs": {"linewidth": 2.0,
                     "color": "black"}}
        ],
        title="JVRC1 walking",
        xlabel="time [s]",
        ylabel="Y position [m]",
        ylim=[-0.6, 0.2]
    )
