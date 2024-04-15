#ifndef PLOT_H
#define PLOT_H
#include <iostream>
#include <vector>
#include <iomanip>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h> // Include the pybind11 headers
#include "Input.h"
namespace py = pybind11;

void startPythonInterpreter() {
    static py::scoped_interpreter guard{}; // Ensure the interpreter is started only once
}

// Initialize the plotting environment
std::pair<py::object, py::object> setupPlot() {
    startPythonInterpreter();
    py::object plt = py::module_::import("matplotlib.pyplot");
    py::object mplot3d = py::module_::import("mpl_toolkits.mplot3d");
    py::object fig = plt.attr("figure")();
    py::object ax = fig.attr("add_subplot")(111, py::arg("projection") = "3d");

    // Drawing X, Y, Z axes at the origin
    // X-axis in red
    ax.attr("plot")(std::vector<double>{0, 1.5}, std::vector<double>{0, 0}, std::vector<double>{0, 0}, "r-");
    // Y-axis in green
    ax.attr("plot")(std::vector<double>{0, 0}, std::vector<double>{0, 1.5}, std::vector<double>{0, 0}, "g-");
    // Z-axis in blue
    ax.attr("plot")(std::vector<double>{0, 0}, std::vector<double>{0, 0}, std::vector<double>{0, 1.5}, "b-");

    ax.attr("set_xlim")(-1.5, 1.5);
    ax.attr("set_ylim")(-1.5, 1.5);
    ax.attr("set_zlim")(-1.5, 1.5);
    return { plt, ax };
}

void updatePlot(py::object& plt, py::object& ax, const std::vector<std::vector<double>>& positions, const Goal goal, double vector_length) {
    ax.attr("cla")();
    std::vector<double> xs, ys, zs;
    for (const auto& pos : positions) {
        xs.push_back(pos[0]);
        ys.push_back(pos[1]);
        zs.push_back(pos[2]);
    }

    // Drawing X, Y, Z axes at the origin
    // X-axis in red
    ax.attr("plot")(std::vector<double>{0, 1.5}, std::vector<double>{0, 0}, std::vector<double>{0, 0}, "r-");
    // Y-axis in green
    ax.attr("plot")(std::vector<double>{0, 0}, std::vector<double>{0, 1.5}, std::vector<double>{0, 0}, "g-");
    // Z-axis in blue
    ax.attr("plot")(std::vector<double>{0, 0}, std::vector<double>{0, 0}, std::vector<double>{0, 1.5}, "b-");

    ax.attr("plot")(xs, ys, zs, "o-", py::arg("color") = "b");
    ax.attr("scatter")(std::vector<double>{goal.position(0)}, std::vector<double>{goal.position(1)}, std::vector<double>{goal.position(2)}, py::arg("color") = "g");

    ax.attr("set_xlim")(-1.5, 1.5);
    ax.attr("set_ylim")(-1.5, 1.5);
    ax.attr("set_zlim")(-1.5, 1.5);
    plt.attr("draw")();
    plt.attr("pause")(0.01);
}

#endif
