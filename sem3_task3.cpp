#include <iostream>
#include <vector>
#include <iomanip>

int main() {
    double v0 = 50.0;
    double g = 9.8;
    double dt = 1.0;
    double t_max = v0 / g * 2; // Примерно до падения

    std::vector<double> times;
    std::vector<double> velocities;

    for (double t = 0.0; t <= t_max; t += dt) {
        double v = v0 - g * t;
        times.push_back(t);
        velocities.push_back(v);
    }

    std::cout << std::setw(10) << "t (c)" << std::setw(15) << "v (м/с)" << std::endl;
    std::cout << std::string(25, '-') << std::endl;

    for (size_t i = 0; i < times.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(10) << times[i]
                  << std::setw(15) << velocities[i] << std::endl;
    }

    return 0;
}