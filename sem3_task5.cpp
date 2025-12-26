#include <iostream>
#include <vector>
#include <iomanip>

struct Point {
    double x, y, z;
};

int main() {
    double vx = 30.0, vy = 20.0, vz = 40.0;
    double dt = 0.1;
    double t_max = 10.0;

    std::vector<Point> traj;
    traj.reserve(static_cast<int>(t_max / dt) + 10);

    double x = 0, y = 0, z = 0;

    for (double t = 0; t <= t_max; t += dt) {
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
        traj.push_back({x, y, z});
    }

    std::cout << std::setw(10) << "t (c)" 
              << std::setw(15) << "x (м)" 
              << std::setw(15) << "y (м)" 
              << std::setw(15) << "z (м)" << std::endl;
    std::cout << std::string(55, '-') << std::endl;

    for (size_t i = 0; i < traj.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(10) << i * dt
                  << std::setw(15) << traj[i].x
                  << std::setw(15) << traj[i].y
                  << std::setw(15) << traj[i].z << std::endl;
    }

    return 0;
}