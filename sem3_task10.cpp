#include <iostream>
#include <vector>
#include <iomanip>

class Rocket {
private:
    double x, y, z;
    double vx, vy, vz;
public:
    Rocket(double vx0, double vy0, double vz0) 
        : x(0), y(0), z(0), vx(vx0), vy(vy0), vz(vz0) {}

    void updatePosition(double dt) {
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
    }

    void printPosition() const {
        std::cout << std::fixed << std::setprecision(2)
                  << "(" << x << ", " << y << ", " << z << ")";
    }
};

int main() {
    std::vector<Rocket> squad;
    squad.reserve(5);
    
    squad.emplace_back(100, 10, 0);
    squad.emplace_back(90, 15, -5);
    squad.emplace_back(110, 5, 10);
    squad.emplace_back(95, 20, -10);
    squad.emplace_back(105, 0, 5);

    double dt = 0.1;
    double T = 10.0;

    for (double t = 0; t <= T; t += dt) {
        for (auto& rocket : squad) {
            rocket.updatePosition(dt);
        }
    }

    std::cout << "Конечные положения ракет:" << std::endl;
    for (size_t i = 0; i < squad.size(); ++i) {
        std::cout << "Ракета " << i+1 << ": ";
        squad[i].printPosition();
        std::cout << std::endl;
    }

    return 0;
}