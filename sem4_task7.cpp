#include <iostream>
#include <iomanip>

class FlightComputer {
private:
    double altitude; // м
    double velocity; // м/с
    double fuel;     // кг
    double thrust;   // Н
public:
    FlightComputer(double h, double v, double f, double t) 
        : altitude(h), velocity(v), fuel(f), thrust(t) {}

    void simulateStep(double dt) {
        if (fuel <= 0) return;
        velocity += (thrust / 1000 - 9.8) * dt;
        altitude += velocity * dt;
        fuel -= dt * 2;
        if (fuel < 0) fuel = 0;
    }

    void printStatus() const {
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "Высота: " << altitude << " м, скорость: " << velocity
                  << " м/с, топливо: " << fuel << " кг" << std::endl;
    }
};

int main() {
    FlightComputer fc(0, 0, 100, 12000);
    for (int i = 0; i < 5; i++) {
        fc.simulateStep(0.5);
        fc.printStatus();
    }
    return 0;
}