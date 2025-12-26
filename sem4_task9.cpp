#include <iostream>
#include <iomanip>

class AutonomousControl {
private:
    double altitude;       // м
    double thrust;         // Н
    double targetAltitude; // м
public:
    AutonomousControl(double alt, double t, double target)
        : altitude(alt), thrust(t), targetAltitude(target) {}

    void updateControl() {
        if (altitude < targetAltitude) {
            thrust += 100; // увеличиваем тягу
        } else {
            thrust -= 100; // уменьшаем тягу
        }
        if (thrust < 0) thrust = 0;
    }

    void simulateStep(double dt) {
        altitude += (thrust / 500) * dt;
    }

    void printStatus() const {
        std::cout << "Высота: " << altitude << " м, тяга: " << thrust << " Н" << std::endl;
    }
};

int main() {
    AutonomousControl ac(0, 2000, 5000);
    for (int i = 0; i < 5; ++i) {
        ac.updateControl();
        ac.simulateStep(1.0);
        ac.printStatus();
    }
    return 0;
}