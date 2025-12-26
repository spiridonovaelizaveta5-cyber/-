#include <iostream>

class Gyroscope {
private:
    double angularVelocity; // °/с
    bool calibrationRequired;
public:
    Gyroscope(double av, bool cr) : angularVelocity(av), calibrationRequired(cr) {}

    void calibrate() { calibrationRequired = false; }

    void printStatus() const {
        std::cout << "Скорость: " << angularVelocity << " °/с | Калибровка "
                  << (calibrationRequired ? "требуется" : "не требуется") << std::endl;
    }
};

int main() {
    Gyroscope g1(120.5, true);
    g1.printStatus();
    
    g1.calibrate();
    g1.printStatus();
    
    return 0;
}