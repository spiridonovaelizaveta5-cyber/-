#include <iostream>
#include <iomanip>

class Sensor {
private:
    double signalStrength; // в дБ
    bool isActive;
public:
    Sensor(double strength) : signalStrength(strength), isActive(true) {
        if (signalStrength < 0) signalStrength = 0;
    }

    void activate() { isActive = true; }
    void deactivate() { isActive = false; }

    void setSignalStrength(double s) {
        if (s < 0) s = 0;
        signalStrength = s;
    }

    double getSignalStrength() const { return signalStrength; }

    void printStatus() const {
        std::cout << "Датчик " << (isActive ? "активен" : "выключен")
                  << " | Сила сигнала: " << std::fixed << std::setprecision(1)
                  << signalStrength << " дБ" << std::endl;
    }
};

int main() {
    Sensor s1(45.5);
    s1.printStatus();
    
    s1.setSignalStrength(-10);
    s1.printStatus();
    
    s1.deactivate();
    s1.printStatus();
    
    return 0;
}