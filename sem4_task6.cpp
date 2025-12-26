#include <iostream>
#include <iomanip>

class RocketStage {
private:
    double thrust;   // Н
    double burnTime; // с
    double mass;     // кг
public:
    RocketStage(double T, double t, double m) : thrust(T), burnTime(t), mass(m) {}

    double getDeltaV() const {
        if (mass == 0) return 0;
        return (thrust * burnTime) / mass;
    }

    void printInfo() const {
        std::cout << "Тяга: " << thrust << " Н, время: " << burnTime
                  << " с, масса: " << mass << " кг, ΔV: " << getDeltaV() << " м/с" << std::endl;
    }
};

int main() {
    RocketStage s1(5000, 10, 1000);
    RocketStage s2(7000, 8, 900);
    RocketStage s3(9000, 6, 800);

    double totalV = s1.getDeltaV() + s2.getDeltaV() + s3.getDeltaV();
    std::cout << "Итоговая скорость ракеты: " << totalV << " м/с" << std::endl;
    
    return 0;
}