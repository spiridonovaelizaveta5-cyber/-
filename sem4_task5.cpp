#include <iostream>

class Autopilot {
private:
    double courseAngle; // градусы
    double altitude;    // метры
public:
    Autopilot(double course, double alt) : courseAngle(course), altitude(alt) {}

    void changeCourse(double delta) { courseAngle += delta; }
    void changeAltitude(double delta) { altitude += delta; }

    void printStatus() const {
        std::cout << "Курс: " << courseAngle << "°, высота: " << altitude << " м" << std::endl;
    }
};

int main() {
    Autopilot a1(90, 1000);
    a1.changeCourse(10);
    a1.changeAltitude(500);
    a1.printStatus();
    
    return 0;
}