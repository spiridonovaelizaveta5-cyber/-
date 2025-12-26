#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

class DroneFlight {
private:
    std::vector<double> x, y;
    double totalDistance;
public:
    DroneFlight() : totalDistance(0) {
        x.push_back(0);
        y.push_back(0);
    }

    void addPoint(double newX, double newY) {
        double dx = newX - x.back();
        double dy = newY - y.back();
        totalDistance += sqrt(dx*dx + dy*dy);
        x.push_back(newX);
        y.push_back(newY);
    }

    double getTotalDistance() const { return totalDistance; }

    void printPath() const {
        std::cout << "Точки маршрута:" << std::endl;
        for (size_t i = 0; i < x.size(); ++i) {
            std::cout << "(" << x[i] << ", " << y[i] << ")" << std::endl;
        }
    }
};

int main() {
    DroneFlight d;
    d.addPoint(0, 0);
    d.addPoint(3, 4);
    d.addPoint(6, 8);
    
    d.printPath();
    std::cout << "Пройдено: " << std::fixed << std::setprecision(1) 
              << d.getTotalDistance() << " м" << std::endl;
    
    return 0;
}