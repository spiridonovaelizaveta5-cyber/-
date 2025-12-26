#include <iostream>
#include <vector>
#include <numeric>
#include <iomanip>

int main() {
    std::vector<double> velocities = { /* данные из задачи 3 */ };

    if (velocities.empty()) {
        std::cout << "Вектор скоростей пуст." << std::endl;
        return 0;
    }

    double sum = std::accumulate(velocities.begin(), velocities.end(), 0.0);
    double avg = sum / velocities.size();

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Средняя скорость: " << avg << " м/с" << std::endl;

    return 0;
}