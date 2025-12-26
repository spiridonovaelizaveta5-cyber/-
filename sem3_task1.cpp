#include <iostream>
#include <vector>
#include <iomanip>

int main() {
    double v0 = 50.0;
    double g = 9.8;
    double dt = 1.0;

    std::vector<double> times;
    std::vector<double> heights;

    // Резервируем память для оптимизации
    times.reserve(static_cast<int>(v0 * 2 / g / dt) + 10);

    for (double t = 0.0; t <= 1000; t += dt) {
        double h = v0 * t - 0.5 * g * t * t;
        if (h < 0) break;
        times.push_back(t);
        heights.push_back(h);
    }

    // Заголовок таблицы
    std::cout << std::setw(10) << "t (c)" << std::setw(15) << "h (м)" << std::endl;
    std::cout << std::string(25, '-') << std::endl;

    // Данные
    for (size_t i = 0; i < times.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(10) << times[i]
                  << std::setw(15) << heights[i] << std::endl;
    }

    return 0;
}