#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>

int main() {
    // Допустим, heights уже заполнен из задачи 1
    std::vector<double> heights = { /* данные */ };
    std::vector<double> times = { /* данные */ };

    if (heights.empty()) {
        std::cout << "Данные отсутствуют." << std::endl;
        return 0;
    }

    // Используем STL
    auto max_it = std::max_element(heights.begin(), heights.end());
    size_t idx = std::distance(heights.begin(), max_it);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Максимальная высота: " << *max_it << " м" << std::endl;
    std::cout << "Время достижения: " << times[idx] << " с" << std::endl;

    return 0;
}