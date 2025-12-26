#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<double> heights = {0, 50, 100, 80, 60, 40, 20, 0};
    std::vector<double> times = {0, 1, 2, 3, 4, 5, 6, 7};

    auto max_it = std::max_element(heights.begin(), heights.end());
    size_t idx = std::distance(heights.begin(), max_it);

    std::cout << "Максимальная высота: " << *max_it 
              << " м в момент t = " << times[idx] << " с" << std::endl;

    return 0;
}