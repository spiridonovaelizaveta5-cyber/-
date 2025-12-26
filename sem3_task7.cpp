#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<double> heights = {50, 40, 30, 20, 10, 0, -5, -10};
    std::vector<double> times = {0, 1, 2, 3, 4, 5, 6, 7};

    auto it = std::find_if(heights.begin(), heights.end(),
                           [](double h) { return h <= 0.0; });

    if (it == heights.end()) {
        std::cout << "Падения не было в диапазоне." << std::endl;
    } else {
        size_t idx = std::distance(heights.begin(), it);
        std::cout << "Падение на шаге " << idx 
                  << ", время = " << times[idx] 
                  << " с, высота = " << *it << " м" << std::endl;
    }

    return 0;
}