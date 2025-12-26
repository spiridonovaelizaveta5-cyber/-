#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<double> heights = {15.0, 5.0, 20.0, 8.0, 30.0, 3.0};

    auto new_end = std::remove_if(heights.begin(), heights.end(),
                                  [](double h) { return h < 10.0; });
    heights.erase(new_end, heights.end());

    std::cout << "Оставшиеся высоты (>= 10 м): ";
    for (double h : heights) std::cout << h << " ";
    std::cout << std::endl;

    return 0;
}