#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<double> heights1 = {0, 20, 40, 60, 40, 20, 0};
    std::vector<double> times1 = {0, 1, 2, 3, 4, 5, 6};

    std::vector<double> heights2 = {0, 30, 50, 70, 50, 30, 10};
    std::vector<double> times2 = {0, 1, 2, 3, 4, 5, 6};

    auto max1_it = std::max_element(heights1.begin(), heights1.end());
    auto max2_it = std::max_element(heights2.begin(), heights2.end());

    size_t idx1 = std::distance(heights1.begin(), max1_it);
    size_t idx2 = std::distance(heights2.begin(), max2_it);

    std::cout << "Ракета 1: max = " << *max1_it << " м в t = " << times1[idx1] << " с" << std::endl;
    std::cout << "Ракета 2: max = " << *max2_it << " м в t = " << times2[idx2] << " с" << std::endl;

    if (*max1_it > *max2_it)
        std::cout << "Ракета 1 выше." << std::endl;
    else if (*max1_it < *max2_it)
        std::cout << "Ракета 2 выше." << std::endl;
    else
        std::cout << "Высоты равны." << std::endl;

    return 0;
}