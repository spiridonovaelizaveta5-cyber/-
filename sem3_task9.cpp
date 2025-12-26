#include <iostream>
#include <vector>
#include <iomanip>

int main() {
    std::vector<double> h = {10.0, 12.0, 11.5, 13.0, 12.5, 11.0, 10.5};
    std::vector<double> smoothed(h.size());

    int n = h.size();
    for (int i = 0; i < n; ++i) {
        if (i == 0 || i == n - 1) {
            smoothed[i] = h[i];
        } else {
            smoothed[i] = (h[i-1] + h[i] + h[i+1]) / 3.0;
        }
    }

    std::cout << std::setw(10) << "Исходная" << std::setw(15) << "Сглаженная" << std::endl;
    for (size_t i = 0; i < h.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(10) << h[i]
                  << std::setw(15) << smoothed[i] << std::endl;
    }

    return 0;
}