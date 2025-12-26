#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<double> sensor_data = {0.1, 0.0, 2.3, 0.0, 4.5, 0.0, 6.7};

    int zero_count = std::count(sensor_data.begin(), sensor_data.end(), 0.0);

    std::cout << "Количество нулевых показаний: " << zero_count << std::endl;

    return 0;
}