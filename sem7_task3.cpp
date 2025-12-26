#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <utility>

int main() {
    std::vector<std::pair<double, double>> data;
    std::ifstream in("altitude.csv");
    if (!in.is_open()) {
        std::cerr << "Ошибка открытия altitude.csv\n";
        return 1;
    }
    std::string line;
    std::getline(in, line); // заголовок
    double t, H;
    char comma;
    while (in >> t >> comma >> H) {
        data.push_back({t, H});
    }
    in.close();

    // Фильтрация
    auto filterLambda = [](const std::pair<double, double>& p) {
        return p.second > 900 && p.second < 1100;
    };
    std::vector<std::pair<double, double>> filtered;
    std::copy_if(data.begin(), data.end(), std::back_inserter(filtered), filterLambda);

    // Запись отфильтрованных данных
    std::ofstream out("filtered.csv");
    out << "t,H\n";
    for (const auto& p : filtered) {
        out << p.first << "," << p.second << "\n";
    }
    out.close();

    // GNUPlot
    FILE* gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set title 'Altitude Filtering'\n");
    fprintf(gp, "set xlabel 'Time (s)'\n");
    fprintf(gp, "set ylabel 'Altitude (m)'\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "plot 'altitude.csv' using 1:2 with points title 'Original', "
                "'filtered.csv' using 1:2 with lines title 'Filtered'\n");
    fflush(gp);
    pclose(gp);

    return 0;
}