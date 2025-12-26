#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

class SensorData {
private:
    std::vector<double> t, h1, h2;
public:
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        std::getline(file, line); // заголовок
        double ti, hi1, hi2;
        char comma;
        while (file >> ti >> comma >> hi1 >> comma >> hi2) {
            t.push_back(ti);
            h1.push_back(hi1);
            h2.push_back(hi2);
        }
        file.close();
        return true;
    }

    void computeDifference() {
        auto diffLambda = [](double a, double b) { return std::abs(a - b); };
        std::ofstream out("diff.csv");
        out << "t,h1,h2,diff\n";
        for (size_t i = 0; i < t.size(); i++) {
            double delta = diffLambda(h1[i], h2[i]);
            out << t[i] << "," << h1[i] << "," << h2[i] << "," << delta << "\n";
        }
        out.close();
    }

    void plot() {
        FILE* gp = popen("gnuplot -persist", "w");
        fprintf(gp, "set title 'Sensor Comparison'\n");
        fprintf(gp, "set xlabel 'Time (s)'\n");
        fprintf(gp, "set ylabel 'Height (m)'\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "plot 'diff.csv' using 1:2 with lines title 'h1(t)', "
                    "'diff.csv' using 1:3 with lines title 'h2(t)', "
                    "'diff.csv' using 1:4 with lines title 'Δh(t)'\n");
        fflush(gp);
        pclose(gp);
    }
};

int main() {
    SensorData sensors;
    if (!sensors.loadFromFile("sensors.csv")) {
        std::cerr << "Ошибка загрузки файла\n";
        return 1;
    }
    sensors.computeDifference();
    sensors.plot();
    return 0;
}