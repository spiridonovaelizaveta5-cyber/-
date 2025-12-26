#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

class Navigator {
private:
    std::vector<double> t, x, y;
public:
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        std::getline(file, line); // заголовок
        double ti, xi, yi;
        char comma;
        while (file >> ti >> comma >> xi >> comma >> yi) {
            t.push_back(ti);
            x.push_back(xi);
            y.push_back(yi);
        }
        file.close();
        return true;
    }

    void computeVelocity() {
        std::vector<double> vx, vy, v;
        for (size_t i = 0; i < t.size() - 1; i++) {
            vx.push_back((x[i+1] - x[i]) / (t[i+1] - t[i]));
            vy.push_back((y[i+1] - y[i]) / (t[i+1] - t[i]));
        }
        // Последнее значение дублируем
        vx.push_back(vx.back());
        vy.push_back(vy.back());

        // Модуль скорости
        auto speedLambda = [](double vx, double vy) { return std::sqrt(vx*vx + vy*vy); };
        for (size_t i = 0; i < t.size(); i++) {
            v.push_back(speedLambda(vx[i], vy[i]));
        }

        // Сохранение
        std::ofstream out("velocity_nav.csv");
        out << "t,v\n";
        for (size_t i = 0; i < t.size(); i++) {
            out << t[i] << "," << v[i] << "\n";
        }
        out.close();
    }

    void plot() {
        FILE* gp = popen("gnuplot -persist", "w");
        fprintf(gp, "set title 'Speed v(t)'\n");
        fprintf(gp, "set xlabel 'Time (s)'\n");
        fprintf(gp, "set ylabel 'Speed (m/s)'\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "plot 'velocity_nav.csv' using 1:2 with linespoints title 'v(t)'\n");
        fflush(gp);
        pclose(gp);
    }
};

int main() {
    Navigator nav;
    if (!nav.loadFromFile("navigation.csv")) {
        std::cerr << "Ошибка загрузки файла\n";
        return 1;
    }
    nav.computeVelocity();
    nav.plot();
    return 0;
}