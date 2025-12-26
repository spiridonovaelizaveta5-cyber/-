#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

class Trajectory {
private:
    std::vector<double> t;
    std::vector<double> x;
public:
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        // Пропускаем заголовок "t,x"
        std::getline(file, line);
        double ti, xi;
        char comma;
        while (file >> ti >> comma >> xi) {
            t.push_back(ti);
            x.push_back(xi);
        }
        file.close();
        return true;
    }

    void computeVelocity() {
        std::vector<double> v;
        auto velLambda = [this](size_t i) {
            return (x[i+1] - x[i]) / (t[i+1] - t[i]);
        };
        for (size_t i = 0; i < t.size() - 1; i++) {
            v.push_back(velLambda(i));
        }
        v.push_back(v.back()); // дублируем последнее значение

        // Сохраняем результаты
        std::ofstream out("velocity.csv");
        out << "t,x,v\n";
        for (size_t i = 0; i < t.size(); i++) {
            out << t[i] << "," << x[i] << "," << v[i] << "\n";
        }
        out.close();
    }

    void plot() {
        FILE* gp = popen("gnuplot -persist", "w");
        fprintf(gp, "set title 'Trajectory x(t)'\n");
        fprintf(gp, "set xlabel 'Time (s)'\n");
        fprintf(gp, "set ylabel 'Position (m)'\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "plot 'velocity.csv' using 1:2 with linespoints title 'x(t)'\n");
        fflush(gp);
        pclose(gp);
    }
};

int main() {
    Trajectory traj;
    if (!traj.loadFromFile("traj.csv")) {
        std::cerr << "Ошибка загрузки файла\n";
        return 1;
    }
    traj.computeVelocity();
    traj.plot();
    return 0;
}