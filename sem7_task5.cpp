#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

class MotionAnalyzer {
private:
    std::vector<double> t, x, v, a;
public:
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        std::getline(file, line); // заголовок
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
        v.resize(t.size());
        auto velLambda = [this](size_t i) {
            return (x[i+1] - x[i]) / (t[i+1] - t[i]);
        };
        for (size_t i = 0; i < t.size() - 1; i++) {
            v[i] = velLambda(i);
        }
        v.back() = v[v.size() - 2]; // последнее = предпоследнее
    }

    void computeAcceleration() {
        a.resize(t.size());
        auto accLambda = [this](size_t i) {
            return (v[i+1] - v[i]) / (t[i+1] - t[i]);
        };
        for (size_t i = 0; i < t.size() - 1; i++) {
            a[i] = accLambda(i);
        }
        a.back() = a[a.size() - 2];
    }

    void saveResults(const std::string& filename) {
        std::ofstream out(filename);
        out << "t,x,v,a\n";
        for (size_t i = 0; i < t.size(); i++) {
            out << t[i] << "," << x[i] << "," << v[i] << "," << a[i] << "\n";
        }
        out.close();
    }

    void plot() {
        FILE* gp = popen("gnuplot -persist", "w");
        fprintf(gp, "set title 'Motion Analysis'\n");
        fprintf(gp, "set xlabel 'Time (s)'\n");
        fprintf(gp, "set ylabel 'Value'\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "plot 'motion_processed.csv' using 1:3 with lines title 'v(t)', "
                    "'motion_processed.csv' using 1:4 with lines title 'a(t)'\n");
        fflush(gp);
        pclose(gp);
    }
};

int main() {
    MotionAnalyzer ma;
    if (!ma.loadFromFile("motion.csv")) {
        std::cerr << "Ошибка загрузки файла\n";
        return 1;
    }
    ma.computeVelocity();
    ma.computeAcceleration();
    ma.saveResults("motion_processed.csv");
    ma.plot();
    return 0;
}