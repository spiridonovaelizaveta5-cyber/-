#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <memory>
#include <cstdio>

const double mass_of_ms21 = 70000.0;          // масса самолета, кг
const double wing_area_of_ms21 = 113.0;       // площадь крыла, м^2
const double sum_thrust_of_ms21 = 2 * 137200.0; // суммарная тяга, Н
const double cy0_of_ms21 = 0.25;               // коэффициент подъемной силы при нулевом угле атаки
const double real_thrust_of_ms21 = 0.9;     // реалистичный процент тяги
const double start_altitude_of_ms21 = 400.0;      // начальная высота, м
const double final_altitude_of_ms21 = 6500.0;       // конечная высота, м
const double start_velocity = 310.0 / 3.6; // начальная скорость, м/с
const double final_velocity = 550.0 / 3.6;  // конечная скорость, м/с
const double gravity = 9.81;               // ускорение свободного падения, м/с^2
const double pi = 3.14159;
const double cy0 = 0.250;
const double max_climb_angle = 15.0; // максимальный угол набора высоты в градусах
const double deg_to_rad = 57.3; // перевод из градусов в радианы
const double max_vert_velocity = 8.0; // максимальная скорость набора высоты
const double min_vert_velocity = start_velocity; // минимальная скорость для набора высоты
const double delta_H = 250;
class CSVWriter {
private:
    std::ofstream file;

public:
    CSVWriter(const std::string& filename) {
        file.open(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }
    }

    ~CSVWriter() {
        if (file.is_open()) file.close();
    }

    void writeRow(const std::vector<double>& row) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << std::setprecision(6) << row[i];
            if (i < row.size() - 1) file << ",";
        }
        file << "\n";
    }

    void writeHeader(const std::vector<std::string>& headers) {
        for (size_t i = 0; i < headers.size(); ++i) {
            file << headers[i];
            if (i < headers.size() - 1) file << ",";
        }
        file << "\n";
    }
};

class TableInterpolator {
private:
    std::vector<double> altitudes;
    std::vector<double> temperatures;
    std::vector<double> pressures;
    std::vector<double> densities;
    std::vector<double> sound_speeds;

    void initializeTableData() {
        altitudes = {0, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
        temperatures = {288.15, 284.9, 281.651, 275.154, 268.659, 262.166, 255.676,
                       249.187, 242.7, 236.215, 229.733, 223.252};
        pressures = {1.01325e5, 9.54613e4, 8.98763e4, 7.95014e4, 7.01212e4, 6.16604e4,
                    5.40483e4, 4.72176e4, 4.11051e4, 3.56516e4, 3.08007e4, 2.64999e4};
        densities = {1.22500, 1.16727, 1.11166, 1.00655, 0.909254, 0.819347, 0.736429,
                    0.660111, 0.590018, 0.526783, 0.467063, 0.413510};
        sound_speeds = {340.294, 338.37, 336.435, 332.532, 328.584, 324.589, 320.545,
                       316.452, 312.306, 308.105, 303.848, 299.532};
    }

public:
    TableInterpolator() {
        initializeTableData();
    }

    double get_eq_value(double current_altitude, const std::vector<double>& altitude, const std::vector<double>& parametrs) const {
        if (current_altitude <= altitude[0]) return parametrs[0];
        if(current_altitude >= altitude.back()) return parametrs.back();

        for (size_t i = 0; i < altitude.size()-1; i++){
            if (current_altitude >= altitude[i] && current_altitude <= altitude[i+1]){
                double d_value = (current_altitude - altitude[i]) / (altitude[i+1]-altitude[i]);
                return parametrs[i] + d_value * (parametrs[i+1] - parametrs[i]);
            }
        }
        return 0.1;
    }

    double getTemperature(double current_altitude) const { return get_eq_value(current_altitude, altitudes, temperatures); }
    double getPressure(double current_altitude) const { return get_eq_value(current_altitude, altitudes, pressures); }
    double getDensity(double current_altitude) const { return get_eq_value(current_altitude, altitudes, densities); }
    double getSoundSpeed(double current_altitude) const { return get_eq_value(current_altitude, altitudes, sound_speeds); }
    double getMach(double V, double current_altitude) const { return V / getSoundSpeed(current_altitude); }
};

class TrajectoryPoint {
public:
    double time;        // Время, с
    double y;           // Высота, м
    double V;           // Скорость, м/с


    TrajectoryPoint(double t = 0, double Y = 0, double vel = 0)
        : time(t), y(Y), V(vel) {}

    void print() const {
        std::cout << "Текущая точка\n";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "t=" << time << "с, H=" << y << "м, V=" << V*3.6 << "км/ч, ";
    }
};

class Trajectory {
private:
    std::vector<TrajectoryPoint> points;

public:
    Trajectory() {}

    void addPoint(const TrajectoryPoint& point) {
        points.push_back(point);
    }
    const std::vector<TrajectoryPoint>& getPoints() const {
         return points;
    }
    double getTotalTime() const {
        return points.empty() ? 0 : points.back().time;
    }

    void saveToCSV(const std::string& filename) {
        CSVWriter csv(filename);
        csv.writeHeader({"time_s", "altitude_m", "velocity_ms"});

        for (const auto& point : points) {
            std::vector<double> row_data = {
                point.time, point.y, point.V,
            };
            csv.writeRow(row_data);
        }

        std::cout << "Траектория сохранена в файл: " << filename << std::endl;
    }

    void plotTrajectory() const {
        FILE* gp = _popen("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\" -persist", "w");
        if (!gp) {
            std::cerr << "Ошибка" << std::endl;
            return;
        }

        fprintf(gp, "set terminal wxt size 1200,800 font 'Arial,12'\n");
        fprintf(gp, "set multiplot layout 2,2 title 'parametres of flight of ms21'\n");

        // Высота от времени
        fprintf(gp, "set title 'height from time'\n");
        fprintf(gp, "set xlabel 'time, sek'\n");
        fprintf(gp, "set ylabel 'height, m'\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "plot '-' with lines lw 2 lc 'blue' title 'height'\n");
        for (const auto& point : points) {
            fprintf(gp, "%f %f\n", point.time, point.y);
        }
        fprintf(gp, "e\n");
        // Скорость от времени
        fprintf(gp, "set title 'velocity from time'\n");
        fprintf(gp, "set xlabel 'time, sek'\n");
        fprintf(gp, "set ylabel 'velocity, km/h'\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "plot '-' with lines lw 2 lc 'red' title 'velocity'\n");
        for (const auto& point : points) {
            fprintf(gp, "%f %f\n", point.time, point.V * 3.6);
        }
        fprintf(gp, "e\n");
        fprintf(gp, "unset multiplot\n");
        fflush(gp);
        std::cout << "Закройте окно Gnuplot чтобы продолжить" << std::endl;
        std::cin.get();
         _pclose(gp);
    }
};

class Aircraft {
private:
    double wing_area;
    double initial_mass;
    TableInterpolator env;

public:
    double mass;
    double thrust;      // Тяга, Н
    double fuel_flow;   // Расход топлива, кг/с
    double Cx0;         // Коэффициент сопротивления при нулевой подъемной силе
    double K;           // Коэффициент индуктивного сопротивления
    double Cy_max;      // Максимальный коэффициент подъемной силы

    Aircraft(double m = mass_of_ms21, double wing = wing_area_of_ms21)
        : wing_area(wing), initial_mass(m), env(), mass(m) {

        thrust = sum_thrust_of_ms21 * real_thrust_of_ms21;
        fuel_flow = 0.69; // кг/с - при крейсерском движении
        Cx0 = 0.0275;
        K = 0.05;
        Cy_max = 1.1;
    }
    // Расчет угла атаки (в градусах)
    double calculate_alpha(double current_altitude, double current_velocity) {
        double rho = env.getDensity(current_altitude);
        double grad_Cy = grad_Cy_alpha();
        double q = 0.5 * rho * current_velocity * current_velocity;
        double current_P = total_thrust(current_altitude);

        double alpha = (mass_of_ms21 * gravity - q * wing_area_of_ms21 * cy0_of_ms21) / (q * wing_area_of_ms21 * grad_Cy + current_P);
        return alpha;

    }
    // Расчет градиента коэффициента подъема
    double grad_Cy_alpha() const{
        double b = 39.6; // размах крыла
        double AR = b*b/wing_area_of_ms21;
        return ((2 * pi * AR) / (2 + pow(4+AR*AR, 0.5))) * pi/180;
    }

    // Коэффициент подъемной силы в зависимости от угла атаки
double getLiftCoefficient(double alpha) const{
        if (alpha < 0) alpha = 0;
        if (alpha > 12) alpha = 12;
    // Для расчета воспользуемся линейной зависимостью
        double grad_Cy = grad_Cy_alpha();
        double Cy = 0.25 + grad_Cy * alpha;
        if (Cy > 1.1)
            return 1.1;
        return Cy;

}
    // Коэффициент сопротивления в зависимости от коэффициента подъемной силы
    double getDragCoefficient(double Cl) const {
        return Cx0 + K * Cl * Cl;
    }

    // Подъемная сила
    double computeLiftForce(double current_velocity, double current_altitude, double alpha) const {
        double rho = env.getDensity(current_altitude);
        double Cy = getLiftCoefficient(alpha);
        double L = 0.5 * rho * current_velocity * current_velocity * wing_area_of_ms21 * Cy;
        return L;
    }

    // Сила сопротивления
    double computeDragForce(double current_velocity, double current_altitude, double alpha) const {
        double rho = env.getDensity(current_altitude);
        double Cx = getDragCoefficient(getLiftCoefficient(alpha));
        double X = 0.5 * rho * current_velocity * current_velocity * wing_area_of_ms21 * Cx;
        return X;
    }
    // Расчет тяги в зависимости от параметров окружающей среды
    double total_thrust(double current_altitude){
        double p_0 = env.getPressure(0);
        double current_p = env.getPressure(current_altitude);
        return sum_thrust_of_ms21*real_thrust_of_ms21 * pow(current_p/p_0, 0.7); // коррекция тяги по плотности и давлению
    }
};


class DynamicProgrammingSolver {
    private:
        TableInterpolator env;
public:
    DynamicProgrammingSolver() {}
    // Расчет этапа разгона
    double calculate_razgon(double altitude, double start_velocity, double final_velocity, Aircraft& ac){
        double avg_Vel = (start_velocity+final_velocity) / 2;
        if (avg_Vel < min_vert_velocity) avg_Vel = min_vert_velocity;
        double current_P = ac.total_thrust(altitude);
        double alpha_degree = ac.calculate_alpha(altitude, avg_Vel);

        double Cx = ac.getDragCoefficient(ac.getLiftCoefficient(alpha_degree));
        double rho = env.getDensity(altitude);
        double q = 0.5 * avg_Vel* avg_Vel * rho;
        double alpha_rad = alpha_degree/deg_to_rad;
        double a_x = ((current_P * cos(alpha_rad)) - q * Cx * wing_area_of_ms21) / mass_of_ms21;
        return (final_velocity - start_velocity) / a_x;

    }
    // Расчет этапа подъема
    double calculate_podiem(double initial_alt, double final_alt, double velocity, Aircraft& ac){
        double avg_alt = (final_alt - initial_alt) / 2;
        double current_P = ac.total_thrust(avg_alt);
        double alpha_degree = ac.calculate_alpha(avg_alt, velocity);

        double Cx = ac.getDragCoefficient(ac.getLiftCoefficient(alpha_degree));
        double rho = env.getDensity(avg_alt);
        double q = 0.5 * velocity*velocity*rho;
        double X = q * wing_area_of_ms21 * Cx;

        double sin_tetha = std::min((current_P-X) / (mass_of_ms21* gravity), max_climb_angle/deg_to_rad);
        double vel_y = velocity * sin_tetha;
        if (vel_y > max_vert_velocity) vel_y = max_vert_velocity;

        double dt = (final_alt-initial_alt) / vel_y;
        return dt;
    }
    // Расчет этапа подъем разгон
    double calculate_podiem_razgon(double initial_alt, double final_alt, double initial_vel, double final_vel, Aircraft& ac){
        double avg_alt = 0.5 * (final_alt - initial_alt);
        double avg_vel = 0.5 * (final_vel - initial_vel);

        double dH = final_alt - initial_alt;
        double dV = final_vel - initial_vel;

        double time_for_climb = calculate_podiem(initial_alt, final_alt, avg_vel, ac);
        double time_for_acc = calculate_razgon(avg_alt, initial_vel, final_vel, ac);

        double dt = std::max(time_for_climb, time_for_acc);

        double Vy = dH/dt;
        if(Vy > max_vert_velocity){
            double optional_dt = dH / max_vert_velocity;
            dt = optional_dt;
        }
        return dt;
    }
    Trajectory computeOptimalTrajectory(Aircraft& ac) {


        int N = (final_altitude_of_ms21 - start_altitude_of_ms21) / delta_H;
        double DELTA_V = (final_velocity - start_velocity) / N;
        std::vector <double> Hgrid(N+1);
        std::vector <double> Vgrid(N+1);

        for (size_t i = 0; i <= N; i++){
            Hgrid[i] = start_altitude_of_ms21 + i * delta_H;
            Vgrid[i] = start_velocity + i * DELTA_V;
        }
        std::cout << "Критерий минимизации времени" << "\n";
        std::cout << "Текущее количество N: " << N << "\n";
        std::vector<std::vector<double>> cost_table(N + 1, std::vector<double>(N + 1, 1e9));
        std::vector<std::vector<double>> time_table(N + 1, std::vector<double>(N + 1, 0.0));
        std::vector<std::vector<int>> prev_i(N + 1, std::vector<int>(N + 1, -1));
        std::vector<std::vector<int>> prev_j(N + 1, std::vector<int>(N + 1, -1));
        cost_table[0][0] = 0.0;
        for (int i = 0; i <= N; i++){
            for (int j = 0; j <= N; j++){
                if (cost_table[i][j] >= 1e9) continue;
                double current_altitude = Hgrid[i];
                double current_velocity = Vgrid[j];
                if (j < N){
                    double final_velocity = Vgrid[j+1];
                    double time_razgon = calculate_razgon(current_altitude, current_velocity, final_velocity, ac);
                    double new_cost = cost_table[i][j]+time_razgon;

                    if(new_cost < cost_table[i][j+1]){
                        cost_table[i][j+1] = new_cost;
                        time_table[i][j+1] = time_table[i][j]+time_razgon;
                        prev_i[i][j+1] = i;
                        prev_j[i][j+1] = j;

                    }
                }

                if(i < N){
                    double final_altitude_of_ms21 = Hgrid[i+1];
                    double time_podiem = calculate_podiem(current_altitude, final_altitude_of_ms21, current_velocity, ac);
                    double new_cost = cost_table[i][j]+time_podiem;

                    if (new_cost < cost_table[i+1][j]){
                        cost_table[i+1][j] = new_cost;
                        time_table[i+1][j] = time_table[i][j] + time_podiem;
                        prev_i[i+1][j] = i;
                        prev_j[i+1][j] = j;
                    }
                }

                if (i < N && j < N){
                    double final_altitude_of_ms21 = Hgrid[i+1];
                    double final_velocity = Vgrid[j+1];
                    double time_podiem_razgon = calculate_podiem_razgon(current_altitude, final_altitude_of_ms21, current_velocity, final_velocity, ac);
                    double new_cost = cost_table[i][j]+time_podiem_razgon;

                    if(new_cost < cost_table[i+1][j+1]){
                        cost_table[i+1][j+1] = new_cost;
                        time_table[i+1][j+1] = time_table[i][j] + time_podiem_razgon;
                        prev_i[i+1][j+1] = i;
                        prev_j[i+1][j+1] = j;
                    }
                }
            }
        }
        // Вывод матрицы времени
        std::cout << "Матрица времени (s):\n";
        std::cout << "     V";
        for (int j = 0; j <= N; j++) {
            std::cout << std::setw(7) << (int)Vgrid[j];
        }
        std::cout << "\nH\n";
        for (int i = 0; i <= N; i++) {
            std::cout << std::setw(5) << (int)Hgrid[i];
            for (int j = 0; j <= N; j++) {
                if (time_table[i][j] < 1e8) {
                    std::cout << std::setw(7) << (int)time_table[i][j];
                }
                else {
                    std::cout << std::setw(7) << "---";
                }
            }
            std::cout << "\n";
        }

        Trajectory trajectory;
        TrajectoryPoint point;
        std::vector<std::pair<double, double> > path;
        int ci = N, cj = N;
        size_t i = 0;
        while (ci >= 0 && cj >= 0) {
            path.push_back(std::make_pair(Hgrid[ci], Vgrid[cj]));
            point.time = time_table[ci][cj];
            point.y = Hgrid[ci];
            point.V = Vgrid[cj];
            trajectory.addPoint(point);
            int pi = prev_i[ci][cj];
            int pj = prev_j[ci][cj];
            if (pi == -1) break;
            ci = pi;
            cj = pj;
            std::cout << "Оптимальный путь полета\n";
            std::cout << "Высота: " <<path[i].first << "\nСкорость: " << path[i].second << "\n\n";
            std::cout << "Текущая точка\n";
            std::cout << "Time: " << point.time << " Y: " << point.y << " V:" << point.V << "\n";
            i++;
        }
    return trajectory;
}
};

int main() {

    try {
        Aircraft ms21;
        DynamicProgrammingSolver solver;
        Trajectory trajectory = solver.computeOptimalTrajectory(ms21);
        trajectory.saveToCSV("ms21_trajectory_realistic.csv");
        trajectory.plotTrajectory();
    } catch (const std::exception& e) {
        std::cerr << "ОШИБКА: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
