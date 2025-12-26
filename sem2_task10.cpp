#include <iostream>
#include <cmath>
using namespace std;

int main() {
    double T_min, T_max, dT, m, C_L, C_D, S, h, V;
    
    cout << "Введите минимальную тягу (Н): "; cin >> T_min;
    cout << "Введите максимальную тягу (Н): "; cin >> T_max;
    cout << "Введите шаг изменения тяги (Н): "; cin >> dT;
    cout << "Введите массу самолета (кг): "; cin >> m;
    cout << "Введите C_L: "; cin >> C_L;
    cout << "Введите C_D: "; cin >> C_D;
    cout << "Введите площадь крыла (м²): "; cin >> S;
    cout << "Введите высоту набора (м): "; cin >> h;
    cout << "Введите скорость (м/с): "; cin >> V;
    
    double rho = 1.225, g = 9.81;
    double best_T = T_min, best_time = INFINITY;
    
    for (double T = T_min; T <= T_max; T += dT) {
        double L = 0.5 * rho * V * V * S * C_L;
        double D = 0.5 * rho * V * V * S * C_D;
        double a_y = (L - m * g) / m;
        
        if (a_y <= 0) continue;
        
        double t = sqrt(2 * h / a_y);
        
        if (t < best_time) {
            best_time = t;
            best_T = T;
        }
    }
    
    if (best_time == INFINITY) {
        cout << "Невозможно набрать высоту при данных параметрах." << endl;
    } else {
        cout << "Оптимальная тяга: " << best_T << " Н" << endl;
        cout << "Минимальное время набора: " << best_time << " с" << endl;
    }
    
    return 0;
}