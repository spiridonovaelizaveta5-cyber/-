#include <iostream>
#include <cmath>
using namespace std;

int main() {
    int N;
    cout << "Введите количество самолетов: ";
    cin >> N;
    
    double max_acceleration = -INFINITY;
    int best_plane = -1;
    
    for (int i = 0; i < N; i++) {
        double m, S, T, C_L, C_D;
        cout << "\nСамолет " << i+1 << ":" << endl;
        cout << "Масса (кг): "; cin >> m;
        cout << "Площадь крыла (м²): "; cin >> S;
        cout << "Тяга (Н): "; cin >> T;
        cout << "C_L: "; cin >> C_L;
        cout << "C_D: "; cin >> C_D;
        
        double rho = 1.225, V = 250, g = 9.81;
        double L = 0.5 * rho * V * V * S * C_L;
        double D = 0.5 * rho * V * V * S * C_D;
        double a_x = (T - D) / m;
        
        cout << "Ускорение: " << a_x << " м/с²" << endl;
        
        if (a_x > max_acceleration) {
            max_acceleration = a_x;
            best_plane = i + 1;
        }
    }
    
    if (best_plane != -1) {
        cout << "\nСамолет " << best_plane << " имеет наибольшее ускорение: " 
             << max_acceleration << " м/с²" << endl;
    }
    
    return 0;
}