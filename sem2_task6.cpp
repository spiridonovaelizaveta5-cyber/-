#include <iostream>
#include <iomanip>
using namespace std;

int main() {
    const int N = 5;
    double V[N] = {100, 110, 120, 130, 140};
    double rho[N] = {1.225, 1.2, 1.18, 1.15, 1.12};
    double S = 50, C_L = 0.3;
    
    cout << setw(10) << "Шаг" << setw(15) << "Скорость" 
         << setw(15) << "Плотность" << setw(20) << "Подъемная сила" << endl;
    
    for (int i = 0; i < N; i++) {
        double L = 0.5 * rho[i] * V[i] * V[i] * S * C_L;
        cout << setw(10) << i+1 
             << setw(15) << V[i] 
             << setw(15) << rho[i] 
             << setw(20) << L << endl;
    }
    
    return 0;
}