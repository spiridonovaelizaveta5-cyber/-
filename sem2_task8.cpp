#include <iostream>
#include <cmath>
#include <algorithm>
using namespace std;

struct Aircraft {
    double m, T, C_L, C_D;
};

int main() {
    const int N = 4;
    Aircraft planes[N] = {
        {10000, 30000, 0.3, 0.02},
        {12000, 35000, 0.35, 0.025},
        {9000, 28000, 0.32, 0.018},
        {11000, 32000, 0.33, 0.022}
    };
    
    double rho = 1.225, S = 50, g = 9.81, V = 250;
    double times[N];
    
    for (int i = 0; i < N; i++) {
        double L = 0.5 * rho * V * V * S * planes[i].C_L;
        double D = 0.5 * rho * V * V * S * planes[i].C_D;
        double a_y = (L - planes[i].m * g) / planes[i].m;
        
        if (a_y <= 0) {
            times[i] = INFINITY;
        } else {
            times[i] = sqrt(2 * 1000 / a_y);
        }
        cout << "Самолет " << i+1 << ": время = " << times[i] << " с" << endl;
    }
    
    // Простая сортировка (пузырьком) для наглядности
    for (int i = 0; i < N-1; i++) {
        for (int j = 0; j < N-i-1; j++) {
            if (times[j] > times[j+1]) {
                swap(times[j], times[j+1]);
                swap(planes[j], planes[j+1]);
            }
        }
    }
    
    cout << "\nОтсортировано по времени набора:" << endl;
    for (int i = 0; i < N; i++) {
        cout << "Самолет " << i+1 << ": " << times[i] << " с" << endl;
    }
    
    return 0;
}