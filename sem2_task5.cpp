#include <iostream>
#include <cmath>
using namespace std;

struct Aircraft {
    double m, S, T, C_L, C_D;
};

int main() {
    const int N = 3;
    Aircraft planes[N] = {
        {10000, 50, 30000, 0.3, 0.02},
        {12000, 60, 35000, 0.35, 0.025},
        {9000, 45, 28000, 0.32, 0.018}
    };
    
    double rho = 1.225, g = 9.81;
    double best_time = INFINITY;
    int best_index = -1;
    
    for (int i = 0; i < N; i++) {
        double L = 0.5 * rho * 250 * 250 * planes[i].S * planes[i].C_L;
        double D = 0.5 * rho * 250 * 250 * planes[i].S * planes[i].C_D;
        double a_y = (L - planes[i].m * g) / planes[i].m;
        
        if (a_y <= 0) {
            cout << "Самолет " << i+1 << " не набирает высоту." << endl;
            continue;
        }
        
        double h = 1000; // заданная высота
        double t = sqrt(2 * h / a_y);
        
        cout << "Самолет " << i+1 << ": время набора = " << t << " с" << endl;
        
        if (t < best_time) {
            best_time = t;
            best_index = i;
        }
    }
    
    if (best_index != -1) {
        cout << "Самолет " << best_index+1 << " набирает высоту быстрее всех." << endl;
    }
    
    return 0;
}