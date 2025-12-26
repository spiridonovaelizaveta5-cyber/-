#include <iostream>
#include <cmath>
using namespace std;

int main() {
    double h, a_y;
    
    cout << "Введите высоту набора h (м): ";
    cin >> h;
    cout << "Введите вертикальное ускорение a_y (м/с²): ";
    cin >> a_y;
    
    if (h <= 0 || a_y <= 0) {
        cout << "Ошибка: высота и ускорение должны быть положительными." << endl;
        return 1;
    }
    
    double t = sqrt(2 * h / a_y);
    
    cout << "Время набора высоты: " << t << " с" << endl;
    
    return 0;
}