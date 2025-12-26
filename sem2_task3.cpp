#include <iostream>
using namespace std;

int main() {
    double m, L, D, T, g = 9.81;
    
    cout << "Введите массу самолета (кг): ";
    cin >> m;
    cout << "Введите подъемную силу L (Н): ";
    cin >> L;
    cout << "Введите сопротивление D (Н): ";
    cin >> D;
    cout << "Введите тягу двигателя T (Н): ";
    cin >> T;
    
    double a_x = (T - D) / m;
    double a_y = (L - m * g) / m;
    
    cout << "Ускорение по направлению движения: " << a_x << " м/с²" << endl;
    cout << "Вертикальное ускорение: " << a_y << " м/с²" << endl;
    
    return 0;
}