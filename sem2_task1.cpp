#include <iostream>
using namespace std;

int main() {
    double rho, V, S, C_L;
    
    cout << "Введите плотность воздуха (кг/м³): ";
    cin >> rho;
    cout << "Введите скорость (м/с): ";
    cin >> V;
    cout << "Введите площадь крыла (м²): ";
    cin >> S;
    cout << "Введите коэффициент подъемной силы C_L: ";
    cin >> C_L;
    
    double L = 0.5 * rho * V * V * S * C_L;
    
    cout << "Подъемная сила L = " << L << " Н" << endl;
    
    return 0;
}