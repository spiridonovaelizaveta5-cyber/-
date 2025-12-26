#include <iostream>
using namespace std;

double calculateDrag(double rho, double V, double S, double C_D) {
    return 0.5 * rho * V * V * S * C_D;
}

int main() {
    double rho, V, S, C_D;
    
    cout << "Введите плотность воздуха (кг/м³): ";
    cin >> rho;
    cout << "Введите скорость (м/с): ";
    cin >> V;
    cout << "Введите площадь (м²): ";
    cin >> S;
    cout << "Введите коэффициент сопротивления C_D: ";
    cin >> C_D;
    
    double D = calculateDrag(rho, V, S, C_D);
    
    cout << "Аэродинамическое сопротивление D = " << D << " Н" << endl;
    
    return 0;
}