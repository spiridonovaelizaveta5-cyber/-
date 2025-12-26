#include <iostream>
using namespace std;

int main() {
    double a;
    
    cout << "Введите ускорение самолета (м/с²): ";
    cin >> a;
    
    if (a > 0.5) {
        cout << "Режим: набор высоты" << endl;
    } else if (a >= 0 && a <= 0.5) {
        cout << "Режим: горизонтальный полет" << endl;
    } else if (a < 0) {
        cout << "Режим: снижение" << endl;
    } else {
        cout << "Некорректный ввод" << endl;
    }
    
    return 0;
}