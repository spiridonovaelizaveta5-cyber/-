#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>

class DataValidator {
private:
    struct Rule {
        double min, max;
    };
    std::map<std::string, Rule> rules;
    std::vector<std::string> errors;

public:
    void addValidationRule(const std::string& field, double min, double max) {
        rules[field] = {min, max};
    }

    bool validateCoordinates(double x, double y, double z) {
        bool valid = true;
        if (rules.count("x") && (x < rules["x"].min || x > rules["x"].max)) {
            errors.push_back("Координата x вне диапазона");
            valid = false;
        }
        if (rules.count("z") && z > 5000) {
            errors.push_back("Высота z превышает максимум 5000");
            valid = false;
        }
        return valid;
    }

    bool validateSpeed(double speed) {
        if (rules.count("speed") && speed > 300) {
            errors.push_back("Скорость превышает максимум 300");
            return false;
        }
        return true;
    }

    bool validateAcceleration(double acceleration) {
        if (rules.count("acceleration") && acceleration > 20) {
            errors.push_back("Ускорение превышает максимум 20");
            return false;
        }
        return true;
    }

    void generateValidationReport() {
        std::ofstream file("validation_report.txt");
        file << "Отчет валидации:\n";
        for (const auto& err : errors) {
            file << err << "\n";
        }
        file << "Общий результат: " << getValidationScore() * 100 << "% данных валидны\n";
        file.close();
    }

    double getValidationScore() {
        return errors.empty() ? 1.0 : 0.0; // simplified
    }
};