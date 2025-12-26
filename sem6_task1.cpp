#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

class TrajectoryLogger {
private:
    struct Point {
        double x, y, z, speed, time;
    };
    std::vector<Point> points;
    std::string filename;

public:
    TrajectoryLogger(const std::string& fname) : filename(fname) {}

    void addPoint(double x, double y, double z, double speed, double time) {
        points.push_back({x, y, z, speed, time});
    }

    bool saveToCSV() {
        std::ofstream file(filename);
        if (!file.is_open()) return false;
        file << "time,x,y,z,speed\n";
        for (const auto& p : points) {
            file << p.time << "," << p.x << "," << p.y << "," << p.z << "," << p.speed << "\n";
        }
        file.close();
        return true;
    }

    bool loadFromCSV() {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string header;
        std::getline(file, header);
        double t, x, y, z, s;
        char comma;
        points.clear();
        while (file >> t >> comma >> x >> comma >> y >> comma >> z >> comma >> s) {
            points.push_back({x, y, z, s, t});
        }
        file.close();
        return true;
    }

    double calculateTotalDistance() {
        double dist = 0.0;
        for (size_t i = 1; i < points.size(); i++) {
            double dx = points[i].x - points[i-1].x;
            double dy = points[i].y - points[i-1].y;
            double dz = points[i].z - points[i-1].z;
            dist += sqrt(dx*dx + dy*dy + dz*dz);
        }
        return dist;
    }

    double findMaxSpeed() {
        double maxSpeed = 0.0;
        for (const auto& p : points) {
            if (p.speed > maxSpeed) maxSpeed = p.speed;
        }
        return maxSpeed;
    }

    void printStatistics() {
        std::cout << "Total points: " << points.size() << "\n";
        std::cout << "Total distance: " << calculateTotalDistance() << "\n";
        std::cout << "Max speed: " << findMaxSpeed() << "\n";
    }
};