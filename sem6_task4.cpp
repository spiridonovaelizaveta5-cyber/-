#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

struct Waypoint {
    int id;
    double x, y, z, speed;
    std::string desc;
};

class WaypointManager {
private:
    std::vector<Waypoint> waypoints;
    int currentIndex = 0;

public:
    void addWaypoint(int id, double x, double y, double z, double speed, const std::string& desc) {
        waypoints.push_back({id, x, y, z, speed, desc});
    }

    bool saveRoute() {
        std::ofstream file("waypoints.bd");
        for (const auto& w : waypoints) {
            file << w.id << "," << w.x << "," << w.y << "," << w.z << "," << w.speed << "," << w.desc << "\n";
        }
        file.close();
        return true;
    }

    bool loadRoute() {
        std::ifstream file("waypoints.bd");
        waypoints.clear();
        int id;
        double x, y, z, speed;
        std::string desc;
        char comma;
        while (file >> id >> comma >> x >> comma >> y >> comma >> z >> comma >> speed >> comma && std::getline(file, desc)) {
            waypoints.push_back({id, x, y, z, speed, desc});
        }
        file.close();
        return !waypoints.empty();
    }

    double calculateTotalDistance() {
        double dist = 0.0;
        for (size_t i = 1; i < waypoints.size(); i++) {
            double dx = waypoints[i].x - waypoints[i-1].x;
            double dy = waypoints[i].y - waypoints[i-1].y;
            double dz = waypoints[i].z - waypoints[i-1].z;
            dist += sqrt(dx*dx + dy*dy + dz*dz);
        }
        return dist;
    }

    Waypoint getNextWaypoint() {
        if (currentIndex < waypoints.size()) return waypoints[currentIndex];
        return {};
    }

    bool checkWaypointReached(double x, double y, double z) {
        if (currentIndex >= waypoints.size()) return false;
        auto& w = waypoints[currentIndex];
        double dist = sqrt(pow(x - w.x, 2) + pow(y - w.y, 2) + pow(z - w.z, 2));
        if (dist < 10.0) { // tolerance
            currentIndex++;
            return true;
        }
        return false;
    }
};