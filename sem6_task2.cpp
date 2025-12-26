#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

class TargetManager {
private:
    struct Target {
        int id;
        std::string name;
        double x, y, z, priority, distance;
    };
    std::vector<Target> targets;
    std::string filename = "targets.bd";

public:
    void addTarget(int id, const std::string& name, double x, double y, double z,
                   double priority, double distance) {
        targets.push_back({id, name, x, y, z, priority, distance});
    }

    bool removeTarget(int target_id) {
        auto it = std::remove_if(targets.begin(), targets.end(),
                                 [target_id](const Target& t) { return t.id == target_id; });
        if (it != targets.end()) {
            targets.erase(it, targets.end());
            return true;
        }
        return false;
    }

    void saveTargetsToFile() {
        std::ofstream file(filename);
        for (const auto& t : targets) {
            file << t.id << "," << t.name << "," << t.x << "," << t.y << "," << t.z << ","
                 << t.priority << "," << t.distance << "\n";
        }
        file.close();
    }

    void loadTargetsFromFile() {
        std::ifstream file(filename);
        targets.clear();
        int id;
        std::string name;
        double x, y, z, pr, dist;
        char comma;
        while (file >> id >> comma && std::getline(file, name, ',') &&
               file >> x >> comma >> y >> comma >> z >> comma >> pr >> comma >> dist) {
            targets.push_back({id, name, x, y, z, pr, dist});
        }
        file.close();
    }

    std::vector<Target> getHighPriorityTargets(double min_priority) {
        std::vector<Target> high;
        std::copy_if(targets.begin(), targets.end(), std::back_inserter(high),
                     [min_priority](const Target& t) { return t.priority >= min_priority; });
        return high;
    }

    void sortByDistance() {
        std::sort(targets.begin(), targets.end(),
                  [](const Target& a, const Target& b) { return a.distance < b.distance; });
    }
};