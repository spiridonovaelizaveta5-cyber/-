#include <iostream>
#include <fstream>
#include <vector>
#include <string>

struct TelemetryData {
    double time, altitude, speed, heading, fuel;
};

class TelemetryLogger {
private:
    std::string baseFilename = "telemetry";
    int fileCounter = 1;
    const size_t maxEntries = 1000;
    std::vector<TelemetryData> buffer;

public:
    bool logData(double time, double altitude, double speed, double heading, double fuel) {
        buffer.push_back({time, altitude, speed, heading, fuel});
        if (buffer.size() >= maxEntries) {
            rotateFileIfNeeded();
        }
        return true;
    }

    void rotateFileIfNeeded() {
        if (buffer.empty()) return;
        std::string filename = baseFilename + "_" + std::to_string(fileCounter++) + ".bin";
        std::ofstream file(filename, std::ios::binary);
        file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(TelemetryData));
        file.close();
        buffer.clear();
    }

    std::vector<TelemetryData> readLogFile(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) return {};
        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        size_t count = size / sizeof(TelemetryData);
        std::vector<TelemetryData> data(count);
        file.read(reinterpret_cast<char*>(data.data()), size);
        file.close();
        return data;
    }

    void printLogSummary() {
        double totalTime = 0, avgAlt = 0, avgSpeed = 0;
        for (const auto& d : buffer) {
            totalTime += d.time;
            avgAlt += d.altitude;
            avgSpeed += d.speed;
        }
        std::cout << "Entries: " << buffer.size() << "\n";
        std::cout << "Avg Altitude: " << avgAlt / buffer.size() << "\n";
        std::cout << "Avg Speed: " << avgSpeed / buffer.size() << "\n";
    }
};