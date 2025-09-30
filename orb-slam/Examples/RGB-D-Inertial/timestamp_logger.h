#ifndef TIMESTAMP_LOGGER_H
#define TIMESTAMP_LOGGER_H

#include <fstream>
#include <chrono>
#include <iomanip>
#include <iostream>

class TimestampLogger {
private:
    std::ofstream csv_file_;
    bool logging_enabled_;
    std::string filename_;
    
public:
    TimestampLogger(const std::string& filename = "timestamp_log.csv") 
        : filename_(filename), logging_enabled_(false) {}
    
    ~TimestampLogger() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }
    
    bool initialize() {
        csv_file_.open(filename_, std::ios::out);
        if (!csv_file_.is_open()) {
            std::cout << "[ERROR] Could not open timestamp log file: " << filename_ << std::endl;
            return false;
        }
        
        // Write CSV header
        csv_file_ << "timestamp,type" << std::endl;
        logging_enabled_ = true;
        
        std::cout << "[TIMESTAMP_LOG] Initialized timestamp logging to: " << filename_ << std::endl;
        return true;
    }
    
    void logTimestamp(double timestamp, int type) {
        if (!logging_enabled_ || !csv_file_.is_open()) {
            return;
        }
        
        csv_file_ << std::fixed << std::setprecision(6) 
                 << timestamp << "," << type << std::endl;
        csv_file_.flush(); // Ensure immediate write
    }
    
    void logColorFrame(double timestamp) {
        logTimestamp(timestamp, 0); // Type 0 = Color/RGB
    }
    
    void logDepthFrame(double timestamp) {
        logTimestamp(timestamp, 1); // Type 1 = Depth
    }
    
    void logAccelData(double timestamp) {
        logTimestamp(timestamp, 2); // Type 2 = Accelerometer
    }
    
    void logGyroData(double timestamp) {
        logTimestamp(timestamp, 3); // Type 3 = Gyroscope
    }
    
    void setEnabled(bool enabled) {
        logging_enabled_ = enabled;
    }
    
    bool isEnabled() const {
        return logging_enabled_;
    }
};

#endif // TIMESTAMP_LOGGER_H