//
// Created by elouarn on 29/03/24.
//

#include "Controller_Txt.h"
#include <algorithm>
#include <execution>
#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>


Controller_Txt::Controller_Txt(SharedDataBuffer<std::vector<LidarScanPoint>>& inputBuffer)
: inputBuffer_(inputBuffer), isOngoing_(false) {
    ResolutionLidar = 8192;
    VisionField = ResolutionLidar / 4;
    VisionFieldSpeed = ResolutionLidar / 36;
    VisionFieldBackward = ResolutionLidar / 72;
    Margin = ResolutionLidar / 12;
}

Controller_Txt::~Controller_Txt() {
    if(isOngoing_) stopController();
}

void Controller_Txt::startController() {
    if (!isOngoing_) {
        isOngoing_ = true;
        dataFile.open(dataFilename, std::ofstream::out | std::ios::app);
        if (!dataFile.is_open()) {
            std::cerr << "Failed to open file: " << dataFilename << std::endl;
            return; // Handle error appropriately
        }
        inputBuffer_.resetInterrupt();
        controllerThread_ = std::thread(&Controller_Txt::run, this);
    }
}


void Controller_Txt::stopController() {
    if (isOngoing_) {
        isOngoing_ = false;
        inputBuffer_.interruptWait();
        stopCondition_.notify_one(); // Notify the Controller_Txt thread to stop
        if (controllerThread_.joinable()) {
            controllerThread_.join();
        }
        dataFile.close();
    }
}

void Controller_Txt::setSpeed(double speedMps){
    double speedKmh = speedMps * 3.6;
    speedKmh = std::clamp(speedKmh, 0.0, MaxSpeed);
    speedData.push_back(speedKmh);
}

void Controller_Txt::setDirectionDegree(double angleDegrees){
    double clampedAngleDeg = std::clamp(angleDegrees, -MaxSteeringAngleDegrees, MaxSteeringAngleDegrees);
    steeringAngleData.push_back(clampedAngleDeg);
}

void Controller_Txt::backward(){
    double backwardSpeedKmh = -1.0;
    speedData.push_back(backwardSpeedKmh);
}

void Controller_Txt::acquireLidarData() {
    inputBuffer_.waitForData();
    auto sharedData = inputBuffer_.getData();
    lidarData.clear();
    if (sharedData){
        for (const auto& scanPoint : *sharedData){
            lidarData.push_back(scanPoint.distance);
        }
    }
}

void Controller_Txt::autoControlLogic(){
    float distanceMax = lidarData[0];
    float distanceMaxSpeed = lidarData[0];
    float distanceMaxBackward = lidarData[0];
    int indexDistanceMax = 0;
    int indexDistanceMaxBackward = 0;
    double anglePoint = 0;

    // Process disparities in Lidar data
    for (int j = 0; j < ResolutionLidar; ++j) {
        // Near to far transition
        if (lidarData[j] < lidarData[(j + 1) % ResolutionLidar] - DistanceDisparity) {
            for (int l = 1; l < Margin; ++l) {
                lidarData[(j + l) % ResolutionLidar] = lidarData[j];
            }
            j += Margin % ResolutionLidar; // Skip ahead
        }
            // Far to near transition
        else if (lidarData[j] > lidarData[(j + 1) % ResolutionLidar] + DistanceDisparity) {
            for (int l = 1; l <= Margin; ++l) {
                lidarData[(j - l + ResolutionLidar) % ResolutionLidar] = lidarData[(j + 1) % ResolutionLidar];
            }
        }
    }

    // Calculate max distances within vision fields and adjust speed accordingly
    for (int k = 0; k < ResolutionLidar; ++k) {
        if (k < VisionField || k > (ResolutionLidar - VisionField)) {
            if (lidarData[k] > distanceMax) {
                distanceMax = lidarData[k];
                indexDistanceMax = k;
            }
        }
        if (k < VisionFieldSpeed || k > (ResolutionLidar - VisionFieldSpeed)) {
            distanceMaxSpeed = std::max(distanceMaxSpeed, lidarData[k]);
        }
        if (k < VisionFieldBackward || k > (ResolutionLidar - VisionFieldBackward)) {
            if (lidarData[k] > distanceMaxBackward) {
                distanceMaxBackward = lidarData[k];
                indexDistanceMaxBackward = k;
            }
        }
    }

    // Wall detection logic
    if (distanceMaxBackward < DistLimitBackward) {
        anglePoint = indexDistanceMaxBackward < VisionFieldBackward ? -22.5 : 22.5;
        backward();
    } else {
        float speedMps = distanceMaxSpeed / CoefSpeed;
        setSpeed(speedMps);

        if (indexDistanceMax < VisionField || indexDistanceMax > (ResolutionLidar - VisionField)) {
            anglePoint = (indexDistanceMax < VisionField) ? indexDistanceMax : -ResolutionLidar + indexDistanceMax;
        }
    }

    // Convert angle points to degrees, avoiding division by zero
    double angleDegrees = anglePoint != 0 ? std::copysign(360.0 / std::abs(ResolutionLidar / anglePoint), anglePoint) : 0;
    setDirectionDegree(angleDegrees);
}

template<typename T>
std::string Controller_Txt::vectorToString(const std::vector<T>& vec) {
    std::string result = "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        result += std::to_string(vec[i]);
        if (i < vec.size() - 1) {
            result += ", ";
        }
    }
    result += "]";
    return result;
}

void Controller_Txt::writeDataToFile() {
    if (dataFile.is_open()) {
        if (!speedData.empty() && !steeringAngleData.empty()) {
            dataFile << vectorToString(speedData.back()) << "\n";
            dataFile << vectorToString(steeringAngleData.back()) << "\n";
            speedData.clear();
            steeringAngleData.clear();
        }
    }else{
        std::cerr << "File stream is not open for writing." << std::endl;
        return;
    }

}

void Controller_Txt::run(){
    while(isOngoing_.load()){
        acquireLidarData();
        autoControlLogic();
        writeDataToFile();
    }
}
