//
// Created by elouarn on 28/03/24.
//

#include "Controller.h"
#include <algorithm>
#include <execution>
#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>
#include <fstream>
#include <string>


Controller::Controller(SharedDataBuffer<std::vector<LidarScanPoint>>& inputBuffer,
                       SharedDataBuffer<LawCommand>& outputBuffer)
                       : inputBuffer_(inputBuffer), outputBuffer_(outputBuffer), isOngoing_(false) {
    ResolutionLidar = 8192;
    VisionField = ResolutionLidar / 4;
    VisionFieldSpeed = ResolutionLidar / 36;
    VisionFieldBackward = ResolutionLidar / 72;
    Margin = ResolutionLidar / 12;
}

Controller::~Controller() {
    if(isOngoing_) stopController();
}

void Controller::startController() {
    if (!isOngoing_) {
        isOngoing_ = true;
        inputBuffer_.resetInterrupt();
        controllerThread_ = std::thread(&Controller::run, this);
    }
}


void Controller::stopController() {
    if (isOngoing_) {
        isOngoing_ = false;
        inputBuffer_.interruptWait();
        stopCondition_.notify_one(); // Notify the controller thread to stop
        if (controllerThread_.joinable()) {
            controllerThread_.join();
        }
    }
}

void Controller::setSpeed(double speedMps){
    double speedKmh = speedMps * 3.6;
    speedKmh = std::clamp(speedKmh, 0.0, MaxSpeed);
    pendingSpeed_ = speedKmh;
    if(pendingSteeringAngle_) {
        LawCommand command{.speed = pendingSpeed_.value(), .steeringAngle = pendingSteeringAngle_.value()};
        outputBuffer_.addData(command);
        // Reset the pending values
        pendingSpeed_.reset();
        pendingSteeringAngle_.reset();
    }
}

void Controller::setDirectionDegree(double angleDegrees){
    double clampedAngleDeg = std::clamp(angleDegrees, -MaxSteeringAngleDegrees, MaxSteeringAngleDegrees);
    pendingSteeringAngle_ = clampedAngleDeg;
    if(pendingSpeed_) {
        LawCommand command{.speed = pendingSpeed_.value(), .steeringAngle = pendingSteeringAngle_.value()};
        outputBuffer_.addData(command);
        // Reset the pending values
        pendingSpeed_.reset();
        pendingSteeringAngle_.reset();
    }
}

void Controller::backward(){
    pendingSpeed_ = -1.0;
    if(pendingSteeringAngle_) {
        LawCommand command{.speed = pendingSpeed_.value(), .steeringAngle = pendingSteeringAngle_.value()};
        outputBuffer_.addData(command);
        // Reset the pending values
        pendingSpeed_.reset();
        pendingSteeringAngle_.reset();
    }
}

void Controller::acquireLidarData() {
    inputBuffer_.waitForData();
    auto sharedData = inputBuffer_.getData();
    lidarData.clear();
    if (sharedData){
        for (const auto& scanPoint : *sharedData){
            lidarData.push_back(scanPoint.distance);
        }
    }
}

void Controller::autoControlLogic(){
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

void Controller::run(){
    while(isOngoing_.load()){
        acquireLidarData();
        autoControlLogic();
    }
}