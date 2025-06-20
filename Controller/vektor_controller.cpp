// File:          vektor_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <iostream>
#include <memory>
#include <cmath>
#include <vector>
#include <chrono>
#include <fstream>
#include <string>
#include <algorithm>
#include "../LiDAR_Project/SharedDataBuffer.h"
#include <atomic>


// All the webots classes are defined in the "webots" namespace
using namespace std::chrono;




class VehicleController : public Driver {
private:
    int ResolutionLidar, VisionField, VisionFieldSpeed, VisionFieldBackward, Margin;
    bool modeAuto = true;
    std::vector<float> lidarData;
    bool loadData;
    std::vector<int> speedData = {0};
    std::vector<int> steeringAngleData = {0};
    steady_clock::time_point tick;
    const milliseconds period = milliseconds(static_cast<int>(1000.0/15.0));
    SharedDataBuffer<std::vector<float>>* lidarDataBuffer;


    void setSpeed(double speedMps){
        double speedKmh = speedMps * 3.6;
        speedKmh = std::clamp(speedKmh, 0.0, MaxSpeed);
        if(loadData) speedData.push_back(static_cast<int>(speedKmh));
    }

    void setDirectionDegree(double angleDegrees){
        float clampedAngleDeg = std::clamp(angleDegrees, -MaxSteeringAngleDegrees, MaxSteeringAngleDegrees);
        float angleRad = -clampedAngleDeg * M_PI/180.0;
        if(loadData) steeringAngleData.push_back(static_cast<int>(clampedAngleDeg));
    }

    void backward(){
        if(loadData) speedData.push_back(-1);
    }

    void acquireLidarData() {
        lidarDataBuffer->waitForData();
        auto sharedData = lidarDataBuffer->getData();
        lidarData.clear();
        if (sharedData){
            for (const auto& scanPoint : *sharedData){
                lidarData.push_back(scanPoint.distance);
            }
        }
    }

    void autoControlLogic(){
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
    std::string vectorToString(const std::vector<T>& vec) {
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

    void writeDataToFile(const std::string& filename, const std::vector<int>& speedData, const std::vector<int>& angleData) {
        std::ofstream fichier(filename, std::ofstream::out);
        if (fichier.is_open()) {
            fichier << vectorToString(speedData) << "\n";
            fichier << vectorToString(angleData) << "\n";
            fichier.close();
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }
    }


public :
    VehicleController(SharedDataBuffer<std::vector<float>* lidarDataBuffer) : Driver(), lidarDataBuffer(lidarDataBuffer){
        ResolutionLidar = 8192;
        VisionField = ResolutionLidar / 4; // 90 degrees
        VisionFieldSpeed = ResolutionLidar / 36; // 10 degrees
        VisionFieldBackward = ResolutionLidar / 72; // 5 degrees
        Margin = ResolutionLidar / 12; // 30 degrees

        lidarData.resize(ResolutionLidar, 0);
        tick = steady_clock::now();
        std::atomic<bool> isOngoing;
    }

    void run(){
        std::cout<<"Click on the 3D view to start\n"<<std::endl;
        std::cout<<"a for auto mode, n to stop\n"<<std::endl;
        while(isOngoing){
            loadData = false;
            auto now = steady_clock::now();
            if (duration_cast<milliseconds>(now - tick) >= period) {
                loadData = true;
                tick = now;
            }
            acquireLidarData();
            if (!modeAuto){
                setDirectionDegree(0.0f);
                setSpeed(0.0f);
            }else{
                autoControlLogic();
            }
        }
        writeDataToFile("data.txt", speedData, steeringAngleData);
    }

};


int main(int argc, char **argv) {
  VehicleController controller;
  std::cout<<"test"<<std::endl;
  controller.run();
  return 0;
}