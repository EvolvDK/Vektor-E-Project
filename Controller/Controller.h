//
// Created by elouarn on 28/03/24.
//

#ifndef RACE_SOFTWARE_CONTROLLER_H
#define RACE_SOFTWARE_CONTROLLER_H

#include <atomic>
#include <vector>
#include <optional>
#include "../LiDAR_Project/commonTypes.h"
#include "../LiDAR_Project/SharedDataBuffer.h"
#include <thread>
#include <condition_variable>


constexpr int DistanceDisparity = 1000;
constexpr double MaxSpeed = 28.0;
constexpr double MaxSteeringAngleDegrees = 16.0;
constexpr double CoefSpeed = 2500.0;
constexpr float DistLimitBackward = 500.0;

class Controller {
public:
    Controller(SharedDataBuffer<std::vector<LidarScanPoint>>& inputBuffer,
               SharedDataBuffer<LawCommand>& outputBuffer);

    ~Controller();

    void startController();
    void stopController();


private:
    SharedDataBuffer<std::vector<LidarScanPoint>>& inputBuffer_;
    SharedDataBuffer<LawCommand>& outputBuffer_;
    std::atomic<bool> isOngoing_;
    std::thread controllerThread_;
    std::condition_variable stopCondition_;

    std::optional<double> pendingSpeed_;
    std::optional<double> pendingSteeringAngle_;

    std::vector<float> lidarData;
    int ResolutionLidar, VisionField, VisionFieldSpeed, VisionFieldBackward, Margin;

    void setSpeed(double speedMps);
    void setDirectionDegree(double angleDegrees);
    void backward();
    void acquireLidarData();

    void autoControlLogic();

    void run();
};


#endif //RACE_SOFTWARE_CONTROLLER_H
