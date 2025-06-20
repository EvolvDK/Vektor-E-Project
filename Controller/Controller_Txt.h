//
// Created by elouarn on 29/03/24.
//

#ifndef RACE_SOFTWARE_CONTROLLER_TXT_H
#define RACE_SOFTWARE_CONTROLLER_TXT_H


#include <atomic>
#include <vector>
#include <optional>
#include "../LiDAR_Project/commonTypes.h"
#include "../LiDAR_Project/SharedDataBuffer.h"
#include <thread>
#include <condition_variable>
#include <string>
#include <fstream>


constexpr int DistanceDisparity = 1000;
constexpr double MaxSpeed = 28.0;
constexpr double MaxSteeringAngleDegrees = 16.0;
constexpr double CoefSpeed = 2500.0;
constexpr float DistLimitBackward = 500.0;

class Controller_Txt {
public:
    Controller_Txt(SharedDataBuffer<std::vector<LidarScanPoint>>& inputBuffer);


    ~Controller_Txt();

    void startController();
    void stopController();


private:
    SharedDataBuffer<std::vector<LidarScanPoint>>& inputBuffer_;
    std::atomic<bool> isOngoing_;
    std::thread controllerThread_;
    std::condition_variable stopCondition_;

    std::vector<double> speedData;
    std::vector<double> steeringAngleData;

    std::string dataFilename = "../lawCommands.txt";
    std::ofstream dataFile;

    std::vector<float> lidarData;
    int ResolutionLidar, VisionField, VisionFieldSpeed, VisionFieldBackward, Margin;

    void setSpeed(double speedMps);
    void setDirectionDegree(double angleDegrees);
    void backward();
    void acquireLidarData();

    void writeDataToFile();

    template<typename T>
    std::string vectorToString(const std::vector<T>& vec);

    void autoControlLogic();

    void run();
};


#endif //RACE_SOFTWARE_CONTROLLER_TXT_H
