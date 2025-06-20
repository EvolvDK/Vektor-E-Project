//
// Created by elouarn on 28/03/24.
//

#ifndef RACE_SOFTWARE_VEKTORSYSTEM_H
#define RACE_SOFTWARE_VEKTORSYSTEM_H

#include "LiDAR_Project/RpLidarInterface.h"
// #include "Controller/Controller.h"
// #include "JetsonNanoUart/Uart.h"
#include "Controller/Controller_Txt.h"
#include "JetsonNanoUart/Uart_Txt.h"
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

#define CONNECTION_TIMEOUT 3000

class VektorSystem {
public:
    VektorSystem(const char* port, int baudRate);
    ~VektorSystem();

    // Disable copy and assignment
    VektorSystem(const VektorSystem&) = delete;
    VektorSystem& operator=(const VektorSystem&) = delete;

    // System control methods
    void startSystem();
    void stopSystem();

    std::atomic<bool> isRunning;

private:
    void monitorLiDARConnection();

    std::condition_variable stopCondition_;

    // Components of the mapping system
    std::unique_ptr<RpLidarInterface> lidarInterface;
    //std::unique_ptr<Controller> controller;
    //std::unique_ptr<Uart> uart;
    std::unique_ptr<Controller_Txt> controller;
    std::unique_ptr<Uart_Txt> uart;

    SharedDataBuffer<std::vector<LidarScanPoint>> rawBuffer;
    SharedDataBuffer<LawCommand> commandBuffer;

    std::thread connectionMonitorThread;
};


#endif //RACE_SOFTWARE_VEKTORSYSTEM_H
