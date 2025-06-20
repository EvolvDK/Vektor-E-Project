//
// Created by elouarn on 28/03/24.
//

#include "VektorSystem.h"
#include <iostream>

VektorSystem::VektorSystem(const char *port, int baudRate)
    : lidarInterface(std::make_unique<RpLidarInterface>(port, baudRate, rawBuffer)),
      //controller(std::make_unique<Controller>(rawBuffer, commandBuffer)),
      //uart(std::make_unique<Uart>(commandBuffer)),
      controller(std::make_unique<Controller_Txt>(rawBuffer)),
      uart(std::make_unique<Uart_Txt>()),
      isRunning(false){
}

VektorSystem::~VektorSystem() {
    if(isRunning) stopSystem();
}

void VektorSystem::monitorLiDARConnection() {
    while (isRunning) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Adjust as needed

        auto now = std::chrono::steady_clock::now();
        std::shared_lock<std::shared_mutex> lock(lidarInterface->getOperationMutex());
        auto lastOperation = lidarInterface->getLastSuccessfulOperation();
        lock.unlock();

        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastOperation).count() > CONNECTION_TIMEOUT) {
            std::cerr << "Potential LiDAR disconnection detected." << std::endl;
            if (!lidarInterface->attemptReconnection()) {
                std::cerr << "Unable to re-establish LiDAR connection. Stopping system." << std::endl;
                stopSystem();
                break;
            }
        }
    }
}


void VektorSystem::startSystem() {
    if (isRunning) {
        std::cerr << "System is already running." << std::endl;
        return;
    }
    if (!lidarInterface->connect()) {
        std::cerr << "Failed to connect to LiDAR." << std::endl;
        return;
    }
    isRunning = true;
    connectionMonitorThread = std::thread(&VektorSystem::monitorLiDARConnection, this);
    lidarInterface->startAcquisition();
    controller->startController();
    uart->startSender();
}

void VektorSystem::stopSystem() {
    if (!isRunning) {
        std::cerr << "System is already stopped." << std::endl;
        return;
    }
    isRunning = false;
    if(uart){
        uart->stopSender();
        std::cout << "Uart stopped" << std::endl;
    }
    if(controller) {
        controller->stopController();
        std::cout << "Controller stopped" << std::endl;
    }
    if(lidarInterface){
        lidarInterface->disconnect();
        lidarInterface->stopAcquisition();
        std::cout << "Acquisition stopped" << std::endl;
    }
    stopCondition_.notify_one();
    if(connectionMonitorThread.joinable()){
        connectionMonitorThread.join();
        std::cout << "Connection Monitor stopped" << std::endl;
    }
}
