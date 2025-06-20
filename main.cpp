//
// Created by elouarn on 28/03/24.
//

#include "VektorSystem.h"
#include <iostream>
#include <string>
#include <thread>
#include <csignal>
#include <chrono>

volatile std::sig_atomic_t signalReceived;

// Signal handler function
void signalHandler(int) {
    signalReceived = true;
}

int main() {
    std::signal(SIGINT, signalHandler);

    // Initialize system parameters
    const char* port = "/dev/ttyUSB0";
    int baudRate = 256000;

    // Create the LidarMappingSystem
    VektorSystem vektorSystem(port, baudRate);

    // Start the system
    vektorSystem.startSystem();

/*    std::thread userInputThread([&lidarSystem]() {
        char ch;
        while (true) {
            std::cin >> ch;
            lidarSystem.handleUserInput(ch);
        }
    });*/

    while (vektorSystem.isRunning && !signalReceived) {
        //std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (signalReceived) {
        vektorSystem.stopSystem();
    }

    // Ensure the input thread finishes
/*    if (userInputThread.joinable()) {
        userInputThread.join();
    }*/

    return 0;
}