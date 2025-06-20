//
// Created by elouarn on 28/03/24.
//

#ifndef RACE_SOFTWARE_UART_H
#define RACE_SOFTWARE_UART_H

#include "../LiDAR_Project/SharedDataBuffer.h"
#include "../LiDAR_Project/commonTypes.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include "unistd.h"
#include "sys/fcntl.h"
#include "termios.h"

// Define Constants
const char *uart_target = "/dev/ttyTHS1";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       B115200

class Uart {
public:
    Uart(SharedDataBuffer<LawCommand>& commandBuffer);
    ~Uart();

    void startSender();
    void stopSender();

    unsigned char serial_message[NSERIAL_CHAR];

private:
    int fid;

    SharedDataBuffer<LawCommand>& commandBuffer_;

    std::atomic<bool> isOngoing_;
    std::thread senderThread_;
    std::condition_variable stopCondition_;

    void run();
    void sendCommand(const LawCommand& command);

    void sendUart(unsigned char *msg);
    bool sendUart_fb(unsigned char *msg);
    void readUart();
    void closeUart();
};


#endif //RACE_SOFTWARE_UART_H
