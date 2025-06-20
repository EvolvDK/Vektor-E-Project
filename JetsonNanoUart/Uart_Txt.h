//
// Created by elouarn on 29/03/24.
//

#ifndef RACE_SOFTWARE_UART_TXT_H
#define RACE_SOFTWARE_UART_TXT_H

#include "../LiDAR_Project/commonTypes.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include "unistd.h"
#include "sys/fcntl.h"
#include "termios.h"
#include <condition_variable>

const char *uart_target = "/dev/ttyTHS1";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       B115200

class Uart_Txt {
public :
    Uart_Txt();
    ~Uart_Txt();

    void startSender();
    void stopSender();

    unsigned char serial_message[NSERIAL_CHAR];

private:
    int fid;

    std::atomic<bool> isOngoing_;
    std::thread senderThread_;
    std::condition_variable stopCondition_;

    void run();
    void sendCommand(const LawCommand& command);

    std::vector<float> parseValues(const std::string& line);
    void clearCommandsFile(const std::string& filename);
    std::vector<LawCommand> readCommandsFromFile(const std::string& filename);


    void sendUart(unsigned char *msg);
    bool sendUart_fb(unsigned char *msg);
    void readUart();
    void closeUart();
};


#endif //RACE_SOFTWARE_UART_TXT_H
