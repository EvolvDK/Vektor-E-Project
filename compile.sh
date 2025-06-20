#g++ main.cpp -lJetsonGPIO -lpthread

# g++-10.1 -o HammerTime main.cpp LiDAR_Project/RpLidarInterface.cpp Controller/Controller.cpp JetsonNanoUart/Uart.cpp VektorSystem.cpp -Iinclude -Llib LiDAR_Project/lib/libsl_lidar_sdk.a -lJetsonGPIO -lpthread -ltbb -std=c++20

g++ -o HammerTime main.cpp LiDAR_Project/RpLidarInterface.cpp Controller/Controller_Txt.cpp JetsonNanoUart/Uart_Txt.cpp VektorSystem.cpp -Iinclude -Llib LiDAR_Project/lib/libsl_lidar_sdk.a -lJetsonGPIO -lpthread -ltbb -std=c++20
