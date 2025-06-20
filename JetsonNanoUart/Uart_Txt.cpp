//
// Created by elouarn on 29/03/24.
//

#include "Uart_Txt.h"
#include <iostream>
#include <fstream>
#include <stdio.h>

Uart_Txt::Uart_Txt()
        : isOngoing_(false) {
    int ii, jj, kk;

    // SETUP SERIAL WORLD
    struct termios  port_options;   // Create the structure
    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port

    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR   - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //	    O_NDELAY / O_NONBLOCK (same function)
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    fid = open(uart_target, O_RDWR | O_NOCTTY );

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(1000000);  // 1 sec delay

    if (fid == -1)
    {
        printf("**Error - Unable to open UART**.  \n=>Ensure it is not in use by another application\n=>Ensure proper privilages are granted to accsess /dev/.. by run as a sudo\n");
    }

    //------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE: - CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD  - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size
    port_options.c_cflag |=  CS8;               // Set the data bits = 8
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
    port_options.c_oflag &= ~OPOST;                           // No Output Processing
    port_options.c_lflag = 0;               //  enable raw input instead of canonical,

    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly

    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 )
    {
        printf("\nERROR in Setting port attributes");
    }
    else
    {
        printf("\nSERIAL Port Good to Go.\n");
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(500000);   // 0.5 sec delay
}


Uart_Txt::~Uart_Txt() {
    stopSender();
}

void Uart_Txt::startSender() {
    if (!isOngoing_) {
        isOngoing_ = true;
        senderThread_ = std::thread(&Uart_Txt::run, this);
    }
}

void Uart_Txt::stopSender() {
    if (isOngoing_) {
        closeUart();
        isOngoing_ = false;
        stopCondition_.notify_one(); // Notify the sender thread to stop
        if (senderThread_.joinable()) {
            senderThread_.join();
        }
    }
}

void Uart_Txt::sendUart(unsigned char *msg){
    //--------------------------------------------------------------
    // TRANSMITTING BYTES
    //--------------------------------------------------------------
    unsigned char tx_buffer[20];
    unsigned char *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];

    // so that i have the number of bytes to write
    // by doing p_tx - tx
    for (int i = 0; i < 20; i++) {
        *p_tx_buffer++ = msg[i];
    }
    //printf("%x%x%x%x%x\n",p_tx_buffer[0],p_tx_buffer[1],p_tx_buffer[2],p_tx_buffer[3],p_tx_buffer[4]);
    //printf("fid 1=%d\n", fid );

    if (fid != -1)
    {
        int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write

        //usleep(1000);   // .001 sec delay

        //printf("Count = %d\n", count);

        if (count < 0)  printf("UART TX error\n");
    }
    //usleep(1000000);  // 1 sec delay
}

bool  Uart_Txt::sendUart_fb(unsigned char *msg){
    //--------------------------------------------------------------
    // TRANSMITTING BYTES WITH LOGICAL FEED BACK
    //--------------------------------------------------------------
    unsigned char tx_buffer[20];
    unsigned char *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];

    // so that i have the number of bytes to write
    // by doing p_tx - tx
    for (int i = 0; i < 20; i++) {
        *p_tx_buffer++ = msg[i];
    }
    //printf("%x%x%x%x%x\n",p_tx_buffer[0],p_tx_buffer[1],p_tx_buffer[2],p_tx_buffer[3],p_tx_buffer[4]);
    printf("fid 1=%d\n", fid );

    if (fid != -1)
    {
        int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
        usleep(1000);   // .001 sec delay
        printf("Count = %d\n", count);
        if (count < 0)
        {
            printf("UART TX error\n");
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
    usleep(1000000);  // 1 sec delay
}
void Uart_Txt::readUart(){

    //--------------------------------------------------------------
    // RECEIVING BYTES - AND BUILD MESSAGE RECEIVED
    //--------------------------------------------------------------
    unsigned char rx_buffer[VMINX];
    bool          pickup = true;
    int ii;
    int           rx_length;
    int           nread = 0;

    tcflush(fid, TCIOFLUSH);
    usleep(1000);   // .001 sec delay
    printf("Ready to receive message.\n");
    for (ii=0; ii<NSERIAL_CHAR; ii++)  serial_message[ii]=' ';

    while (pickup && fid != -1)
    {
        nread++;
        rx_length = read(fid, (void*)rx_buffer, VMINX);   // Filestream, buffer to store in, number of bytes to read (max)
        printf("Event %d, rx_length=%d, Read=%s\n",  nread, rx_length, rx_buffer );
        if (rx_length < 0)
        {
            //An error occured (will occur if there are no bytes)
        }
        if (rx_length == 0)
        {
            //No data waiting
        }
        if (rx_length>=0)
        {
            if (nread<=NSERIAL_CHAR){
                serial_message[nread-1] = rx_buffer[0];   // Build message 1 character at a time
                printf("%x ",serial_message[nread-1]);
            }
            if (rx_buffer[0]=='#')   pickup=false;                               // # symbol is terminator
        }
    }
    printf("\nMessage Received:");
}

void Uart_Txt::closeUart(){
    //-------------------------------------------
    //  CLOSE THE SERIAL PORT
    //-------------------------------------------
    close(fid);
}

std::vector<LawCommand> readCommandsFromFile(const std::string& filename) {
    std::vector<LawCommand> commands;
    std::ifstream file(filename);
    std::string line;

    // Read speed values
    std::getline(file, line);
    std::vector<float> speeds = parseValues(line);

    // Read steering angle values
    std::getline(file, line);
    std::vector<float> steeringAngles = parseValues(line);

    // Combine speed and steering angles into LawCommand objects
    for (size_t i = 0; i < speeds.size() && i < steeringAngles.size(); ++i) {
        commands.push_back({speeds[i], steeringAngles[i]});
    }

    return commands;
}

void clearCommandsFile(const std::string& filename) {
    std::ofstream ofs(filename, std::ofstream::trunc);
}

std::vector<float> parseValues(const std::string& line) {
    std::vector<float> values;
    std::stringstream ss(line.substr(1, line.size() - 2)); // Remove brackets
    std::string value;
    while (std::getline(ss, value, ',')) {
        values.push_back(std::stof(value));
    }
    return values;
}

void Uart_Txt::sendCommand(const LawCommand& command){
    unsigned char message[3];
    message[0] = static_cast<unsigned char>((command.speed * 100 / 8) + 100);
    message[1] = static_cast<unsigned char>(command.steeringAngle + 16);
    message[2] = '\0';
    sendUart(message);
    std::cout << "Sent " <<" : " << (signed char)((command.speed) * 100 / 8) << " / " << (char) message[0] << ", " << command.steeringAngle << " / " << static_cast<char>(message[1]) << std::endl;
}

void Uart_Txt::run(){
    auto period = std::chrono::milliseconds(static_cast<int>(88));
    auto nextSendTime = std::chrono::steady_clock::now() + period;
    while (isOngoing_.load()) {
        auto currentTime = std::chrono::steady_clock::now();
        if (currentTime >= nextSendTime) {
            std::vector<LawCommand> commands = readCommandsFromFile("../lawCommands.txt");
            for (const auto& command : commands) {
                sendCommand(command);
                nextSendTime = currentTime + period;
            }
            clearCommandsFile("../lawCommands.txt");
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
