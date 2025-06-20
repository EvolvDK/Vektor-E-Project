#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

// For the sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SERVER_ADDR "127.0.0.1"
#define SERVER_PORT 12000

int main(int argc, char* argv[])
{
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr;

    char send_buff[1025];
    time_t ticks;

    listenfd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET refers to IPV4, SOCK_STREAM and means we use TCP.

    memset(&serv_addr, '0', sizeof(serv_addr)); // Fills serv_addr with 0s
    memset(send_buff, '0', sizeof(send_buff));  // Fills buffer with 0s

    serv_addr.sin_family        = AF_INET;
    serv_addr.sin_addr.s_addr   = htonl(INADDR_ANY);    // Sets the listening address, htonl makes sure makes sure bytes (here uint32_t) are network ordered.
    serv_addr.sin_port          = htons(SERVER_PORT);   // Sets the listening port, htons makes sure bytes (here uint16_t) are network ordered.

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, 10);   // Start listening, 10 connections maximum.

    while(1)
    {
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);    // Tries the three-way TCP handshake.
        std::cout << "Accepted a connection." << std::endl;

        ticks = time(NULL);
        snprintf(send_buff, sizeof(send_buff), "%.24s\r\n", ctime(&ticks)); // Writes the date & time on buffer

        std::cout << "Wrote " << send_buff << " to client." << std::endl;
        write(connfd, send_buff, strlen(send_buff));

        close(connfd);
        sleep(1);
    }

    return 0;
}