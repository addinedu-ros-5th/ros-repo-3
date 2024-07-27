#pragma once

#include <iostream>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

class IpAddress {
public:
    IpAddress() {}

    std::string getLocalIP() const 
    {
        struct ifaddrs *interfaces;
        std::string ipAddress;

        if (getifaddrs(&interfaces) == 0) 
        {
            for (struct ifaddrs *ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) 
            {
                if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) 
                {
                    struct sockaddr_in *sockaddr = (struct sockaddr_in *)ifa->ifa_addr;
                    ipAddress = inet_ntoa(sockaddr->sin_addr);
                    break;
                }
            }
            freeifaddrs(interfaces);
        } 
        else 
        {
            std::cerr << "getifaddrs failed" << std::endl;
        }

        return ipAddress;
    }
};
