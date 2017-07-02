//
// Created by metataro on 10.05.17.
//

#ifndef IANA_DRIVER_DRIVER_H
#define IANA_DRIVER_DRIVER_H

namespace Iana
{
    class Driver
    {
    public:
        virtual ~Driver() {};
        virtual void Run() = 0;
    };
};

#endif //IANA_DRIVER_DRIVER_H
