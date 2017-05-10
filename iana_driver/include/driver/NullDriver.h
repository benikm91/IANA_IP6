//
// Created by metataro on 10.05.17.
//

#ifndef IANA_DRIVER_NULLDRIVER_H
#define IANA_DRIVER_NULLDRIVER_H

namespace Iana
{
    class NullDriver: public Driver
    {
    public:
        void Run() override {}
    };
};

#endif //IANA_DRIVER_NULLDRIVER_H
