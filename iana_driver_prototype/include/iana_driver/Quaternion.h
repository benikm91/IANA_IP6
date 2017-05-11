//
// Created by benjamin on 24.04.17.
//

#include "Vector3.h"

#ifndef IANA_DRIVER_QUATERNION_H
#define IANA_DRIVER_QUATERNION_H

namespace Iana
{

    struct Quaternion
    {
        static Quaternion FromMsg(const geometry_msgs::Quaternion& msgs);

        Quaternion(double x, double y, double z, double w) : m_axis(Vector3(x, y, z)), m_w(w)
        { }

        // FROM: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        std::tuple<double, double, double> ToEulerianAngle() const
        {
            double roll, pitch, yaw;
            double ysqr = Y() * Y();

            // roll (x-axis rotation)
            double t0 = +2.0 * (W() * X() + Y() * Z());
            double t1 = +1.0 - 2.0 * (X() * X() + ysqr);
            roll = std::atan2(t0, t1);

            // pitch (y-axis rotation)
            double t2 = +2.0 * (W() * Y() - Z() * X());
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            pitch = std::asin(t2);

            // yaw (z-axis rotation)
            double t3 = +2.0 * (W() * Z() + X() * Y());
            double t4 = +1.0 - 2.0 * (ysqr + Z() * Z());
            yaw = std::atan2(t3, t4);

            return std::make_tuple(roll, pitch, yaw);
        }
    private:
        Vector3 m_axis;
        double m_w;
    public:
        double X() const { return m_axis.X(); }
        double Y() const { return m_axis.Y(); }
        double Z() const { return m_axis.Z(); }
        double W() const { return m_w; }

    };

    Quaternion Quaternion::FromMsg(const geometry_msgs::Quaternion& msgs)
    {
        return Quaternion
        (
            msgs.x,
            msgs.y,
            msgs.z,
            msgs.w
        );
    }

}


#endif //IANA_DRIVER_QUATERNION_H
