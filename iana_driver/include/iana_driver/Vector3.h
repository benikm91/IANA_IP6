#ifndef IANA_DRIVER_VECTOR3_H
#define IANA_DRIVER_VECTOR3_H

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include <math.h>

typedef geometry_msgs::Vector3 Vector3Message;

namespace Iana
{

    class Vector3 {

    public:

        static Vector3 FromMsg(Vector3Message msg) { return Vector3(msg.x, msg.y, msg.z); }
        static Vector3 FromMsg(geometry_msgs::Point msg) { return Vector3(msg.x, msg.y, msg.z); }

    public:
        const static Vector3 Zero;
        const static Vector3 Up;
        const static Vector3 Down;
        const static Vector3 Left;
        const static Vector3 Right;
        const static Vector3 Forward;
        const static Vector3 Backward;

    private:
        double m_x;
        double m_y;
        double m_z;

    public:
        Vector3() = delete;

        Vector3(double x, double y, double z) : m_x(x), m_y(y), m_z(z) { }

        Vector3(const Vector3 &v) = default;

        // Here Vector3 instead of Vector3& return type because of immutability of Vector3, is this correct?
        Vector3& operator=(const Vector3 &v) = default;

        Vector3(Vector3 &&) = default;

        Vector3 &operator=(Vector3 &&) = default;

    public:
        friend Vector3 operator+(const Vector3 &l, const Vector3 &r)
        {
            return Vector3(l.m_x + r.m_x, l.m_y + r.m_y, l.m_z + r.m_z);
        }

        friend Vector3 operator-(const Vector3 &l, const Vector3 &r)
        {
            return Vector3(l.m_x - r.m_x, l.m_y - r.m_y, l.m_z - r.m_z);
        }

        friend Vector3 operator*(const double s, const Vector3 &v)
        {
            return Vector3(v.X() * s, v.Y() * s, v.Z() * s);
        }

        friend std::ostream& operator<< (std::ostream& stream, const Vector3& vector)
        {
            stream << "(" << vector.X() << ", " << vector.Y() << ", " << vector.Z() << ")";
            return stream;
        }

        // can we move this out of Vector3 in C++ (like Monkey Patching in C# or implicit cast from scala??)
        operator Vector3Message() const {
            Vector3Message msg;
            msg.x = m_x;
            msg.y = m_y;
            msg.z = m_z;
            return msg;
        }

        inline double X() const { return m_x; }
        inline double Y() const { return m_y; }
        inline double Z() const { return m_z; }

        Vector3 Negate() { return Vector3(-X(), -Y(), -Z()); }

        double Length() { return sqrt(pow(X(), 2) + pow(Y(), 2) + pow(Z(), 2)); }

    };

    const Vector3 Vector3::Zero(0, 0, 0);
    const Vector3 Vector3::Up(0, 1, 0);
    const Vector3 Vector3::Down(0, -1, 0);
    const Vector3 Vector3::Left(1, 0, 0);
    const Vector3 Vector3::Right(-1, 0, 0);
    const Vector3 Vector3::Forward(0, 0, 1);
    const Vector3 Vector3::Backward(0, 0, -1);

}

#endif
