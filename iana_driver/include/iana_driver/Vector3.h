#ifndef IANA_DRIVER_VECTOR3_H
#define IANA_DRIVER_VECTOR3_H

#include "geometry_msgs/Vector3.h"
typedef geometry_msgs::Vector3 Vector3Message;

namespace Iana
{

    class Vector3 {

    public:
        static Vector3* Zero;
        static Vector3* Up;
        static Vector3* Down;
        static Vector3* Left;
        static Vector3* Right;
        static Vector3* Forward;
        static Vector3* Backward;

    public:
        double x;
        double y;
        double z;

    public:
        Vector3() = delete;

        Vector3(double x, double y, double z) : x(x), y(y), z(z) { }

        Vector3(const Vector3 &v) = default;

        // Here Vector3 instead of Vector3& return type because of immutability of Vector3, is this correct?
        Vector3& operator=(const Vector3 &v) = default;

        Vector3(Vector3 &&) = default;

        Vector3 &operator=(Vector3 &&) = default;

    public:
        friend Vector3 operator+(const Vector3 &l, const Vector3 &r) {
            return Vector3(l.x + r.x, l.y + r.y, l.z + r.z);
        }

        friend Vector3 operator-(const Vector3 &l, const Vector3 &r) {
            return Vector3(l.x - r.x, l.y - r.y, l.z - r.z);
        }

        // can we move this out of Vector3 in C++ (like Monkey Patching in C# or implicit cast from scala??)
        operator Vector3Message() const {
            Vector3Message message;
            message.x = this->x;
            message.y = this->y;
            message.z = this->z;
            return message;
        }
    };

    Vector3* Vector3::Zero = new Vector3(0, 0, 0);
    Vector3* Vector3::Up = new Vector3(0, 1, 0);
    Vector3* Vector3::Down = new Vector3(0, -1, 0);
    Vector3* Vector3::Left = new Vector3(1, 0, 0);
    Vector3* Vector3::Right = new Vector3(-1, 0, 0);
    Vector3* Vector3::Forward = new Vector3(0, 0, 1);
    Vector3* Vector3::Backward = new Vector3(0, 0, -1);

}

#endif
