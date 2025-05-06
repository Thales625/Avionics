#pragma once

#include "Vector3.hpp"

#include <cmath>

template<typename T>
struct Quaternion {
    T w, x, y, z;

    // ctor
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(T n_w, T n_x, T n_y, T n_z) : w(n_w), x(n_x), y(n_y), z(n_z) {}

    static Quaternion<T> identity() {
        return Quaternion<T>();
    }

    static Quaternion<T> fromEulerAngles(T pitch, T yaw, T roll) {
        const T cp = std::cos(pitch * T(0.5));
        const T sp = std::sin(pitch * T(0.5));
        const T cy = std::cos(yaw * T(0.5));
        const T sy = std::sin(yaw * T(0.5));
        const T cr = std::cos(roll * T(0.5));
        const T sr = std::sin(roll * T(0.5));

        const Quaternion<T> q = {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };

        return q.normalized();
    }

    // methods
    inline Quaternion<T> conjugate() const {
        return {w, -x, -y, -z};
    }

    inline Quaternion<T> normalized() const {
        const T norm = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / norm, x / norm, y / norm, z / norm};
    }
	
    inline Quaternion<T> inversed() const {
        const T norm_sq = w * w + x * x + y * y + z * z;
        if (norm_sq == T(0)) {
            throw std::runtime_error("Cannot invert a quaternion with zero norm.");
        }
        return Quaternion<T>(w / norm_sq, -x / norm_sq, -y / norm_sq, -z / norm_sq);
    }

    inline Vector3<T> rotate(const Vector3<T>& v) const {
        const Quaternion<T> res = (*this) * Quaternion<T>(0, v.x, v.y, v.z) * this->conjugate();
        return {res.x, res.y, res.z};
    }

    // operators
    inline Quaternion<T> operator*(const Quaternion<T>& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    inline Vector3<T> operator*(const Vector3<T>& v) const {
        const Quaternion<T> res = (*this) * Quaternion<T>(0, v.x, v.y, v.z) * this->inversed();
        return Vector3<T>{res.x, res.y, res.z};
    }
};