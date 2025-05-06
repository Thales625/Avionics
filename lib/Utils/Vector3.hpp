#pragma once

#include <cmath>

template <typename T>
struct Vector3 {
	T x, y, z;

	// ctor
	Vector3() : x(0), y(0), z(0) {}
	Vector3(T v) : x(v), y(v), z(v) {}
	Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

	// methods
	inline T Magnitude() const { return sqrt(x * x + y * y + z * z); }

	inline const Vector3<T> Normalized() const {
		const T mag = Magnitude();
		if (mag == 0) return Vector3<T>();
		return Vector3<T>(x / mag, y / mag, z / mag);
	}

	inline const T Dot(const Vector3<T> other) const { return x * other.x + y * other.y + z * other.z; }

	inline Vector3<double> Cross(const Vector3<double> other) const {
		return Vector3<double>(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
	}

	inline T Distance (const Vector3<T> other) const {
		return operator-(other).Magnitude();
	}

	// rotations
	inline const Vector3<T> rotateX(const T thetta) const {
		const float c = cos(thetta);
		const float s = sin(thetta);

		return Vector3<T>(x, y * c + z * s, z * c - y * s);
	}

	inline const Vector3 rotateY(const T thetta) const {
		const float c = cos(thetta);
		const float s = sin(thetta);

		return Vector3<T>(x * c + z * s, y, z * c - x * s);
	}

	inline const Vector3<T> rotateZ(const T thetta) const {
		const float c = cos(thetta);
		const float s = sin(thetta);

		return Vector3<T>(x * c - y * s, y * c + x * s, z);
	}

	// operators
	inline const Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); }

	inline const Vector3<T> operator-(const Vector3<T> &other) const {
		return Vector3<T>(x - other.x, y - other.y, z - other.z);
	}
	inline const Vector3<T> operator+(const Vector3<T> &other) const {
		return Vector3<T>(x + other.x, y + other.y, z + other.z);
	}
	inline const Vector3<T> operator*(const Vector3<T> &other) const {
		return Vector3<T>(x * other.x, y * other.y, z * other.z);
	}
	inline const Vector3<T> operator*(const T &other) const {
		return Vector3<T>(x * other, y * other, z * other);
	}
	inline friend const Vector3<T> operator*(const T &lhs, const Vector3<T> &other) {
		return other * lhs;
	}
	inline const Vector3<T> operator/(const T &other) const {
		return Vector3<T>(x / other, y / other, z / other);
	}

	// compound assignment operations
	inline Vector3<T> &operator+=(const Vector3<T> &other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	inline Vector3<T> &operator-=(const Vector3<T> &other) {
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}
	inline Vector3<T> &operator*=(const T &other) {
		x *= other;
		y *= other;
		z *= other;
		return *this;
	}
	inline Vector3<T> &operator/=(const T &other) {
		x /= other;
		y /= other;
		z /= other;
		return *this;
	}
};
