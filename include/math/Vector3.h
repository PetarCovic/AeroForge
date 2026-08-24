#pragma once

#include <cstddef>

namespace aeroforge::math
{
	struct Matrix;

	struct Vector3
	{
		double x;
		double y;
		double z;
		Vector3();
		Vector3(double x, double y, double z);
		Vector3 add(const Vector3& other) const;
		Vector3 operator+(const Vector3& other) const;
		Vector3& operator+=(const Vector3& other);
		Vector3 sub(const Vector3& other) const;
		Vector3 operator-(const Vector3& other) const;
		Vector3& operator-=(const Vector3& other);
		Vector3 scale(const double scalar) const;
		Vector3 operator*(const double scalar) const;
		Vector3& operator*=(const double scalar);
		Vector3 div(const double divisor) const;
		Vector3 operator/(const double divisor) const;
		Vector3& operator/=(const double divisor);
		Vector3 negation() const;
		Vector3 operator-() const;
		double dot(const Vector3& other) const;
		Vector3 cross(const Vector3& other) const;
		double magnitude() const;
		double squaredMagnitude() const;
		Vector3 normalize() const;
		Vector3 project(const Vector3& other) const;
		bool isFinite() const;
		bool isApprox(const Vector3& other, double epsilon) const;
		static Vector3 unitX();
		static Vector3 unitY();
		static Vector3 unitZ();
		Vector3 reject(const Vector3& other) const;
		bool isNormalized() const;
		Vector3 reflect(const Vector3& normal) const;
		Vector3 lerp(const Vector3& other, double t) const;
		double& operator[](std::size_t index);
		const double& operator[](std::size_t index) const;
	};

	Vector3 toVector3(const Matrix& matrix);

	Vector3 operator*(double scalar, const Vector3& vector);
}