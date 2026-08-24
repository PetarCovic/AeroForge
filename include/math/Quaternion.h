#pragma once

#include <cstddef>

namespace aeroforge::math
{
	struct Vector3;
	struct Matrix;

	struct Quaternion
	{
		double w;
		double x;
		double y;
		double z;
		Quaternion();
		Quaternion(double w, double x, double y, double z);
		static Quaternion identity();
		Quaternion add(const Quaternion& other)const;
		Quaternion operator+(const Quaternion& other) const;
		Quaternion& operator+=(const Quaternion& other);
		Quaternion sub(const Quaternion& other)const;
		Quaternion operator-(const Quaternion& other) const;
		Quaternion& operator-=(const Quaternion& other);
		Quaternion scale(const double scalar) const;
		Quaternion operator*(const double scalar) const;
		Quaternion& operator*=(const double scalar);
		Quaternion div(const double divisor) const;
		Quaternion operator/(const double divisor) const;
		Quaternion& operator/=(const double divisor);
		double& operator[](std::size_t index);
		const double& operator[](std::size_t index) const;
		double dot(const Quaternion& other) const;
		Quaternion conjugate() const;
		Quaternion inverse() const;
		double norm() const;
		double squaredNorm() const;
		Quaternion& normalize();
		Quaternion normalized() const;
		Quaternion mult(const Quaternion& other) const;
		Quaternion operator*(const Quaternion& other) const;
		Quaternion& operator*=(const Quaternion& other);
		Vector3 rotate(const Vector3& vector) const;
		Vector3 rotateInverse(const Vector3& vector) const;
		Vector3 rotationAxis() const;
		double rotationAngle() const;
		Matrix toRotationMatrix() const;
		static Quaternion fromAxisAngle(const Vector3& axis, double angle);
		static Quaternion fromRotationMatrix(const Matrix& matrix);
		static Quaternion fromRotationX(double angle);
		static Quaternion fromRotationY(double angle);
		static Quaternion fromRotationZ(double angle);
		static Quaternion fromTwoVectors(const Vector3& from, const Vector3& to);
		bool isFinite() const;
		bool isApprox(const Quaternion& other, const double epsilon) const;
		bool isNormalized(const double epsilon) const;
		bool isIdentity(const double epsilon) const;
		bool isSameRotation(const Quaternion& other, const double epsilon) const;
		Quaternion lerp(const Quaternion& other, double t) const; // Component-wise linear interpolation. Not guaranteed to be normalized.
		Quaternion slerp(const Quaternion& other, double t) const; // Normalized linear interpolation. Returns a unit quaternion and uses the shortest path.
		Quaternion nlerp(const Quaternion& other, double t) const; // Spherical linear interpolation. Returns a unit quaternion and uses the shortest path.
		Quaternion derivativeFromAngularVelocity(const Vector3& angularVelocity) const;
	};

	Quaternion operator*(double scalar, const Quaternion& quaternion);
}