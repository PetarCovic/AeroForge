#include <math/Vector3.h>
#include <math/Matrix.h>
#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace aeroforge::math
{
	Vector3::Vector3()
	{
		this->x = 0.0;
		this->y = 0.0;
		this->z = 0.0;
	}

	Vector3::Vector3(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3 Vector3::add(const Vector3& other) const
	{
		return Vector3(this->x + other.x, this->y + other.y, this->z + other.z);
	}

	Vector3 Vector3::operator+(const Vector3& other) const
	{
		return Vector3(this->x + other.x, this->y + other.y, this->z + other.z);
	}

	Vector3& Vector3::operator+=(const Vector3& other)
	{
		this->x += other.x;
		this->y += other.y;
		this->z += other.z;
		return *this;
	}

	Vector3 Vector3::sub(const Vector3& other) const
	{
		return Vector3(this->x - other.x, this->y - other.y, this->z - other.z);
	}

	Vector3 Vector3::operator-(const Vector3& other) const
	{
		return Vector3(this->x - other.x, this->y - other.y, this->z - other.z);
	}

	Vector3& Vector3::operator-=(const Vector3& other)
	{
		this->x -= other.x;
		this->y -= other.y;
		this->z -= other.z;
		return *this;
	}

	Vector3 Vector3::scale(const double scalar) const
	{
		return Vector3(this->x * scalar, this->y * scalar, this->z * scalar);
	}

	Vector3 Vector3::operator*(const double scalar) const
	{
		return Vector3(this->x * scalar, this->y * scalar, this->z * scalar);
	}

	Vector3& Vector3::operator*=(const double scalar)
	{
		this->x *= scalar;
		this->y *= scalar;
		this->z *= scalar;
		return *this;
	}

	Vector3 Vector3::div(const double divisor) const
	{
		if (divisor == 0)
		{
			throw std::invalid_argument("Divisor cannot be zero");
		}

		return Vector3(this->x / divisor, this->y / divisor, this->z / divisor);
	}

	Vector3 Vector3::operator/(const double divisor) const
	{
		if (divisor == 0)
		{
			throw std::invalid_argument("Divisor cannot be zero");
		}
		return Vector3(this->x / divisor, this->y / divisor, this->z / divisor);
	}

	Vector3& Vector3::operator/=(const double divisor)
	{
		if (divisor == 0)
		{
			throw std::invalid_argument("Divisor cannot be zero");
		}
		this->x /= divisor;
		this->y /= divisor;
		this->z /= divisor;
		return *this;
	}

	Vector3 Vector3::negation() const
	{
		return Vector3(-this->x, -this->y, -this->z);
	}

	Vector3 Vector3::operator-() const
	{
		return Vector3(-this->x, -this->y, -this->z);
	}

	double Vector3::dot(const Vector3& other) const
	{
		return this->x * other.x + this->y * other.y + this->z * other.z;
	}

	Vector3 Vector3::cross(const Vector3& other) const
	{
		return Vector3(this->y * other.z - this->z * other.y, -1 * (this->x * other.z - this->z * other.x), this->x * other.y - this->y * other.x);
	}

	double Vector3::magnitude() const
	{
		return std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
	}

	double Vector3::squaredMagnitude() const
	{
		return this->x * this->x + this->y * this->y + this->z * this->z;
	}

	Vector3 Vector3::normalize() const
	{
		double mag = magnitude();

		if (mag == 0.0)
		{
			throw std::domain_error("Cannot normalize a zero vector");
		}

		return Vector3(this->x / mag, this->y / mag, this->z / mag);
	}

	Vector3 Vector3::project(const Vector3& other) const
	{
		double dotProduct = this->dot(other);
		double otherMagnitudeSquared = other.squaredMagnitude();
		if (otherMagnitudeSquared == 0)
		{
			throw std::domain_error("Cannot project onto a zero vector");
		}
		double scale = dotProduct / otherMagnitudeSquared;
		return other.scale(scale);
	}

	bool Vector3::isFinite() const
	{
		return std::isfinite(this->x) && std::isfinite(this->y) && std::isfinite(this->z);
	}

	bool Vector3::isApprox(const Vector3& other, double epsilon) const
	{
		if (epsilon <= 0)
		{
			throw std::invalid_argument("Epsilon must be positive");
		}

		return std::abs(this->x - other.x) < epsilon && std::abs(this->y - other.y) < epsilon && std::abs(this->z - other.z) < epsilon;
	}

	Vector3 Vector3::unitX()
	{
		return Vector3(1, 0, 0);
	}

	Vector3 Vector3::unitY()
	{
		return Vector3(0, 1, 0);
	}

	Vector3 Vector3::unitZ()
	{
		return Vector3(0, 0, 1);
	}

	Vector3 Vector3::reject(const Vector3& other) const
	{
		Vector3 projection = this->project(other);
		return this->sub(projection);
	}

	bool Vector3::isNormalized() const
	{
		double val = magnitude();

		return std::abs(val - 1.0) < 1e-6;
	}

	Vector3 Vector3::reflect(const Vector3& normal) const
	{
		if (normal.isNormalized())
		{
			double dotProduct = this->dot(normal);
			return this->sub(normal.scale(2 * dotProduct));
		}
		throw std::domain_error("Vector is not normalized");
	}

	Vector3 Vector3::lerp(const Vector3& other, double t) const
	{
		return Vector3(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->z + (other.z - this->z) * t);
	}

	bool Vector3::isZero() const
	{
		return this->x == 0 && this->y == 0 && this->z == 0;
	}

	double& Vector3::operator[](std::size_t index)
	{
		switch (index)
		{
		case 0: return this->x;
		case 1: return this->y;
		case 2: return this->z;
		default: throw std::out_of_range("Index out of range");
		}
	}

	const double& Vector3::operator[](std::size_t index) const
	{
		switch (index)
		{
		case 0: return this->x;
		case 1: return this->y;
		case 2: return this->z;
		default: throw std::out_of_range("Index out of range");
		}
	}

	Vector3 toVector3(const Matrix& matrix)
	{
		if (matrix.rows() != 3 || matrix.columns() != 1)
		{
			throw std::invalid_argument("Matrix must be 3x1 to convert to Vector3");
		}
		return Vector3(matrix(0, 0), matrix(1, 0), matrix(2, 0));
	}

	Vector3 operator*(double scalar, const Vector3& vector)
	{
		return vector * scalar;
	}
}