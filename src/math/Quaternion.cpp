#include <math/Quaternion.h>
#include <math/Vector3.h>
#include <math/Matrix.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aeroforge::math
{
	Quaternion::Quaternion()
		: w(1.0),
		x(0.0),
		y(0.0),
		z(0.0)
	{
	}

	Quaternion::Quaternion(
		double w,
		double x,
		double y,
		double z)
		: w(w),
		x(x),
		y(y),
		z(z)
	{
	}

	Quaternion Quaternion::identity()
	{
		return Quaternion(
			1.0,
			0.0,
			0.0,
			0.0);
	}

	Quaternion Quaternion::add(
		const Quaternion& other) const
	{
		return Quaternion(
			this->w + other.w,
			this->x + other.x,
			this->y + other.y,
			this->z + other.z);
	}

	Quaternion Quaternion::operator+(
		const Quaternion& other) const
	{
		return this->add(other);
	}

	Quaternion& Quaternion::operator+=(
		const Quaternion& other)
	{
		this->w += other.w;
		this->x += other.x;
		this->y += other.y;
		this->z += other.z;

		return *this;
	}

	Quaternion Quaternion::sub(
		const Quaternion& other) const
	{
		return Quaternion(
			this->w - other.w,
			this->x - other.x,
			this->y - other.y,
			this->z - other.z);
	}

	Quaternion Quaternion::operator-(
		const Quaternion& other) const
	{
		return this->sub(other);
	}

	Quaternion& Quaternion::operator-=(
		const Quaternion& other)
	{
		this->w -= other.w;
		this->x -= other.x;
		this->y -= other.y;
		this->z -= other.z;

		return *this;
	}

	Quaternion Quaternion::scale(
		const double scalar) const
	{
		return Quaternion(
			this->w * scalar,
			this->x * scalar,
			this->y * scalar,
			this->z * scalar);
	}

	Quaternion Quaternion::operator*(
		const double scalar) const
	{
		return this->scale(scalar);
	}

	Quaternion& Quaternion::operator*=(
		const double scalar)
	{
		this->w *= scalar;
		this->x *= scalar;
		this->y *= scalar;
		this->z *= scalar;

		return *this;
	}

	Quaternion Quaternion::div(
		const double divisor) const
	{
		if (divisor == 0.0)
		{
			throw std::invalid_argument(
				"Quaternion divisor cannot be zero");
		}

		return Quaternion(
			this->w / divisor,
			this->x / divisor,
			this->y / divisor,
			this->z / divisor);
	}

	Quaternion Quaternion::operator/(
		const double divisor) const
	{
		return this->div(divisor);
	}

	Quaternion& Quaternion::operator/=(
		const double divisor)
	{
		if (divisor == 0.0)
		{
			throw std::invalid_argument(
				"Quaternion divisor cannot be zero");
		}

		this->w /= divisor;
		this->x /= divisor;
		this->y /= divisor;
		this->z /= divisor;

		return *this;
	}

	double& Quaternion::operator[](
		std::size_t index)
	{
		switch (index)
		{
		case 0:
			return this->w;

		case 1:
			return this->x;

		case 2:
			return this->y;

		case 3:
			return this->z;

		default:
			throw std::out_of_range(
				"Quaternion index out of bounds");
		}
	}

	const double& Quaternion::operator[](
		std::size_t index) const
	{
		switch (index)
		{
		case 0:
			return this->w;

		case 1:
			return this->x;

		case 2:
			return this->y;

		case 3:
			return this->z;

		default:
			throw std::out_of_range(
				"Quaternion index out of bounds");
		}
	}

	double Quaternion::dot(
		const Quaternion& other) const
	{
		return
			this->w * other.w +
			this->x * other.x +
			this->y * other.y +
			this->z * other.z;
	}

	Quaternion Quaternion::conjugate() const
	{
		return Quaternion(
			this->w,
			-this->x,
			-this->y,
			-this->z);
	}

	Quaternion Quaternion::inverse() const
	{
		const double normSquared =
			this->squaredNorm();

		if (normSquared == 0.0)
		{
			throw std::domain_error(
				"Zero quaternion has no inverse");
		}

		return this->conjugate() / normSquared;
	}

	double Quaternion::norm() const
	{
		return std::sqrt(
			this->squaredNorm());
	}

	double Quaternion::squaredNorm() const
	{
		return
			this->w * this->w +
			this->x * this->x +
			this->y * this->y +
			this->z * this->z;
	}

	Quaternion& Quaternion::normalize()
	{
		const double n = this->norm();

		if (n == 0.0)
		{
			throw std::domain_error(
				"Zero quaternion cannot be normalized");
		}

		*this /= n;

		return *this;
	}

	Quaternion Quaternion::normalized() const
	{
		const double n = this->norm();

		if (n == 0.0)
		{
			throw std::domain_error(
				"Zero quaternion cannot be normalized");
		}

		return *this / n;
	}

	Quaternion Quaternion::mult(
		const Quaternion& other) const
	{
		return Quaternion(
			this->w * other.w -
			this->x * other.x -
			this->y * other.y -
			this->z * other.z,

			this->w * other.x +
			this->x * other.w +
			this->y * other.z -
			this->z * other.y,

			this->w * other.y -
			this->x * other.z +
			this->y * other.w +
			this->z * other.x,

			this->w * other.z +
			this->x * other.y -
			this->y * other.x +
			this->z * other.w);
	}

	Quaternion Quaternion::operator*(
		const Quaternion& other) const
	{
		return this->mult(other);
	}

	Quaternion& Quaternion::operator*=(
		const Quaternion& other)
	{
		*this = this->mult(other);

		return *this;
	}

	Vector3 Quaternion::rotate(
		const Vector3& vector) const
	{
		const Quaternion q =
			this->normalized();

		const Quaternion vectorQuaternion(
			0.0,
			vector.x,
			vector.y,
			vector.z);

		const Quaternion result =
			q *
			vectorQuaternion *
			q.conjugate();

		return Vector3(
			result.x,
			result.y,
			result.z);
	}

	Vector3 Quaternion::rotateInverse(
		const Vector3& vector) const
	{
		const Quaternion q =
			this->normalized();

		const Quaternion vectorQuaternion(
			0.0,
			vector.x,
			vector.y,
			vector.z);

		const Quaternion result =
			q.conjugate() *
			vectorQuaternion *
			q;

		return Vector3(
			result.x,
			result.y,
			result.z);
	}

	Vector3 Quaternion::rotationAxis() const
	{
		Quaternion q =
			this->normalized();

		/*
		 * q and -q represent the same rotation.
		 * Prefer the representation whose scalar
		 * component is positive so the returned
		 * angle is the shortest angle in [0, pi].
		 */
		if (q.w < 0.0)
		{
			q *= -1.0;
		}

		const double vectorMagnitude =
			std::sqrt(
				q.x * q.x +
				q.y * q.y +
				q.z * q.z);

		/*
		 * The axis of a zero-angle rotation is
		 * mathematically undefined. Use +X as
		 * the documented conventional axis.
		 */
		if (vectorMagnitude < 1e-12)
		{
			return Vector3::unitX();
		}

		return Vector3(
			q.x / vectorMagnitude,
			q.y / vectorMagnitude,
			q.z / vectorMagnitude);
	}

	double Quaternion::rotationAngle() const
	{
		Quaternion q =
			this->normalized();

		/*
		 * q and -q represent the same rotation.
		 * Canonicalize to the shortest rotation.
		 */
		if (q.w < 0.0)
		{
			q *= -1.0;
		}

		const double vectorMagnitude =
			std::sqrt(
				q.x * q.x +
				q.y * q.y +
				q.z * q.z);

		return 2.0 * std::atan2(
			vectorMagnitude,
			q.w);
	}

	Matrix Quaternion::toRotationMatrix() const
	{
		const Quaternion q =
			this->normalized();

		const double xx = q.x * q.x;
		const double yy = q.y * q.y;
		const double zz = q.z * q.z;

		const double xy = q.x * q.y;
		const double xz = q.x * q.z;
		const double yz = q.y * q.z;

		const double wx = q.w * q.x;
		const double wy = q.w * q.y;
		const double wz = q.w * q.z;

		return Matrix{
			{
				1.0 - 2.0 * (yy + zz),
				2.0 * (xy - wz),
				2.0 * (xz + wy)
			},
			{
				2.0 * (xy + wz),
				1.0 - 2.0 * (xx + zz),
				2.0 * (yz - wx)
			},
			{
				2.0 * (xz - wy),
				2.0 * (yz + wx),
				1.0 - 2.0 * (xx + yy)
			}
		};
	}

	Quaternion Quaternion::fromAxisAngle(
		const Vector3& axis,
		double angle)
	{
		const double axisMagnitude =
			axis.magnitude();

		if (axisMagnitude == 0.0)
		{
			throw std::domain_error(
				"Rotation axis cannot be zero");
		}

		const Vector3 normalizedAxis =
			axis / axisMagnitude;

		const double halfAngle =
			angle * 0.5;

		const double sinHalfAngle =
			std::sin(halfAngle);

		return Quaternion(
			std::cos(halfAngle),
			normalizedAxis.x * sinHalfAngle,
			normalizedAxis.y * sinHalfAngle,
			normalizedAxis.z * sinHalfAngle);
	}

	Quaternion Quaternion::fromRotationMatrix(
		const Matrix& matrix)
	{
		if (matrix.rows() != 3 ||
			matrix.columns() != 3)
		{
			throw std::invalid_argument(
				"Rotation matrix must be 3x3");
		}

		if (!matrix.isFinite())
		{
			throw std::invalid_argument(
				"Rotation matrix must contain only finite values");
		}

		constexpr double epsilon = 1e-9;

		const Matrix orthogonalityCheck =
			matrix.transpose() * matrix;

		const Matrix identity =
			Matrix::identity(3);

		if (!orthogonalityCheck.isApprox(identity, epsilon))
		{
			throw std::invalid_argument(
				"Matrix must be orthogonal");
		}

		if (std::abs(matrix.determinant() - 1.0) > epsilon)
		{
			throw std::invalid_argument(
				"Matrix determinant must be approximately 1");
		}

		const double trace =
			matrix(0, 0) +
			matrix(1, 1) +
			matrix(2, 2);

		double w;
		double x;
		double y;
		double z;

		if (trace > 0.0)
		{
			const double s =
				2.0 *
				std::sqrt(
					std::max(
						0.0,
						trace + 1.0));

			w = 0.25 * s;
			x =
				(matrix(2, 1) -
					matrix(1, 2)) / s;
			y =
				(matrix(0, 2) -
					matrix(2, 0)) / s;
			z =
				(matrix(1, 0) -
					matrix(0, 1)) / s;
		}
		else if (
			matrix(0, 0) > matrix(1, 1) &&
			matrix(0, 0) > matrix(2, 2))
		{
			const double s =
				2.0 *
				std::sqrt(
					std::max(
						0.0,
						1.0 +
						matrix(0, 0) -
						matrix(1, 1) -
						matrix(2, 2)));

			w =
				(matrix(2, 1) -
					matrix(1, 2)) / s;

			x = 0.25 * s;

			y =
				(matrix(0, 1) +
					matrix(1, 0)) / s;

			z =
				(matrix(0, 2) +
					matrix(2, 0)) / s;
		}
		else if (
			matrix(1, 1) > matrix(2, 2))
		{
			const double s =
				2.0 *
				std::sqrt(
					std::max(
						0.0,
						1.0 +
						matrix(1, 1) -
						matrix(0, 0) -
						matrix(2, 2)));

			w =
				(matrix(0, 2) -
					matrix(2, 0)) / s;

			x =
				(matrix(0, 1) +
					matrix(1, 0)) / s;

			y = 0.25 * s;

			z =
				(matrix(1, 2) +
					matrix(2, 1)) / s;
		}
		else
		{
			const double s =
				2.0 *
				std::sqrt(
					std::max(
						0.0,
						1.0 +
						matrix(2, 2) -
						matrix(0, 0) -
						matrix(1, 1)));

			w =
				(matrix(1, 0) -
					matrix(0, 1)) / s;

			x =
				(matrix(0, 2) +
					matrix(2, 0)) / s;

			y =
				(matrix(1, 2) +
					matrix(2, 1)) / s;

			z = 0.25 * s;
		}

		Quaternion result(
			w,
			x,
			y,
			z);

		return result.normalized();
	}

	Quaternion Quaternion::fromRotationX(
		double angle)
	{
		const double halfAngle =
			angle * 0.5;

		return Quaternion(
			std::cos(halfAngle),
			std::sin(halfAngle),
			0.0,
			0.0);
	}

	Quaternion Quaternion::fromRotationY(
		double angle)
	{
		const double halfAngle =
			angle * 0.5;

		return Quaternion(
			std::cos(halfAngle),
			0.0,
			std::sin(halfAngle),
			0.0);
	}

	Quaternion Quaternion::fromRotationZ(
		double angle)
	{
		const double halfAngle =
			angle * 0.5;

		return Quaternion(
			std::cos(halfAngle),
			0.0,
			0.0,
			std::sin(halfAngle));
	}

	Quaternion Quaternion::fromTwoVectors(
		const Vector3& from,
		const Vector3& to)
	{
		const double fromMagnitude =
			from.magnitude();

		if (fromMagnitude == 0.0)
		{
			throw std::domain_error(
				"From vector cannot be zero");
		}

		const double toMagnitude =
			to.magnitude();

		if (toMagnitude == 0.0)
		{
			throw std::domain_error(
				"To vector cannot be zero");
		}

		const Vector3 a =
			from / fromMagnitude;

		const Vector3 b =
			to / toMagnitude;

		const double dot =
			std::clamp(
				a.dot(b),
				-1.0,
				1.0);

		constexpr double epsilon =
			1e-10;

		/*
		 * Same direction:
		 * no rotation required.
		 */
		if (dot > 1.0 - epsilon)
		{
			return Quaternion::identity();
		}

		/*
		 * Opposite directions:
		 * choose any stable axis perpendicular
		 * to the source direction.
		 */
		if (dot < -1.0 + epsilon)
		{
			Vector3 perpendicular;

			if (std::abs(a.x) < 0.9)
			{
				perpendicular =
					a.cross(
						Vector3::unitX());
			}
			else
			{
				perpendicular =
					a.cross(
						Vector3::unitY());
			}

			perpendicular.normalize();

			/*
			 * cos(pi / 2) = 0
			 * sin(pi / 2) = 1
			 */
			return Quaternion(
				0.0,
				perpendicular.x,
				perpendicular.y,
				perpendicular.z);
		}

		const Vector3 cross =
			a.cross(b);

		Quaternion result(
			1.0 + dot,
			cross.x,
			cross.y,
			cross.z);

		return result.normalized();
	}

	bool Quaternion::isFinite() const
	{
		return
			std::isfinite(this->w) &&
			std::isfinite(this->x) &&
			std::isfinite(this->y) &&
			std::isfinite(this->z);
	}

	bool Quaternion::isZero() const
	{
		return w == 0.0
			&& x == 0.0
			&& y == 0.0
			&& z == 0.0;
	}

	bool Quaternion::isApprox(
		const Quaternion& other,
		const double epsilon) const
	{
		if (epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"Epsilon must be greater than zero");
		}

		if (!this->isFinite() ||
			!other.isFinite())
		{
			return false;
		}

		const auto approximatelyEqual =
			[epsilon](
				double a,
				double b)
			{
				const double difference =
					std::abs(a - b);

				const double scale =
					std::max({
						1.0,
						std::abs(a),
						std::abs(b)
						});

				return difference <=
					epsilon * scale;
			};

		return
			approximatelyEqual(
				this->w,
				other.w) &&
			approximatelyEqual(
				this->x,
				other.x) &&
			approximatelyEqual(
				this->y,
				other.y) &&
			approximatelyEqual(
				this->z,
				other.z);
	}

	bool Quaternion::isNormalized(
		const double epsilon) const
	{
		if (epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"Epsilon must be greater than zero");
		}

		if (!this->isFinite())
		{
			return false;
		}

		return std::abs(
			this->squaredNorm() - 1.0)
			<= epsilon;
	}

	bool Quaternion::isIdentity(
		const double epsilon) const
	{
		return this->isSameRotation(
			Quaternion::identity(),
			epsilon);
	}

	bool Quaternion::isSameRotation(
		const Quaternion& other,
		const double epsilon) const
	{
		if (epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"Epsilon must be greater than zero");
		}

		if (!this->isFinite() ||
			!other.isFinite())
		{
			return false;
		}

		if (this->squaredNorm() == 0.0 ||
			other.squaredNorm() == 0.0)
		{
			return false;
		}

		const Quaternion a =
			this->normalized();

		const Quaternion b =
			other.normalized();

		if (a.isApprox(b, epsilon))
		{
			return true;
		}

		const Quaternion negativeB(
			-b.w,
			-b.x,
			-b.y,
			-b.z);

		return a.isApprox(
			negativeB,
			epsilon);
	}

	// Performs component-wise linear interpolation.
	// The returned quaternion is NOT guaranteed to be normalized.
	Quaternion Quaternion::lerp(
		const Quaternion& other,
		double t) const
	{
		/*
		 * Raw quaternion component interpolation.
		 * This does NOT guarantee unit length.
		 */
		return
			*this * (1.0 - t) +
			other * t;
	}

	// Performs normalized linear interpolation between orientations.
	// Returns a normalized quaternion and uses the shortest rotation path.
	Quaternion Quaternion::nlerp(
		const Quaternion& other,
		double t) const
	{
		Quaternion a =
			this->normalized();

		Quaternion b =
			other.normalized();

		/*
		 * q and -q represent the same orientation.
		 * Choose the shortest interpolation path.
		 */
		if (a.dot(b) < 0.0)
		{
			b *= -1.0;
		}

		return a.lerp(
			b,
			t).normalized();
	}

	// Performs spherical linear interpolation between orientations.
	// Returns a normalized quaternion and uses the shortest rotation path.
	Quaternion Quaternion::slerp(
		const Quaternion& other,
		double t) const
	{
		Quaternion a =
			this->normalized();

		Quaternion b =
			other.normalized();

		double dot =
			a.dot(b);

		/*
		 * Use the equivalent quaternion -b when
		 * needed so interpolation takes the
		 * shortest path.
		 */
		if (dot < 0.0)
		{
			b *= -1.0;
			dot = -dot;
		}

		dot = std::clamp(
			dot,
			-1.0,
			1.0);

		/*
		 * Very small angular separations make the
		 * SLERP denominator poorly conditioned.
		 */
		if (dot > 0.9995)
		{
			return a.lerp(
				b,
				t).normalized();
		}

		const double theta =
			std::acos(dot);

		const double sinTheta =
			std::sin(theta);

		if (std::abs(sinTheta) < 1e-12)
		{
			return a.lerp(
				b,
				t).normalized();
		}

		const double weightA =
			std::sin(
				(1.0 - t) * theta) /
			sinTheta;

		const double weightB =
			std::sin(
				t * theta) /
			sinTheta;

		return (
			a * weightA +
			b * weightB
			).normalized();
	}

	Quaternion Quaternion::derivativeFromAngularVelocity(
		const Vector3& angularVelocity) const
	{
		/*
		 * AeroForge convention:
		 *
		 * q_NB maps BODY -> NAVIGATION.
		 *
		 * angularVelocity is the body's angular
		 * velocity relative to navigation,
		 * expressed in BODY coordinates.
		 *
		 * Therefore:
		 *
		 * q_dot = 1/2 * q_NB * omega_B
		 *
		 * where omega_B is represented as the
		 * pure quaternion [0, wx, wy, wz].
		 */

		const Quaternion omega(
			0.0,
			angularVelocity.x,
			angularVelocity.y,
			angularVelocity.z);

		return 0.5 *
			(*this * omega);
	}

	Quaternion operator*(
		double scalar,
		const Quaternion& quaternion)
	{
		return quaternion * scalar;
	}
}