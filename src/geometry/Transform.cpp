#include<geometry/transform.h>
#include <cmath>

namespace aeroforge::geometry
{
	aeroforge::math::Matrix translation(double x, double y, double z)
	{
		aeroforge::math::Matrix result(4, 4);

		result(0, 0) = 1.0;
		result(1, 1) = 1.0;
		result(2, 2) = 1.0;
		result(3, 3) = 1.0;

		result(0, 3) = x;
		result(1, 3) = y;
		result(2, 3) = z;

		return result;
	}

	aeroforge::math::Matrix rotationX(double angleRadians)
	{
		const double c = std::cos(angleRadians);
		const double s = std::sin(angleRadians);

		aeroforge::math::Matrix result(4, 4);

		result(0, 0) = 1.0;

		result(1, 1) = c;
		result(1, 2) = -s;

		result(2, 1) = s;
		result(2, 2) = c;

		result(3, 3) = 1.0;

		return result;
	}

	aeroforge::math::Matrix rotationY(double angleRadians)
	{
		const double c = std::cos(angleRadians);
		const double s = std::sin(angleRadians);

		aeroforge::math::Matrix result(4, 4);

		result(0, 0) = c;
		result(0, 2) = s;

		result(1, 1) = 1.0;

		result(2, 0) = -s;
		result(2, 2) = c;

		result(3, 3) = 1.0;

		return result;
	}

	aeroforge::math::Matrix rotationZ(double angleRadians)
	{
		const double c = std::cos(angleRadians);
		const double s = std::sin(angleRadians);

		aeroforge::math::Matrix result(4, 4);

		result(0, 0) = c;
		result(0, 1) = -s;

		result(1, 0) = s;
		result(1, 1) = c;

		result(2, 2) = 1.0;
		result(3, 3) = 1.0;

		return result;
	}

	aeroforge::math::Matrix rotationAxis(
		const aeroforge::math::Vector3& axis,
		double angleRadians)
	{
		aeroforge::math::Vector3 n = axis.normalize();

		const double x = n.x;
		const double y = n.y;
		const double z = n.z;

		const double c = std::cos(angleRadians);
		const double s = std::sin(angleRadians);
		const double oneMinusC = 1.0 - c;

		aeroforge::math::Matrix result(4, 4);

		result(0, 0) = c + x * x * oneMinusC;
		result(0, 1) = x * y * oneMinusC - z * s;
		result(0, 2) = x * z * oneMinusC + y * s;

		result(1, 0) = y * x * oneMinusC + z * s;
		result(1, 1) = c + y * y * oneMinusC;
		result(1, 2) = y * z * oneMinusC - x * s;

		result(2, 0) = z * x * oneMinusC - y * s;
		result(2, 1) = z * y * oneMinusC + x * s;
		result(2, 2) = c + z * z * oneMinusC;

		result(3, 3) = 1.0;

		return result;
	}

	/*
	Add to aeroforge::visualization or some kind of rendering path eventually
	Matrix perspective(
		double verticalFovRadians,
		double aspectRatio,
		double nearPlane,
		double farPlane)
	{
		if (aspectRatio <= 0.0)
		{
			throw std::invalid_argument(
				"Perspective aspect ratio must be positive");
		}

		if (nearPlane <= 0.0)
		{
			throw std::invalid_argument(
				"Perspective near plane must be positive");
		}

		if (farPlane <= nearPlane)
		{
			throw std::invalid_argument(
				"Perspective far plane must be greater than near plane");
		}

		if (verticalFovRadians <= 0.0 ||
			verticalFovRadians >= std::numbers::pi)
		{
			throw std::invalid_argument(
				"Perspective field of view must be between 0 and pi radians");
		}

		const double f =
			1.0 / std::tan(verticalFovRadians / 2.0);

		Matrix result(4, 4);

		result(0, 0) = f / aspectRatio;
		result(1, 1) = f;

		result(2, 2) =
			(farPlane + nearPlane) /
			(nearPlane - farPlane);

		result(2, 3) =
			(2.0 * farPlane * nearPlane) /
			(nearPlane - farPlane);

		result(3, 2) = -1.0;

		return result;
	}
	
	Matrix orthographic(
			double left,
			double right,
			double bottom,
			double top,
			double nearPlane,
			double farPlane)
		{
			if (right == left)
			{
				throw std::invalid_argument(
					"Orthographic left and right cannot be equal");
			}

			if (top == bottom)
			{
				throw std::invalid_argument(
					"Orthographic top and bottom cannot be equal");
			}

			if (farPlane == nearPlane)
			{
				throw std::invalid_argument(
					"Orthographic near and far planes cannot be equal");
			}

			Matrix result(4, 4);

			result(0, 0) = 2.0 / (right - left);

			result(1, 1) = 2.0 / (top - bottom);

			result(2, 2) =
				-2.0 / (farPlane - nearPlane);

			result(0, 3) =
				-(right + left) /
				(right - left);

			result(1, 3) =
				-(top + bottom) /
				(top - bottom);

			result(2, 3) =
				-(farPlane + nearPlane) /
				(farPlane - nearPlane);

			result(3, 3) = 1.0;

			return result;
		}
	
	*/
}